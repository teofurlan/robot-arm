#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <ctime>
#include "ctrl_mtds.h"

static int fails = 0;
static void CHECK(bool ok, const char *what, double got=0, double want=0){
    if(!ok){ printf("  FALLA: %s (got %.9g, want %.9g)\n", what, got, want); ++fails; }
}

int main(){
    printf("=== 1. FK: pose de reposo (marco robot, mm) ===\n");
    float tip[3], mnt[3], ax[3];
    ctrl_tip(0,0,0,tip); ctrl_mount(0,0,0,mnt); ctrl_axis(0,0,0,ax);
    printf("  montaje = (%.4f, %.4f, %.4f)   |.| = %.4f\n", mnt[0],mnt[1],mnt[2],
           sqrtf(mnt[0]*mnt[0]+mnt[1]*mnt[1]+mnt[2]*mnt[2]));
    printf("  punta   = (%.4f, %.4f, %.4f)\n", tip[0],tip[1],tip[2]);
    printf("  eje     = (%.6f, %.6f, %.6f)\n", ax[0],ax[1],ax[2]);
    // la punta debe estar a L_TOOL del montaje, sobre el eje
    float dd[3] = { tip[0]-mnt[0], tip[1]-mnt[1], tip[2]-mnt[2] };
    float dn = sqrtf(dd[0]*dd[0]+dd[1]*dd[1]+dd[2]*dd[2]);
    CHECK(fabsf(dn - L_TOOL) < 1e-2f, "|punta-montaje| == L_TOOL", dn, L_TOOL);
    float cross = fabsf(dd[0]/dn*ax[0] + dd[1]/dn*ax[1] + dd[2]/dn*ax[2]);
    CHECK(fabsf(cross-1.0f) < 1e-5f, "punta sobre el eje del vastago", cross, 1.0);

    printf("\n=== 2. Largos de eslabon constantes en cualquier pose ===\n");
    float worstA=0, worstB=0;
    for(int i=0;i<2000;++i){
        float t1=(float)((i*37%360)-180)*0.01745f*0.5f;
        float t2=(float)((i*53%360)-180)*0.01745f*0.5f;
        float t3=(float)((i*91%360)-180)*0.01745f;
        float m[3]; ctrl_mount(t1,t2,t3,m);
        float e[3]; { float dx,dy,dz; mtds_chain(t1,t2,0,0,0,0,true,dx,dy,dz); mtds_cad_to_robot(dx,dy,dz,e[0],e[1],e[2]); }
        float la = sqrtf(e[0]*e[0]+e[1]*e[1]+e[2]*e[2]);
        float lb = sqrtf((m[0]-e[0])*(m[0]-e[0])+(m[1]-e[1])*(m[1]-e[1])+(m[2]-e[2])*(m[2]-e[2]));
        worstA = fmaxf(worstA, fabsf(la-A_Y));
        worstB = fmaxf(worstB, fabsf(lb-sqrtf(B_X*B_X+B_Y*B_Y+B_Z*B_Z)));
    }
    printf("  error maximo del brazo superior: %.6f mm | del antebrazo: %.6f mm\n", worstA, worstB);
    CHECK(worstA<1e-2f && worstB<1e-2f, "largos constantes", fmaxf(worstA,worstB), 0);

    printf("\n=== 3. Jacobiano analitico vs diferencias finitas ===\n");
    float worstJ = 0;
    for(int i=0;i<500;++i){
        float t1=(float)((i*29%200)-100)*0.01745f*0.3f;
        float t2=(float)((i*61%200)-100)*0.01745f*0.5f;
        float t3=(float)((i*113%360)-180)*0.01745f;
        float J[3][3]; ctrl_jacobian(t1,t2,t3,J);
        const float h=1e-4f;
        for(int j=0;j<3;++j){
            float p[3]={t1,t2,t3}, q[3]={t1,t2,t3};
            p[j]+=h; q[j]-=h;
            float a[3],b[3]; ctrl_tip(p[0],p[1],p[2],a); ctrl_tip(q[0],q[1],q[2],b);
            for(int r=0;r<3;++r){
                float fd=(a[r]-b[r])/(2*h);
                worstJ = fmaxf(worstJ, fabsf(fd - J[r][j]));
            }
        }
    }
    printf("  discrepancia maxima: %.5f mm/rad (sobre magnitudes de ~700 mm/rad)\n", worstJ);
    CHECK(worstJ < 3.0f, "jacobiano correcto (tolerancia de ruido float32; la formula se verifico exacta en doble precision)", worstJ, 0);

    printf("\n=== 4. Seguimiento: la punta se mueve donde se pide ===\n");
    CtrlState s; ctrl_init(s);
    // dejar que el filtro se asiente y medir la direccion del movimiento
    for(int m=0;m<2;++m){
        CtrlMode mode = (m==0)?CTRL_TOOL:CTRL_WORLD;
        ctrl_init(s);
        float p0[3]; ctrl_tip(s.t1,s.t2,s.t3,p0);
        float want[3];
        if(mode==CTRL_TOOL){ float u[3]; ctrl_axis(s.t1,s.t2,s.t3,u); want[0]=u[0];want[1]=u[1];want[2]=u[2]; }
        else { want[0]=1;want[1]=0;want[2]=0; }
        for(int k=0;k<120;++k) ctrl_step(s, mode, 1.0f,0,0, 0,0, 0.02f);
        float p1[3]; ctrl_tip(s.t1,s.t2,s.t3,p1);
        float d[3]={p1[0]-p0[0],p1[1]-p0[1],p1[2]-p0[2]};
        float n=sqrtf(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
        float cosang=(d[0]*want[0]+d[1]*want[1]+d[2]*want[2])/n;
        printf("  %-11s desplazamiento %.2f mm, angulo con lo pedido %.2f deg  (limitado=%d singular=%d)\n",
               mode==CTRL_TOOL?"CTRL_TOOL":"CTRL_WORLD", n, acosf(fmaxf(-1.f,fminf(1.f,cosang)))*57.2958f,
               (int)s.limited, (int)s.singular);
        CHECK(n > 1.0f, "se movio algo", n, 1.0);
        CHECK(cosang > 0.90f, "se movio en la direccion pedida", cosang, 1.0);
    }

    printf("\n=== 5. Topes de recorrido: nunca se violan ===\n");
    bool viol=false; int hits=0;
    unsigned seed=12345;
    ctrl_init(s);
    for(int k=0;k<200000;++k){
        seed = seed*1103515245u + 12345u;
        float r1 = (float)((seed>>16)&0x7fff)/16383.5f - 1.0f;
        seed = seed*1103515245u + 12345u;
        float r2 = (float)((seed>>16)&0x7fff)/16383.5f - 1.0f;
        seed = seed*1103515245u + 12345u;
        float r3 = (float)((seed>>16)&0x7fff)/16383.5f - 1.0f;
        ctrl_step(s, (k%2)?CTRL_TOOL:CTRL_WORLD, r1,r2,r3, r1, 0.5f, 0.02f);
        if(s.limited) ++hits;
        const float e=1e-5f;
        if(s.t1<T_MIN[0]-e||s.t1>T_MAX[0]+e||s.t2<T_MIN[1]-e||s.t2>T_MAX[1]+e||s.t3<T_MIN[2]-e||s.t3>T_MAX[2]+e){
            viol=true; printf("  VIOLACION en k=%d: %.6f %.6f %.6f\n",k,s.t1,s.t2,s.t3); break;
        }
    }
    printf("  200.000 ciclos con comando aleatorio: violaciones=%d, ciclos recortados=%d\n",(int)viol,hits);
    CHECK(!viol, "topes respetados", 0,0);

    printf("\n=== 6. Tope de velocidad por junta ===\n");
    ctrl_init(s);
    float maxstep=0;
    for(int k=0;k<3000;++k){
        float a=s.t1,b=s.t2,c=s.t3;
        ctrl_step(s, CTRL_TOOL, 1.0f,1.0f,1.0f, 0,0, 0.02f);
        maxstep = fmaxf(maxstep, fmaxf(fabsf(s.t1-a), fmaxf(fabsf(s.t2-b), fabsf(s.t3-c))));
    }
    printf("  paso maximo por junta: %.5f deg (tope %.2f deg)\n", maxstep*57.2958f, CTRL_DTHETA_MAX);
    CHECK(maxstep*57.2958f <= CTRL_DTHETA_MAX + 1e-3f, "velocidad limitada", maxstep*57.2958f, CTRL_DTHETA_MAX);

    printf("\n=== 7. Costo de computo ===\n");
    ctrl_init(s);
    const int N=2000000;
    clock_t t0=clock();
    volatile float sink=0;
    for(int k=0;k<N;++k){
        ctrl_step(s, CTRL_TOOL, 0.7f, -0.3f, 0.45f, 0.2f, 0.5f, 0.02f);
        sink += s.t1;
        if((k&1023)==1023) ctrl_init(s);
    }
    double us = (double)(clock()-t0)/CLOCKS_PER_SEC*1e6/N;
    printf("  %.3f us por ciclo en x86 (sink=%.3f)\n", us, (double)sink);
    printf("  incluso 50x mas lento en la ESP32 = %.1f us, sobre un presupuesto de 20.000 us (%.3f%%)\n",
           us*50, us*50/20000.0*100.0);

    printf("\n=== 8. Trama serie ===\n");
    ctrl_init(s); s.t1=0.1f; s.t2=-0.4f; s.t3=0.9f; s.roll=-33.5f; s.grip=0.6f;
    char buf[64]; ctrl_frame(s, buf, sizeof buf);
    printf("  %s", buf);

    printf("\n%s  (%d fallas)\n", fails? "HAY FALLAS" : "TODO OK", fails);
    return fails?1:0;
}
