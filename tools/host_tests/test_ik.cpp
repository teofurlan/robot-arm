#include <cstdio>
#include <cmath>
#include <cstdint>
#include "ik_mtds.h"
int main(){
    int bad=0; float worst=0, worstT=0;
    printf("=== reposo ===\n");
    float x,y,z; fk_mtds(0,0,0,x,y,z); printf("  montaje %.3f %.3f %.3f\n",x,y,z);
    fk_mtds_tip(0,0,0,x,y,z); printf("  punta   %.3f %.3f %.3f\n",x,y,z);
    printf("=== ida y vuelta FK->IK->FK (montaje y punta) ===\n");
    int n=0;
    for(int i=0;i<40;++i)for(int j=0;j<40;++j)for(int k=0;k<20;++k){
        float t1=(i-20)*0.03f, t2=(j-20)*0.05f, t3=(k-10)*0.15f;
        float a,b,c; fk_mtds(t1,t2,t3,a,b,c);
        float o1,o2,o3;
        if(calcular_ik_mtds(a,b,c,t1,t2,t3,o1,o2,o3)){
            float p,q,r; fk_mtds(o1,o2,o3,p,q,r);
            float e=sqrtf((p-a)*(p-a)+(q-b)*(q-b)+(r-c)*(r-c));
            if(e>worst) worst=e;
            ++n;
        } else ++bad;
        fk_mtds_tip(t1,t2,t3,a,b,c);
        if(calcular_ik_mtds_tip(a,b,c,t1,t2,t3,o1,o2,o3)){
            float p,q,r; fk_mtds_tip(o1,o2,o3,p,q,r);
            float e=sqrtf((p-a)*(p-a)+(q-b)*(q-b)+(r-c)*(r-c));
            if(e>worstT) worstT=e;
        } else ++bad;
    }
    printf("  %d poses resueltas | error maximo montaje %.6f mm | punta %.6f mm | no resueltas %d\n",n,worst,worstT,bad);
    printf("=== rechazos correctos ===\n");
    float o1,o2,o3;
    printf("  inalcanzable (5000,0,0): %s\n", calcular_ik_mtds(5000,0,0,0,0,0,o1,o2,o3)?"ACEPTO (MAL)":"rechazado ok");
    printf("  degenerado   (0,0,0)   : %s\n", calcular_ik_mtds(0,0,0,0,0,0,o1,o2,o3)?"ACEPTO (MAL)":"rechazado ok");
    printf("  wrapper con topes, reposo: %s\n", calcular_ik(357.652f,2.203f,395.010f,o1,o2,o3)?"acepto ok":"RECHAZO (MAL)");
    return (worst<1e-2f && worstT<1e-2f)?0:1;
}
