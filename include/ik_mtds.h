// =====================================================================
//  ik_mtds.h  —  Cinematica inversa para el brazo MTDS (v1 real)
// =====================================================================
//  Reemplaza a calcular_ik() de "RCM Robot Arm/src/receiver/receiver.cpp".
//
//  POR QUE: la calcular_ik() original resuelve un brazo antropomorfico
//  clasico (giro de base vertical + hombro y codo COPLANARES, resuelto
//  por teorema del coseno). El brazo MTDS real NO tiene esa anatomia:
//  tiene un CARDAN de 2 GDL en la base — dos ejes perpendiculares que se
//  cortan en un punto — mas el codo. Los ejes del hombro y del codo NO
//  son paralelos, asi que no existe un plano del brazo y el teorema del
//  coseno no aplica.
//
//  Es una cadena Z-X-Z con offsets. Tiene solucion analitica cerrada
//  (nada de solvers iterativos): ~6 llamadas trigonometricas. Corre sin
//  problema en la ESP32.
//
//  ---------------------------------------------------------------------
//  ⚠️ REVISION 2026-08-20 (segunda): los parametros geometricos se
//  mudaron a geom_mtds.h y SE CORRIGIERON contra el modelo vivo de
//  Fusion. El B anterior (325.2415, 76.2284, 45.6542) estaba mal: daba
//  un alcance maximo menor que la distancia realmente medida al punto de
//  montaje, lo cual es imposible. Ver la nota de procedencia en
//  geom_mtds.h. Si tocas la geometria, tocala ALLA, no aca.
//  ---------------------------------------------------------------------
//
//  ⚠️ ESTA IK NO IMPLEMENTA NI PERMITE UN RCM. Con 3 juntas moviendo la
//  recta del vastago y 2 restricciones de RCM quedan 1 GDL controlable y
//  hacen falta 3. Es imposible por conteo de grados de libertad, no por
//  falta de codigo. Ver seccion 17 de CONTEXTO_PROYECTO.md y
//  rcm_explicado.html. Para el control de la v1 usar ctrl_mtds.h.
// =====================================================================
#ifndef IK_MTDS_H
#define IK_MTDS_H

#include "geom_mtds.h"
#include <math.h>

// ---------------------------------------------------------------------
//  CINEMATICA DIRECTA — punto de montaje de la herramienta, marco robot
// ---------------------------------------------------------------------
inline void fk_mtds(float t1, float t2, float t3,
                    float &x_rob, float &y_rob, float &z_rob)
{
    float dx, dy, dz;
    mtds_chain(t1, t2, t3, B_X, B_Y, B_Z, true, dx, dy, dz);
    mtds_cad_to_robot(dx, dy, dz, x_rob, y_rob, z_rob);
}

// CINEMATICA DIRECTA — PUNTA de la herramienta, marco robot
inline void fk_mtds_tip(float t1, float t2, float t3,
                        float &x_rob, float &y_rob, float &z_rob)
{
    float dx, dy, dz;
    mtds_chain(t1, t2, t3, C0_X, C0_Y, C0_Z, true, dx, dy, dz);
    mtds_cad_to_robot(dx, dy, dz, x_rob, y_rob, z_rob);
}

// envuelve un angulo a (-pi, pi]
inline float ik_wrap(float a)
{
    while (a >  (float)M_PI) a -= 2.0f*(float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f*(float)M_PI;
    return a;
}

// ---------------------------------------------------------------------
//  CINEMATICA INVERSA GENERICA
// ---------------------------------------------------------------------
//  Resuelve t1,t2,t3 para llevar el punto definido por el vector
//  (vx,vy,vz) del marco del antebrazo hasta el objetivo dado en MARCO
//  ROBOT. Pasando (B_X,B_Y,B_Z) se resuelve el punto de montaje;
//  pasando (C0_X,C0_Y,C0_Z) se resuelve la punta de la herramienta.
//
//  prev1/2/3 = angulos actuales, para elegir entre las hasta 4 ramas de
//  solucion la mas cercana (continuidad, sin saltos bruscos).
//
//  COMO SE DEDUCE:
//   1. Como los ejes de J1 y J2 se cortan en O, la distancia al objetivo
//      depende SOLO de t3:  |d|^2 = K + Rm*sin(t3 + PHI). Un asin.
//   2. Con t3 conocido, t2 sale de la componente Z: un acos.
//   3. t1 sale por diferencia de atan2.
// ---------------------------------------------------------------------
inline bool ik_mtds_generic(float x_rob, float y_rob, float z_rob,
                            float vx, float vy, float vz,
                            float prev1, float prev2, float prev3,
                            float &t1, float &t2, float &t3)
{
    // marco robot -> marco CAD (relativo a O)
    const float dx = x_rob;
    const float dy = z_rob;
    const float dz = -y_rob;
    const float d2 = dx*dx + dy*dy + dz*dz;

    const float Kb = vx*vx + vy*vy + vz*vz;
    const float K  = A_Y*A_Y + Kb;
    const float Ca = 2.0f * A_Y * vx;
    const float Cb = 2.0f * A_Y * vy;
    const float Rm = sqrtf(Ca*Ca + Cb*Cb);
    if (Rm < 1e-9f) return false;
    const float PHI = atan2f(Cb, Ca);

    const float val = (d2 - K) / Rm;
    if (val > 1.0f || val < -1.0f) return false;   // fuera de alcance
    const float base = asinf(val);
    const float t3_cand[2] = { base - PHI, (float)M_PI - base - PHI };

    bool  found = false;
    float bestCost = 0.0f, b1 = 0.0f, b2 = 0.0f, b3 = 0.0f;

    for (int i = 0; i < 2; ++i) {
        const float T3 = t3_cand[i];
        const float c3 = cosf(T3), s3 = sinf(T3);
        const float cx = vx * c3 - vy * s3;
        const float cy = A_Y + vx * s3 + vy * c3;
        const float cz = vz;

        const float Rc = sqrtf(cy*cy + cz*cz);
        if (Rc < 1e-6f) continue;
        const float q = dz / Rc;
        if (q > 1.0f || q < -1.0f) continue;
        const float lam = atan2f(cy, cz);
        const float ac  = acosf(q);

        for (int k = 0; k < 2; ++k) {
            const float T2 = (k == 0 ? ac : -ac) - lam;
            const float c2 = cosf(T2), s2 = sinf(T2);
            const float mx = cx;
            const float my = cy * c2 + cz * s2;
            if (sqrtf(mx*mx + my*my) < 1e-6f) continue;
            if (sqrtf(dx*dx + dy*dy) < 1e-6f) continue;
            const float T1 = atan2f(my, mx) - atan2f(dy, dx);

            const float e1 = ik_wrap(T1 - prev1);
            const float e2 = ik_wrap(T2 - prev2);
            const float e3 = ik_wrap(T3 - prev3);
            const float cost = e1*e1 + e2*e2 + e3*e3;
            if (!found || cost < bestCost) {
                found = true; bestCost = cost; b1 = T1; b2 = T2; b3 = T3;
            }
        }
    }
    if (!found) return false;
    t1 = b1; t2 = b2; t3 = b3;
    return true;
}

// Objetivo = punto de montaje de la herramienta.
inline bool calcular_ik_mtds(float x_rob, float y_rob, float z_rob,
                             float prev1, float prev2, float prev3,
                             float &t1, float &t2, float &t3)
{
    return ik_mtds_generic(x_rob, y_rob, z_rob, B_X, B_Y, B_Z,
                           prev1, prev2, prev3, t1, t2, t3);
}

// Objetivo = PUNTA de la herramienta.
inline bool calcular_ik_mtds_tip(float x_rob, float y_rob, float z_rob,
                                 float prev1, float prev2, float prev3,
                                 float &t1, float &t2, float &t3)
{
    return ik_mtds_generic(x_rob, y_rob, z_rob, C0_X, C0_Y, C0_Z,
                           prev1, prev2, prev3, t1, t2, t3);
}

// ---------------------------------------------------------------------
//  Chequeo de topes (la IK sola solo verifica ALCANCE, no recorrido)
// ---------------------------------------------------------------------
inline bool ik_within_limits(float t1, float t2, float t3)
{
    return t1 >= T_MIN[0] && t1 <= T_MAX[0]
        && t2 >= T_MIN[1] && t2 <= T_MAX[1]
        && t3 >= T_MIN[2] && t3 <= T_MAX[2];
}

// ---------------------------------------------------------------------
//  WRAPPER DROP-IN — misma firma que la calcular_ik() original.
//  Mantiene internamente la ultima solucion valida para la continuidad.
//  Rechaza tambien lo que caiga fuera de los topes de recorrido.
// ---------------------------------------------------------------------
inline bool calcular_ik(float x, float y, float z,
                        float &t1, float &t2, float &t3)
{
    static float p1 = 0.0f, p2 = 0.0f, p3 = 0.0f;
    float o1, o2, o3;
    if (!calcular_ik_mtds(x, y, z, p1, p2, p3, o1, o2, o3)) return false;
    if (!ik_within_limits(o1, o2, o3)) return false;
    p1 = o1; p2 = o2; p3 = o3;
    t1 = o1; t2 = o2; t3 = o3;
    return true;
}

// Posicion de reposo (todos los angulos en cero), marco robot, mm.
// Montaje: 357.652 / 2.203 / 395.010   Punta: 655.685 / 4.813 / 429.212
#endif // IK_MTDS_H
