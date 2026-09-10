// =====================================================================
//  geom_mtds.h  —  Geometria del brazo MTDS v1, medida sobre el CAD
// =====================================================================
//  UNICA FUENTE DE VERDAD de los parametros geometricos. La incluyen
//  ik_mtds.h y ctrl_mtds.h; si cambia el modelo, se toca SOLO este archivo.
//
//  ---------------------------------------------------------------------
//  PROCEDENCIA DE ESTOS NUMEROS (importante para no volver a equivocarse)
//  ---------------------------------------------------------------------
//  Medidos el 2026-08-20 directamente sobre "Ensamble Brazo Robot v1" en
//  el Hub, via la API de Fusion, con el modelo EN UNA POSE CUALQUIERA
//  (J1=162.39953 deg, J2=181.9387 deg, J3=69.76656 deg, J4=284.88287 deg)
//  y retro-transformados a la pose de referencia (todos los joints en 0)
//  usando la propia cinematica de este header. El procedimiento se validó
//  antes de usarlo: la FK de aca, alimentada con esos rotationValue de
//  Fusion, reproduce la posicion medida del eje del codo con 0.003 mm de
//  error. O sea: la estructura cinematica y los signos coinciden con
//  Fusion, verificado contra el modelo vivo.
//
//  Fuentes exactas de cada numero:
//   - O      : interseccion de los ejes de J1 y J2 en el cuerpo
//              "center (1)" (cara cilindrica r=3.2 sobre Z en x=3.1998,
//              y=5.5000; caras r=3.3698 sobre el eje de J2 en z=-4.7002).
//              Es invariante a la pose: los dos ejes del cardan pasan
//              siempre por ahi.
//   - A_Y    : distancia perpendicular de O al eje del codo (cara
//              cilindrica r=3.8 de "Elbow Support").
//   - B, U0  : del cuerpo "Vastago_Pinza_placeholder_30cm" (cilindro
//              r=0.4, largo 30 cm exacto) dentro de "Muneca Roll":
//              su cara plana de la base da el punto de montaje, y su
//              cara cilindrica da la direccion del vastago.
//
//  ⚠️ CORRECCION RESPECTO DE LA REVISION 17 DEL CONTEXTO. Dos cosas que
//     estaban mal documentadas y quedan corregidas aca:
//      1) B valia (325.2415, 76.2284, 45.6542) mm, |B|=337.16. El valor
//         medido sobre el modelo es (357.6524, 51.6101, -2.2033),
//         |B|=361.36 — 24 mm mas largo y con otra direccion. El valor
//         viejo no cerraba: daba un alcance maximo (A_Y+|B|=680.6 mm)
//         MENOR que la distancia realmente medida al punto de montaje
//         (700.9 mm), lo cual es geometricamente imposible.
//      2) Se documentaba que el eje de J4 (y por lo tanto el del vastago)
//         era Z_cad. NO ES ASI: el eje del vastago es practicamente la
//         PROLONGACION DEL ANTEBRAZO (1.67 deg de B/|B|) y esta a 90.5 deg
//         de Z. Ver U0.
//
//  ✔️ Verificado ademas: el eje de J4 y el eje del vastago son COLINEALES
//     (0.0008 mm de separacion). Consecuencia: el valor de J4 no mueve de
//     lugar ni al punto de montaje ni al eje del vastago — solo hace girar
//     la pinza sobre si misma. Por eso J4 no entra en la FK/IK de mas
//     abajo, y por eso no puede ayudar a resolver el RCM.
// =====================================================================
#ifndef GEOM_MTDS_H
#define GEOM_MTDS_H

#include <math.h>

// ---------------------------------------------------------------------
//  MARCOS DE COORDENADAS
// ---------------------------------------------------------------------
//  CAD   : el del ensamble de Fusion. "Arriba" del brazo = +Y. mm.
//  ROBOT : el que usa el codigo de control. Origen en el centro del
//          cardan, +Z arriba, +X hacia adelante (donde apunta el
//          antebrazo en reposo), +Y lateral (mano derecha).
//
//      X_rob =   x_cad - O_X
//      Y_rob = -(z_cad - O_Z)
//      Z_rob =   y_cad - O_Y
//
//  Es una rotacion propia, no una reflexion (se cuido el signo para no
//  espejar el brazo).
// ---------------------------------------------------------------------
static const float O_X = 32.0f;      // centro del cardan, marco CAD, mm
static const float O_Y = 55.0f;
static const float O_Z = -47.0f;

// ---------------------------------------------------------------------
//  PARAMETROS DEL BRAZO (mm)
// ---------------------------------------------------------------------
//  A_Y : brazo superior. Del centro del cardan al eje del codo, puro
//        sobre +Y del marco del eslabon 1.
static const float A_Y = 343.400f;

//  B_* : antebrazo. Del eje del codo al PUNTO DE MONTAJE DE LA
//        HERRAMIENTA (donde arranca el vastago), en el marco del
//        antebrazo con J3 = 0.
static const float B_X = 357.6524f;
static const float B_Y =  51.6101f;
static const float B_Z =  -2.2033f;

//  U0_* : direccion UNITARIA del vastago en ese mismo marco. Es tambien
//         el eje de J4 (verificado colineal). Practicamente la
//         prolongacion del antebrazo: 1.67 deg de B/|B|.
static const float U0_X = 0.9934421f;
static const float U0_Y = 0.1140048f;
static const float U0_Z = -0.0087004f;

//  L_TOOL : largo del vastago + pinza laparoscopica, del punto de
//           montaje a la punta. Dato de Teo, y confirmado en el CAD
//           (el cilindro mide 300.0 mm exactos).
static const float L_TOOL = 300.0f;

// ---------------------------------------------------------------------
//  C0 = B + L_TOOL * U0  ->  vector del codo a la PUNTA de la
//  herramienta, en el marco del antebrazo. Con esto, la FK de la punta
//  es identica en forma a la del punto de montaje: solo cambia el
//  vector constante. (Se deriva de los de arriba; no editar a mano.)
// ---------------------------------------------------------------------
static const float C0_X = B_X + L_TOOL * U0_X;   // 655.6850
static const float C0_Y = B_Y + L_TOOL * U0_Y;   //  85.8115
static const float C0_Z = B_Z + L_TOOL * U0_Z;   //  -4.8134

// ---------------------------------------------------------------------
//  LIMITES DE RECORRIDO POR JUNTA (rad)
// ---------------------------------------------------------------------
//  ⚠️ TODO — VALORES PROVISORIOS. Teo va a medir los grados reales de
//  cada eje del cardan (pendiente 15 del contexto); hasta entonces se
//  usan los soft-limits que ya estan en el firmware del Mega,
//  convertidos con X_STEPS_PER_DEGREE = 17.77:
//      eje X : +-178 pasos          -> +-10.02 deg
//      eje Y : -1066 .. +178 pasos  -> -59.99 .. +10.02 deg
//      eje Z : +-5000 pasos         -> +-281.4 deg  (practicamente libre)
//
//  ⚠️ TODO 2 — el mapeo motor->junta esta ASUMIDO como X->J1, Y->J2,
//  Z->J3 (por el orden en que aparecen en mega.cpp y por el tamano
//  relativo de los rangos, que calza con "la inclinacion lateral tiene
//  recorrido estricto"). HAY QUE CONFIRMARLO EN EL BRAZO antes de
//  confiar en estos topes: si el mapeo esta cruzado, los topes protegen
//  la junta equivocada, que es peor que no tenerlos.
// ---------------------------------------------------------------------
#define MTDS_DEG2RAD(x) ((float)((x) * 0.01745329252f))

static const float T_MIN[3] = { MTDS_DEG2RAD(-10.02f), MTDS_DEG2RAD(-59.99f), MTDS_DEG2RAD(-281.4f) };
static const float T_MAX[3] = { MTDS_DEG2RAD( 10.02f), MTDS_DEG2RAD( 10.02f), MTDS_DEG2RAD( 281.4f) };

// pasos por grado del Mega (ya confirmado en codigo real)
static const float STEPS_PER_DEGREE = 17.77f;

// ---------------------------------------------------------------------
//  BLOQUES DE ROTACION (las mismas convenciones de signo que Fusion)
// ---------------------------------------------------------------------
//  P(t1) = Rz(-t1) = [[ c, s, 0], [-s, c, 0], [0,0,1]]
//  Q(t2) = Rx(-t2) = [[ 1, 0, 0], [ 0, c, s], [0,-s,c]]
//  Rz(t3)          = [[ c,-s, 0], [ s, c, 0], [0,0,1]]
//
//  Cadena completa, para un vector v del marco del antebrazo:
//      v_cad_rel_O = P(t1) * Q(t2) * Rz(t3) * v
//  Y para un punto:
//      p_cad_rel_O = P(t1) * Q(t2) * ( (0,A_Y,0) + Rz(t3) * v )
// ---------------------------------------------------------------------
inline void mtds_chain(float t1, float t2, float t3,
                       float vx, float vy, float vz, bool addUpperArm,
                       float &ox, float &oy, float &oz)
{
    const float c3 = cosf(t3), s3 = sinf(t3);
    float ax = vx * c3 - vy * s3;
    float ay = vx * s3 + vy * c3;
    float az = vz;
    if (addUpperArm) ay += A_Y;

    const float c2 = cosf(t2), s2 = sinf(t2);
    const float bx = ax;
    const float by = ay * c2 + az * s2;
    const float bz = -ay * s2 + az * c2;

    const float c1 = cosf(t1), s1 = sinf(t1);
    ox = bx * c1 + by * s1;
    oy = -bx * s1 + by * c1;
    oz = bz;
}

// marco CAD (relativo a O) -> marco ROBOT
inline void mtds_cad_to_robot(float dx, float dy, float dz,
                              float &xr, float &yr, float &zr)
{
    xr = dx;  yr = -dz;  zr = dy;
}

#endif // GEOM_MTDS_H
