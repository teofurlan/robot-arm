// =====================================================================
//  ctrl_mtds.h  —  Control por herramienta para el brazo MTDS v1
// =====================================================================
//  QUE HACE Y POR QUE
//  ------------------
//  Hoy el firmware "robot arm" mueve un motor por cada eje del joystick.
//  Eso es lo menos intuitivo posible: el operador tiene que pensar en
//  motores, no en la herramienta. Este header cambia eso: los joysticks
//  comandan el movimiento de la PUNTA DE LA HERRAMIENTA, y el software
//  reparte ese movimiento entre J1, J2 y J3.
//
//  ⚠️ ESTE HEADER NO IMPLEMENTA UN RCM, A PROPOSITO. Con 3 juntas que
//  mueven la recta del vastago y 2 restricciones que impone un RCM,
//  quedan 1 GDL controlable y hacen falta 3: es imposible, y no por
//  falta de codigo (ver seccion 17 de CONTEXTO_PROYECTO.md y
//  rcm_explicado.html). La v1 es un brazo teleoperado de 3 ejes y este
//  control es lo mejor que se puede hacer con ese hardware. El punto de
//  pivote de la herramienta se va a mover; eso es esperable y correcto.
//
//  DOS MODOS
//  ---------
//  CTRL_TOOL   La velocidad pedida se interpreta en el marco DE LA
//              HERRAMIENTA. Es el modo recomendado: se siente como tener
//              el instrumento en la mano.
//                 a = avanzar / retroceder a lo largo del vastago
//                 b = barrer la punta hacia un costado
//                 c = barrer la punta hacia arriba / abajo
//              (b y c, con la base quieta, hacen pivotear el vastago:
//               es el gesto de "apuntar".)
//
//  CTRL_WORLD  La velocidad pedida se interpreta en el marco ROBOT:
//                 a = +X adelante,  b = +Y lateral,  c = +Z arriba
//              Mas predecible para pruebas y calibracion; menos
//              "quirurgico" para manejar.
//
//  COMO FUNCIONA (en una linea): jacobiano de la punta + minimos
//  cuadrados amortiguados (damped least squares), o sea, "movete lo mas
//  parecido posible a lo que te pido, sin volverte loco cerca de una
//  singularidad".
//
//  SEGURIDAD: topes por junta, limite de velocidad por junta, suavizado
//  del comando, y zona muerta del joystick. Si el paso pedido no se
//  puede respetar, se recorta — nunca se ignora el tope.
//
//  COSTO: ~30 llamadas trigonometricas y una inversa de 3x3 por ciclo.
//  Medido en x86: ver el test. Sobra para el lazo de 20 ms a 50 Hz.
// =====================================================================
#ifndef CTRL_MTDS_H
#define CTRL_MTDS_H

#include "geom_mtds.h"
#include <math.h>

// ---------------------------------------------------------------------
//  AJUSTES (los unicos numeros que conviene tocar para calibrar el tacto)
// ---------------------------------------------------------------------
static const float CTRL_TIP_SPEED   = 60.0f;   // mm/s a fondo de joystick
static const float CTRL_ROLL_SPEED  = 90.0f;   // deg/s a fondo, para J4
static const float CTRL_ROLL_LIMIT  = 90.0f;   // deg, medio recorrido util del SG5010
static const float CTRL_DEADZONE    = 0.08f;   // 0..1, zona muerta
static const float CTRL_EXPO        = 0.45f;   // 0 = lineal, 1 = muy suave al centro
static const float CTRL_SMOOTH      = 0.25f;   // filtro del comando, 0..1 (chico = mas suave)
static const float CTRL_DTHETA_MAX  = 0.9f;    // deg por ciclo, tope de velocidad por junta
static const float CTRL_LAMBDA      = 0.06f;   // amortiguacion base (adimensional)
static const float CTRL_DET_REF     = 1.0e8f;  // referencia de |det J| para subir la amortiguacion

enum CtrlMode { CTRL_TOOL = 0, CTRL_WORLD = 1 };

// ---------------------------------------------------------------------
//  ESTADO. Vive entre ciclos; se inicializa una vez con ctrl_init().
// ---------------------------------------------------------------------
typedef struct {
    float t1, t2, t3;      // rad, angulos de las juntas
    float roll;            // deg, J4
    float grip;            // 0..1, apertura de pinza
    float fa, fb, fc;      // comando filtrado (uso interno)
    bool  limited;         // true si en el ultimo ciclo se toco un tope
    bool  singular;        // true si el ultimo ciclo estuvo cerca de una singularidad
} CtrlState;

inline void ctrl_init(CtrlState &s)
{
    s.t1 = s.t2 = s.t3 = 0.0f;
    s.roll = 0.0f;
    s.grip = 0.0f;
    s.fa = s.fb = s.fc = 0.0f;
    s.limited = false;
    s.singular = false;
}

// ---------------------------------------------------------------------
//  CORTE INMEDIATO. Pone el comando filtrado en cero de golpe, sin dejarlo
//  decaer. Es lo que hay que llamar cuando se cae el enlace: si solo se
//  manda comando cero, el filtro de suavizado tarda ~0,6 s en apagarse y el
//  brazo sigue moviendose por inercia todo ese tiempo (encontrado por el
//  test del receiver, no en la teoria). Para un watchdog eso no sirve.
// ---------------------------------------------------------------------
inline void ctrl_stop(CtrlState &s)
{
    s.fa = 0.0f; s.fb = 0.0f; s.fc = 0.0f;
}

// ---------------------------------------------------------------------
//  FK de la PUNTA de la herramienta, en marco robot (mm)
// ---------------------------------------------------------------------
inline void ctrl_tip(float t1, float t2, float t3, float out[3])
{
    float dx, dy, dz;
    mtds_chain(t1, t2, t3, C0_X, C0_Y, C0_Z, true, dx, dy, dz);
    mtds_cad_to_robot(dx, dy, dz, out[0], out[1], out[2]);
}

// FK del punto de montaje (arranque del vastago), marco robot
inline void ctrl_mount(float t1, float t2, float t3, float out[3])
{
    float dx, dy, dz;
    mtds_chain(t1, t2, t3, B_X, B_Y, B_Z, true, dx, dy, dz);
    mtds_cad_to_robot(dx, dy, dz, out[0], out[1], out[2]);
}

// direccion unitaria del vastago, marco robot
inline void ctrl_axis(float t1, float t2, float t3, float out[3])
{
    float dx, dy, dz;
    mtds_chain(t1, t2, t3, U0_X, U0_Y, U0_Z, false, dx, dy, dz);
    mtds_cad_to_robot(dx, dy, dz, out[0], out[1], out[2]);
    const float n = sqrtf(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
    if (n > 1e-9f) { out[0] /= n; out[1] /= n; out[2] /= n; }
}

// ---------------------------------------------------------------------
//  JACOBIANO ANALITICO de la punta: J[fila=eje robot][col=junta], mm/rad
// ---------------------------------------------------------------------
inline void ctrl_jacobian(float t1, float t2, float t3, float J[3][3])
{
    const float c1 = cosf(t1), s1 = sinf(t1);
    const float c2 = cosf(t2), s2 = sinf(t2);
    const float c3 = cosf(t3), s3 = sinf(t3);

    // w = (0,A_Y,0) + Rz(t3)*C0        (marco del eslabon 2)
    const float wx = C0_X*c3 - C0_Y*s3;
    const float wy = A_Y + C0_X*s3 + C0_Y*c3;
    const float wz = C0_Z;
    // dw/dt3 = Rz'(t3)*C0
    const float wx3 = -C0_X*s3 - C0_Y*c3;
    const float wy3 =  C0_X*c3 - C0_Y*s3;
    const float wz3 = 0.0f;

    // m = Q(t2)*w  y  dm/dt2 = Q'(t2)*w
    const float mx = wx,  my =  wy*c2 + wz*s2;
    const float mx2 = 0.0f, my2 = -wy*s2 + wz*c2, mz2 = -wy*c2 - wz*s2;
    // dm/dt3 = Q(t2)*dw/dt3
    const float mx3 = wx3, my3 = wy3*c2 + wz3*s2, mz3 = -wy3*s2 + wz3*c2;

    // d = P(t1)*m ; derivadas en marco CAD
    // dd/dt1 = P'(t1)*m
    const float a1x = -mx*s1 + my*c1;
    const float a1y = -mx*c1 - my*s1;
    const float a1z = 0.0f;
    // dd/dt2 = P(t1)*dm/dt2
    const float a2x = mx2*c1 + my2*s1;
    const float a2y = -mx2*s1 + my2*c1;
    const float a2z = mz2;
    // dd/dt3 = P(t1)*dm/dt3
    const float a3x = mx3*c1 + my3*s1;
    const float a3y = -mx3*s1 + my3*c1;
    const float a3z = mz3;

    // CAD -> ROBOT:  Xr = dx, Yr = -dz, Zr = dy
    J[0][0] = a1x;  J[0][1] = a2x;  J[0][2] = a3x;
    J[1][0] = -a1z; J[1][1] = -a2z; J[1][2] = -a3z;
    J[2][0] = a1y;  J[2][1] = a2y;  J[2][2] = a3y;
}

// ---------------------------------------------------------------------
//  utilidades chicas
// ---------------------------------------------------------------------
inline float ctrl_clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

inline float ctrl_shape(float v)   // zona muerta + curva expo
{
    const float a = fabsf(v);
    if (a <= CTRL_DEADZONE) return 0.0f;
    float u = (a - CTRL_DEADZONE) / (1.0f - CTRL_DEADZONE);
    u = (1.0f - CTRL_EXPO) * u + CTRL_EXPO * u * u * u;
    return (v < 0.0f) ? -u : u;
}

// ---------------------------------------------------------------------
//  PASO DE CONTROL. Llamar una vez por ciclo del lazo (50 Hz -> dt=0.02).
//
//  a, b, c : comandos normalizados -1..1 (crudos del joystick; la zona
//            muerta y la curva se aplican adentro).
//            CTRL_TOOL  -> a=avance, b=lateral, c=arriba/abajo
//            CTRL_WORLD -> a=+X, b=+Y, c=+Z (marco robot)
//  rollCmd : -1..1, velocidad de giro de J4
//  gripCmd : 0..1 absoluto (el potenciometro del control remoto)
//
//  Devuelve false si no se pudo mover nada (comando nulo o bloqueado).
// ---------------------------------------------------------------------
inline bool ctrl_step(CtrlState &s, CtrlMode mode,
                      float a, float b, float c,
                      float rollCmd, float gripCmd, float dt)
{
    s.limited = false;
    s.singular = false;

    // --- pinza y roll: directos, no pasan por el jacobiano ---
    s.grip = ctrl_clampf(gripCmd, 0.0f, 1.0f);
    // El roll lo hace un servo SG5010, que NO da vueltas completas: tiene ~180
    // grados utiles. Por eso se RECORTA a +-90 en vez de envolver el angulo —
    // envolverlo mandaria al servo un salto de 360 grados que fisicamente no
    // puede hacer, y ademas cruzaria el tope de golpe.
    // TODO: confirmar el recorrido real del SG5010 montado y ajustar CTRL_ROLL_LIMIT.
    s.roll = ctrl_clampf(s.roll + ctrl_shape(rollCmd) * CTRL_ROLL_SPEED * dt,
                         -CTRL_ROLL_LIMIT, CTRL_ROLL_LIMIT);

    // --- comando: forma + suavizado ---
    const float ta = ctrl_shape(a), tb = ctrl_shape(b), tc = ctrl_shape(c);
    s.fa += CTRL_SMOOTH * (ta - s.fa);
    s.fb += CTRL_SMOOTH * (tb - s.fb);
    s.fc += CTRL_SMOOTH * (tc - s.fc);

    if (fabsf(s.fa) < 1e-4f && fabsf(s.fb) < 1e-4f && fabsf(s.fc) < 1e-4f)
        return false;

    // --- velocidad pedida de la punta, en marco robot (mm/s) ---
    float v[3];
    if (mode == CTRL_WORLD) {
        v[0] = s.fa * CTRL_TIP_SPEED;
        v[1] = s.fb * CTRL_TIP_SPEED;
        v[2] = s.fc * CTRL_TIP_SPEED;
    } else {
        // base ortonormal atada a la herramienta: u (eje), e1, e2
        float u[3]; ctrl_axis(s.t1, s.t2, s.t3, u);
        float r[3] = { 0.0f, 0.0f, 1.0f };
        if (fabsf(u[2]) > 0.9f) { r[0] = 1.0f; r[2] = 0.0f; }
        float e1[3];
        const float d = r[0]*u[0] + r[1]*u[1] + r[2]*u[2];
        e1[0] = r[0] - d*u[0]; e1[1] = r[1] - d*u[1]; e1[2] = r[2] - d*u[2];
        float n = sqrtf(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
        if (n < 1e-6f) return false;
        e1[0] /= n; e1[1] /= n; e1[2] /= n;
        const float e2[3] = { u[1]*e1[2] - u[2]*e1[1],
                              u[2]*e1[0] - u[0]*e1[2],
                              u[0]*e1[1] - u[1]*e1[0] };
        for (int i = 0; i < 3; ++i)
            v[i] = (s.fa*u[i] + s.fb*e1[i] + s.fc*e2[i]) * CTRL_TIP_SPEED;
    }

    // --- jacobiano y minimos cuadrados amortiguados ---
    float J[3][3];
    ctrl_jacobian(s.t1, s.t2, s.t3, J);

    // A = J*J^T + lambda^2 * I    (3x3 simetrica)
    float A[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            A[i][j] = J[i][0]*J[j][0] + J[i][1]*J[j][1] + J[i][2]*J[j][2];

    // amortiguacion adaptativa: cuanto mas cerca de la singularidad
    // (|det J| chico), mas amortiguacion. Escala con el tamano del brazo.
    const float detJ = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1])
                     - J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0])
                     + J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);
    float k = fabsf(detJ) / CTRL_DET_REF;
    if (k > 1.0f) k = 1.0f;
    const float lam = CTRL_LAMBDA * (1.0f + 24.0f * (1.0f - k));
    if (k < 0.15f) s.singular = true;
    const float lam2 = lam * lam * CTRL_DET_REF / 1.0e4f;  // escala a mm^2
    A[0][0] += lam2; A[1][1] += lam2; A[2][2] += lam2;

    // resolver A*y = v (3x3 por cofactores; A es simetrica definida positiva)
    const float c00 = A[1][1]*A[2][2] - A[1][2]*A[2][1];
    const float c01 = A[0][2]*A[2][1] - A[0][1]*A[2][2];
    const float c02 = A[0][1]*A[1][2] - A[0][2]*A[1][1];
    const float det = A[0][0]*c00 + A[1][0]*c01 + A[2][0]*c02;
    if (fabsf(det) < 1e-12f) return false;
    const float c11 = A[0][0]*A[2][2] - A[0][2]*A[2][0];
    const float c12 = A[0][2]*A[1][0] - A[0][0]*A[1][2];
    const float c22 = A[0][0]*A[1][1] - A[0][1]*A[1][0];
    const float y[3] = {
        (c00*v[0] + c01*v[1] + c02*v[2]) / det,
        (c01*v[0] + c11*v[1] + c12*v[2]) / det,
        (c02*v[0] + c12*v[1] + c22*v[2]) / det
    };
    // dtheta = J^T * y * dt
    float dth[3];
    for (int j = 0; j < 3; ++j)
        dth[j] = (J[0][j]*y[0] + J[1][j]*y[1] + J[2][j]*y[2]) * dt;

    // --- tope de velocidad por junta: se escala el paso COMPLETO, para
    //     no torcer la direccion del movimiento pedido ---
    const float dmax = MTDS_DEG2RAD(CTRL_DTHETA_MAX);
    float worst = 1.0f;
    for (int j = 0; j < 3; ++j) {
        const float m = fabsf(dth[j]) / dmax;
        if (m > worst) worst = m;
    }
    if (worst > 1.0f) {
        for (int j = 0; j < 3; ++j) dth[j] /= worst;
        s.limited = true;
    }

    // --- topes de recorrido: se escala el paso para quedarse adentro ---
    const float th[3] = { s.t1, s.t2, s.t3 };
    float scale = 1.0f;
    for (int j = 0; j < 3; ++j) {
        const float nx = th[j] + dth[j];
        if (nx > T_MAX[j] && dth[j] > 1e-12f) {
            const float f = (T_MAX[j] - th[j]) / dth[j];
            if (f < scale) scale = (f > 0.0f) ? f : 0.0f;
        } else if (nx < T_MIN[j] && dth[j] < -1e-12f) {
            const float f = (T_MIN[j] - th[j]) / dth[j];
            if (f < scale) scale = (f > 0.0f) ? f : 0.0f;
        }
    }
    if (scale < 1.0f) s.limited = true;

    s.t1 = ctrl_clampf(th[0] + dth[0]*scale, T_MIN[0], T_MAX[0]);
    s.t2 = ctrl_clampf(th[1] + dth[1]*scale, T_MIN[1], T_MAX[1]);
    s.t3 = ctrl_clampf(th[2] + dth[2]*scale, T_MIN[2], T_MAX[2]);
    return scale > 0.0f;
}

// ---------------------------------------------------------------------
//  Trama serie para el Mega: <T1,T2,T3,Roll,Pinza>
//  Mismo formato que ya espera "RCM Robot Arm" (angulos absolutos en
//  grados; el Mega los convierte a pasos con STEPS_PER_DEGREE).
//  buf debe tener al menos 64 bytes.
// ---------------------------------------------------------------------
inline void ctrl_frame(const CtrlState &s, char *buf, int n)
{
    snprintf(buf, (size_t)n, "<%.2f,%.2f,%.2f,%.1f,%d>\n",
             (double)(s.t1 * 57.2957795f),
             (double)(s.t2 * 57.2957795f),
             (double)(s.t3 * 57.2957795f),
             (double)s.roll,
             (int)(s.grip * 255.0f + 0.5f));
}

// Mapeo desde el DataPacket del control remoto (-200..200 por eje).
// ⚠️ TODO: confirmar en el brazo real que estos ejes de joystick son los
// mas comodos; es la primera cosa a ajustar despues de probarlo.
inline void ctrl_from_packet(int16_t upperJoyX, int16_t upperJoyY,
                             int16_t lowerJoyX, int16_t lowerJoyY,
                             uint8_t pot,
                             float &a, float &b, float &c,
                             float &roll, float &grip)
{
    a    =  (float)upperJoyY / 200.0f;   // avance / retroceso
    b    =  (float)upperJoyX / 200.0f;   // barrido lateral
    c    =  (float)lowerJoyY / 200.0f;   // barrido arriba / abajo
    roll =  (float)lowerJoyX / 200.0f;   // giro de la pinza (J4)
    grip =  (float)pot / 255.0f;
    a = ctrl_clampf(a, -1.0f, 1.0f);  b = ctrl_clampf(b, -1.0f, 1.0f);
    c = ctrl_clampf(c, -1.0f, 1.0f);  roll = ctrl_clampf(roll, -1.0f, 1.0f);
}

#endif // CTRL_MTDS_H
