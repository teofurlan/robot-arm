# Control de la v1 — por herramienta, sin RCM

## Qué cambió

Antes: **un eje de joystick = un motor**. El operador tenía que pensar en motores.

Ahora: **los joysticks mueven la punta de la herramienta**, y el software reparte
ese movimiento entre J1, J2 y J3 usando el jacobiano del brazo. La ESP32 hace el
cálculo y le manda al Mega **ángulos absolutos de junta**.

| | Antes | Ahora |
|---|---|---|
| Trama ESP32 → Mega | `<vx1,vy1,vx2,vy2,pot,reset>` (velocidades) | `<T1,T2,T3,Roll,Pinza>` (ángulos en grados) y `<H>` (home) |
| Quién decide el movimiento | el Mega, eje por eje | la ESP32, en el espacio de la herramienta |
| Topes de recorrido | solo en el Mega, en pasos | en la ESP32 (en ángulos) **y** en el Mega (en pasos, como red de seguridad) |
| Watchdog | velocidades a cero → se detiene | objetivo = posición actual → se queda quieto |

## ⚠️ Este firmware no implementa RCM, y es a propósito

Con 3 juntas moviendo la recta del vástago y 2 restricciones que impone un punto
de pivote fijo, quedan **1 grado de libertad controlable** y hacen falta **3**
(cabeceo, guiñada, profundidad de inserción). Es imposible por conteo de grados
de libertad, no por falta de código: ningún solver lo resuelve.

Verificado por cinco vías independientes: el rango del jacobiano de la
restricción es exactamente 2 en 3.000 combinaciones (pose, punto), nunca 1; el
conjunto solución es una curva de dimensión 1; de 225 direcciones de herramienta
pedidas, sólo 1 admite además respetar el RCM; y buscando por cálculo cerrado
**el mejor punto de pivote posible** para este brazo, el desvío mediano sigue
siendo de 101 mm — unas diez veces el diámetro de un trocar laparoscópico.

**Consecuencia práctica:** el punto de entrada de la herramienta se va a mover
mientras se maneja. Eso es esperable y correcto para este hardware. La v1 es un
**brazo teleoperado de 3 ejes**, no un simulador laparoscópico.

Para tener RCM de verdad hace falta un cambio estructural, no de software: llevar
el cruce de los ejes del cardán a un punto en el aire por donde pase el vástago,
agregar un eje lineal de inserción y sacar el codo. Eso da los 4 grados de
libertad laparoscópicos completos con la misma cantidad de motores, y sin
cinemática inversa (el RCM lo garantiza la estructura). Es trabajo de v2.

## Los dos modos de control

Se elige con la variable `mode` en `receiver.cpp`.

- **`CTRL_TOOL`** (por defecto, recomendado) — la velocidad pedida se interpreta
  en el marco **de la herramienta**: avanzar/retroceder a lo largo del vástago,
  barrer la punta a un costado, barrer arriba/abajo. Con la base quieta, barrer
  *es* apuntar. Se siente como tener el instrumento en la mano.
- **`CTRL_WORLD`** — la velocidad pedida se interpreta en el marco del robot:
  +X adelante, +Y lateral, +Z arriba. Más predecible; sirve para calibrar y para
  verificar el sentido de giro de cada motor.

Mapeo de los joysticks (en `ctrl_from_packet`, fácil de cambiar):

| Control | `CTRL_TOOL` | `CTRL_WORLD` |
|---|---|---|
| Joystick sup. vertical | entrar / salir | +X |
| Joystick sup. horizontal | barrer al costado | +Y |
| Joystick inf. vertical | barrer arriba / abajo | +Z |
| Joystick inf. horizontal | roll de la pinza (J4) | roll de la pinza (J4) |
| Potenciómetro | apertura de pinza | apertura de pinza |
| Ambos botones | home | home |

## Cómo funciona

`ctrl_mtds.h`, una llamada por ciclo de 20 ms:

1. Zona muerta + curva expo sobre el joystick, y filtro de suavizado.
2. La velocidad pedida de la punta se pasa al marco correspondiente.
3. Jacobiano analítico 3×3 de la punta, y **mínimos cuadrados amortiguados**
   con amortiguación adaptativa según `|det J|`: cerca de una singularidad el
   brazo se pone lento en vez de dar un latigazo.
4. Tope de velocidad por junta, escalando el paso **completo** para no torcer la
   dirección del movimiento pedido.
5. Topes de recorrido, escalando el paso para quedarse adentro.

Costo: **0,11 µs por ciclo en x86**. Incluso 50× más lento en la ESP32 son ~5 µs,
o sea **0,03 %** del presupuesto de 20 ms a 50 Hz.

## Geometría

`geom_mtds.h` es la **única fuente de verdad**. Si cambia el modelo 3D, se toca
solo ese archivo. Cada número tiene su procedencia documentada ahí.

| Parámetro | Valor |
|---|---|
| `O` — centro del cardán (marco CAD) | (32,0 · 55,0 · −47,0) mm |
| `A_Y` — brazo superior | 343,400 mm |
| `B` — antebrazo, al montaje de la herramienta | (357,6524 · 51,6101 · −2,2033) mm, \|B\| = 361,364 mm |
| `U0` — eje del vástago (= eje de J4) | (0,9934421 · 0,1140048 · −0,0087004) |
| `L_TOOL` | 300,0 mm |
| Punta en reposo (marco robot) | X = 655,7 · Y = 4,8 · Z = 429,2 mm |

Medido el 20/08/2026 sobre `Ensamble Brazo Robot v1` en Fusion, con el modelo en
una pose cualquiera y retro-transformado a la pose de referencia. El
procedimiento se validó antes de usarlo: la FK de estos headers, alimentada con
los valores de articulación que reporta Fusion, reproduce la posición medida del
eje del codo con **0,003 mm** de error.

## ⚠️ Dos TODO de calibración antes de subirlo al brazo

1. **Los topes por junta son provisorios.** Hoy son los soft-limits del firmware
   anterior convertidos con 17,77 pasos/grado: J1 ±10,02°, J2 −59,99/+10,02°,
   J3 ±281,4°. Faltan los grados reales de cada eje del cardán.
2. **El mapeo motor → junta está asumido** como X→J1, Y→J2, Z→J3. Hay que
   confirmarlo en el brazo: si está cruzado, los topes protegen la junta
   equivocada, que es peor que no tenerlos.

Y una convención a revisar: **al terminar el home, las tres juntas quedan
definidas como ángulo 0.** Los offsets del home (178 pasos en X, 200 en Y) vienen
del firmware anterior; hay que ajustarlos para que ese cero coincida con el cero
del CAD. El eje Z (codo) no tiene fin de carrera: toma su posición actual como
cero.

## Pruebas

```
make -C tools/host_tests
```

Compila el firmware con un stub de la API de Arduino/ESP-NOW y corre cuatro
suites en la PC, sin hardware:

| Suite | Qué verifica |
|---|---|
| `ctrl_mtds.h` | FK, largos de eslabón constantes, jacobiano analítico contra numérico, seguimiento de la dirección pedida, topes en 200.000 ciclos aleatorios, tope de velocidad, costo de cómputo |
| `ik_mtds.h` | 32.000 poses de ida y vuelta FK→IK→FK, rechazo de objetivos inalcanzables y degenerados |
| firmware del Mega | parseo de la trama, conversión a pasos, recorte en los seis extremos, descarte de tramas corruptas, llegada al objetivo sin pasarse, watchdog que mantiene posición, home |
| firmware del receiver | ritmo de 50 Hz, trama emitida, topes en 3.000 ciclos con joystick aleatorio, watchdog que detiene en un ciclo, home pedido una sola vez por flanco |

El test del receiver encontró un problema real de seguridad durante el
desarrollo: al caerse el enlace, mandar comando cero no alcanzaba, porque el
filtro de suavizado tardaba ~0,6 s en decaer y el brazo seguía moviéndose todo
ese tiempo. Se agregó `ctrl_stop()`, que lo corta en un ciclo.
