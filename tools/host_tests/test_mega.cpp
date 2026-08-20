// Prueba de la logica del firmware del Mega, corriendo en la PC con el stub
// de Arduino. Verifica el parseo de la trama, la conversion a pasos, los
// topes, el comportamiento del watchdog y el movimiento hacia el objetivo.
#include "stub/Arduino.h"
#include "../../src/mega/mega.cpp"

static int fails = 0;
#define CHECK(c, msg) do{ if(!(c)){ printf("  FALLA: %s\n", msg); ++fails; } }while(0)
#define CHECKEQ(a, b, msg) do{ long _x=(long)(a), _y=(long)(b); \
  if(_x!=_y){ printf("  FALLA: %s (fue %ld, esperado %ld)\n", msg, _x, _y); ++fails; } }while(0)

static void send(const char* frame){ Serial1.feed(std::string(frame)+"\n"); loop(); }

int main(){
  setup();
  printf("=== 1. Trama valida -> angulos convertidos a pasos ===\n");
  send("<5.00,-20.00,40.00,30.0,128>");
  CHECKEQ(tgtX, lround(5.0*17.77),   "tgtX");
  CHECKEQ(tgtY, lround(-20.0*17.77), "tgtY");
  CHECKEQ(tgtZ, lround(40.0*17.77),  "tgtZ");
  CHECKEQ(wrist.last, 120,           "servo de roll (90+30)");
  CHECKEQ(clamp.last, map(128,0,255,0,180), "servo de pinza");
  printf("  tgt = %ld / %ld / %ld pasos | roll servo %d | pinza servo %d\n",
         tgtX,tgtY,tgtZ,wrist.last,clamp.last);

  printf("=== 2. Los topes se aplican en el Mega tambien ===\n");
  send("<90.00,90.00,9000.00,0.0,0>");
  CHECKEQ(tgtX, LIM_X,     "tgtX recortado a LIM_X");
  CHECKEQ(tgtY, LIM_Y_DER, "tgtY recortado a LIM_Y_DER");
  CHECKEQ(tgtZ, LIM_Z_MAX, "tgtZ recortado a LIM_Z_MAX");
  send("<-90.00,-90.00,-9000.00,0.0,0>");
  CHECKEQ(tgtX, -LIM_X,    "tgtX recortado a -LIM_X");
  CHECKEQ(tgtY, LIM_Y_IZQ, "tgtY recortado a LIM_Y_IZQ");
  CHECKEQ(tgtZ, LIM_Z_MIN, "tgtZ recortado a LIM_Z_MIN");
  printf("  recorte OK en los seis extremos\n");

  printf("=== 3. El roll se recorta al medio recorrido del servo ===\n");
  send("<0.00,0.00,0.00,300.0,0>");
  CHECKEQ(wrist.last, 180, "roll saturado arriba");
  send("<0.00,0.00,0.00,-300.0,0>");
  CHECKEQ(wrist.last, 0,   "roll saturado abajo");

  printf("=== 4. Tramas corruptas se descartan sin tocar el objetivo ===\n");
  send("<10.00,0.00,0.00,0.0,0>");
  long kx=tgtX, ky=tgtY, kz=tgtZ;
  const char* malas[] = {"<1,2>", "<a,b,c,d,e>", "<1.0 2.0 3.0 4.0 5.0>", "<>", "<1.0,2.0,,4.0,5.0>"};
  for (unsigned i=0;i<sizeof(malas)/sizeof(*malas);++i) send(malas[i]);
  CHECK(tgtX==kx && tgtY==ky && tgtZ==kz, "el objetivo no cambio con tramas corruptas");
  printf("  5 tramas corruptas ignoradas, objetivo intacto\n");

  printf("=== 5. Movimiento: llega al objetivo y no se pasa ===\n");
  send("<3.00,0.00,0.00,0.0,0>");
  long want = tgtX;
  for (int i=0;i<20000 && posX!=want;++i){ advance_us(MIN_STEP_INTERVAL+1); loop(); }
  CHECKEQ(posX, want, "posX llego al objetivo");
  long before = g_stepCount[X_STEP];
  for (int i=0;i<500;++i){ advance_us(MIN_STEP_INTERVAL+1); loop(); }
  CHECKEQ(g_stepCount[X_STEP]-before, 0, "no sigue pulsando al llegar");
  printf("  posX = %ld pasos, sin pulsos de mas\n", posX);

  printf("=== 6. WATCHDOG: se queda quieto, NO vuelve al origen ===\n");
  send("<8.00,-30.00,50.00,0.0,0>");
  for (int i=0;i<40000;++i){ advance_us(MIN_STEP_INTERVAL+1); loop(); }
  long hx=posX, hy=posY, hz=posZ;
  CHECK(hx!=0 || hy!=0 || hz!=0, "el brazo se movio fuera del origen");
  advance_ms(600); loop();                       // se corta el enlace
  CHECKEQ(tgtX, hx, "tgtX = posicion actual");
  CHECKEQ(tgtY, hy, "tgtY = posicion actual");
  CHECKEQ(tgtZ, hz, "tgtZ = posicion actual");
  long sx=g_stepCount[X_STEP];
  for (int i=0;i<5000;++i){ advance_us(MIN_STEP_INTERVAL+1); loop(); }
  CHECKEQ(g_stepCount[X_STEP]-sx, 0, "no dio ni un paso con el enlace caido");
  CHECK(posX==hx && posY==hy && posZ==hz, "quedo exactamente donde estaba");
  printf("  quedo en %ld / %ld / %ld pasos y no se movio mas\n", posX,posY,posZ);

  printf("=== 7. HOME: deja las tres juntas en angulo 0 ===\n");
  g_pinIn[X_MIN_PIN] = LOW; g_pinIn[Y_MIN_PIN] = LOW;   // sensores ya tocados
  Serial1.feed("<H>\n"); loop();
  CHECKEQ(posX,0,"posX en 0"); CHECKEQ(posY,0,"posY en 0"); CHECKEQ(posZ,0,"posZ en 0");
  CHECKEQ(tgtX,0,"tgtX en 0"); CHECKEQ(tgtY,0,"tgtY en 0"); CHECKEQ(tgtZ,0,"tgtZ en 0");
  CHECKEQ(wrist.last,90,"roll centrado");
  CHECK(!haciendoHome, "salio del estado de home");
  printf("  home terminado, todo en cero\n");

  printf("\n%s (%d fallas)\n", fails? "HAY FALLAS":"TODO OK", fails);
  return fails?1:0;
}
