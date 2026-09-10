// Prueba de la logica del firmware del receiver (ESP32) en la PC, con el stub
// de Arduino/ESP-NOW. Verifica el ritmo del lazo, la trama que sale hacia el
// Mega, el watchdog y el pedido de home.
#include "stub/Arduino.h"
#include "stub/WiFi.h"
#include "stub/esp_now.h"
#include "../../src/receiver/receiver.cpp"

static int fails = 0;
#define CHECK(c, msg) do{ if(!(c)){ printf("  FALLA: %s\n", msg); ++fails; } }while(0)

static void rx(int16_t ux,int16_t uy,int16_t lx,int16_t ly,uint8_t pot,uint8_t rst){
  DataPacket p; p.upperJoyX=ux; p.upperJoyY=uy; p.lowerJoyX=lx; p.lowerJoyY=ly;
  p.pot=pot; p.reset=rst;
  g_recvCb(nullptr, (const uint8_t*)&p, (int)sizeof(p));
}
// corre n ciclos de 20 ms, reinyectando el paquete en cada uno (como el control real a 50 Hz)
static void run(int n, int16_t ux,int16_t uy,int16_t lx,int16_t ly,uint8_t pot,uint8_t rst){
  for(int i=0;i<n;i++){ rx(ux,uy,lx,ly,pot,rst); advance_ms(20); loop(); }
}
static int countFrames(const std::string& s){
  int n=0; for(size_t i=0;i<s.size();++i) if(s[i]=='<') ++n; return n;
}

int main(){
  setup();
  printf("=== 1. Arranque ===\n");
  CHECK(g_recvCb != nullptr, "callback de ESP-NOW registrado");
  printf("  %s", Serial.out.c_str());

  printf("=== 2. Ritmo del lazo: 50 Hz ===\n");
  MegaSerial.clear();
  run(50, 0,200,0,0, 0,0);                       // 50 ciclos = 1 s simulado
  int nf = countFrames(MegaSerial.out);
  CHECK(nf >= 45 && nf <= 55, "una trama por ciclo de 20 ms");
  printf("  %d tramas en 1 s simulado\n", nf);

  printf("=== 3. La trama que sale es la esperada ===\n");
  std::string o = MegaSerial.out;
  size_t a = o.rfind('<'), b = o.find('>', a);
  std::string last = o.substr(a, b-a+1);
  float t1,t2,t3,rl; int gp;
  int got = sscanf(last.c_str(), "<%f,%f,%f,%f,%d>", &t1,&t2,&t3,&rl,&gp);
  CHECK(got == 5, "la trama tiene 5 campos parseables");
  CHECK(fabsf(t1 - ctrl.t1*57.2957795f) < 0.02f, "T1 de la trama coincide con el estado");
  CHECK(fabsf(t2 - ctrl.t2*57.2957795f) < 0.02f, "T2 de la trama coincide con el estado");
  CHECK(fabsf(t3 - ctrl.t3*57.2957795f) < 0.02f, "T3 de la trama coincide con el estado");
  printf("  %s -> T1=%.2f T2=%.2f T3=%.2f Roll=%.1f Pinza=%d\n", last.c_str(), t1,t2,t3,rl,gp);

  printf("=== 4. El brazo efectivamente se movio y respeta los topes ===\n");
  CHECK(fabsf(ctrl.t1) + fabsf(ctrl.t2) + fabsf(ctrl.t3) > 0.01f, "hubo movimiento");
  CHECK(ctrl.t1 >= T_MIN[0]-1e-5f && ctrl.t1 <= T_MAX[0]+1e-5f, "t1 dentro de los topes");
  CHECK(ctrl.t2 >= T_MIN[1]-1e-5f && ctrl.t2 <= T_MAX[1]+1e-5f, "t2 dentro de los topes");
  CHECK(ctrl.t3 >= T_MIN[2]-1e-5f && ctrl.t3 <= T_MAX[2]+1e-5f, "t3 dentro de los topes");
  printf("  T = %.2f / %.2f / %.2f deg\n",
         ctrl.t1*57.2957795f, ctrl.t2*57.2957795f, ctrl.t3*57.2957795f);

  printf("=== 5. Recorrido largo: nunca se sale de los topes ===\n");
  bool viol=false;
  for(int i=0;i<3000;i++){
    int16_t ux=(int16_t)(200*sinf(i*0.031f)), uy=(int16_t)(200*sinf(i*0.017f));
    int16_t lx=(int16_t)(200*sinf(i*0.043f)), ly=(int16_t)(200*sinf(i*0.011f));
    rx(ux,uy,lx,ly,(uint8_t)(i%256),0); advance_ms(20); loop();
    if(ctrl.t1<T_MIN[0]-1e-4f||ctrl.t1>T_MAX[0]+1e-4f||
       ctrl.t2<T_MIN[1]-1e-4f||ctrl.t2>T_MAX[1]+1e-4f||
       ctrl.t3<T_MIN[2]-1e-4f||ctrl.t3>T_MAX[2]+1e-4f) { viol=true; break; }
    if(fabsf(ctrl.roll)>CTRL_ROLL_LIMIT+1e-4f){ viol=true; break; }
  }
  CHECK(!viol, "3000 ciclos con joystick aleatorio, sin violar topes");
  printf("  3000 ciclos (60 s simulados) sin violaciones\n");

  printf("=== 6. WATCHDOG: sin enlace el brazo se queda donde esta ===\n");
  float k1=ctrl.t1,k2=ctrl.t2,k3=ctrl.t3;
  advance_ms(600); loop();                                    // primer ciclo con el enlace ya vencido
  CHECK(fabsf(ctrl.t1-k1)<1e-4f && fabsf(ctrl.t2-k2)<1e-4f && fabsf(ctrl.t3-k3)<1e-4f,
        "se detiene en UN ciclo, sin coasting del filtro");
  for(int i=0;i<100;i++){ advance_ms(20); loop(); }          // 2 s mas sin paquetes
  CHECK(fabsf(ctrl.t1-k1)<1e-4f && fabsf(ctrl.t2-k2)<1e-4f && fabsf(ctrl.t3-k3)<1e-4f,
        "los angulos no cambiaron con el enlace caido");
  // y la trama sigue saliendo, repitiendo la MISMA pose (mantiene, no vuelve al origen)
  MegaSerial.clear(); advance_ms(20); loop();
  CHECK(MegaSerial.out.find('<') != std::string::npos, "sigue mandando la pose actual");
  printf("  angulos congelados en %.2f / %.2f / %.2f y la trama se repite\n",
         ctrl.t1*57.2957795f, ctrl.t2*57.2957795f, ctrl.t3*57.2957795f);

  printf("=== 7. RESET -> pide home una sola vez y pone el estado en cero ===\n");
  MegaSerial.clear();
  run(1, 0,0,0,0, 0,1);
  CHECK(MegaSerial.out.find("<H>") != std::string::npos, "mando <H>");
  CHECK(fabsf(ctrl.t1)<1e-6f && fabsf(ctrl.t2)<1e-6f && fabsf(ctrl.t3)<1e-6f,
        "el estado quedo en cero");
  MegaSerial.clear();
  run(5, 0,0,0,0, 0,1);                                       // sigue apretado
  CHECK(MegaSerial.out.find("<H>") == std::string::npos, "no repite <H> mientras sigue apretado");
  printf("  home pedido una sola vez por flanco\n");

  printf("\n%s (%d fallas)\n", fails? "HAY FALLAS":"TODO OK", fails);
  return fails?1:0;
}
