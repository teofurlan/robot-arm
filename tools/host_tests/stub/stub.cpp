#include "Arduino.h"
unsigned long g_micros = 1000000UL;
int g_pin[100] = {0};
int g_pinIn[100];
long g_stepCount[100] = {0};
SerialStub Serial;
SerialStub Serial1;
struct InitPins { InitPins(){ for(int i=0;i<100;i++) g_pinIn[i]=HIGH; } } _ip;
