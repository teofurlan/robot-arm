// Stub minimo de la API de Arduino, solo para compilar y probar la logica del
// firmware en la PC. NO se compila en el dispositivo (vive bajo test/).
#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <deque>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define SERIAL_8N1 0
#define F(x) (x)
#define PROGMEM

// ---- reloj simulado ----
extern unsigned long g_micros;
inline unsigned long micros(){ return g_micros; }
inline unsigned long millis(){ return g_micros / 1000UL; }
inline void delay(unsigned long ms){ g_micros += ms * 1000UL; }
inline void delayMicroseconds(unsigned long us){ g_micros += us; }
inline void advance_us(unsigned long us){ g_micros += us; }
inline void advance_ms(unsigned long ms){ g_micros += ms * 1000UL; }

// ---- pines simulados ----
extern int g_pin[100];
extern int g_pinIn[100];
extern long g_stepCount[100];
inline void pinMode(int, int){}
inline void digitalWrite(int p, int v){
  if(p>=0 && p<100){ if(v==HIGH && g_pin[p]==LOW) g_stepCount[p]++; g_pin[p]=v; }
}
inline int digitalRead(int p){ return (p>=0&&p<100)? g_pinIn[p] : HIGH; }

template<class T> T constrain(T v, T lo, T hi){ return v<lo?lo:(v>hi?hi:v); }
inline long map(long x,long a,long b,long c,long d){ return (x-a)*(d-c)/(b-a)+c; }

// ---- String minimo ----
class String {
public:
  std::string s;
  String(){} String(const char*p):s(p?p:""){} String(const std::string&x):s(x){}
  int indexOf(char c) const { size_t i=s.find(c); return i==std::string::npos?-1:(int)i; }
  String substring(int a,int b) const { if(a<0)a=0; if(b>(int)s.size())b=s.size();
    return String(s.substr(a, (b>a)?(b-a):0)); }
  const char* c_str() const { return s.c_str(); }
  bool operator==(const char*p) const { return s==p; }
};

// ---- puertos serie simulados ----
class SerialStub {
public:
  std::deque<char> in;      // lo que el firmware va a leer
  std::string out;          // lo que el firmware escribio
  void begin(long){} void begin(long,int,int,int){}
  int available(){ return (int)in.size(); }
  int read(){ if(in.empty()) return -1; char c=in.front(); in.pop_front(); return c; }
  String readStringUntil(char t){ std::string r;
    while(!in.empty()){ char c=in.front(); in.pop_front(); if(c==t) break; r+=c; }
    return String(r); }
  void print(const char*p){ out+=p; } void print(const String&p){ out+=p.s; }
  void println(const char*p){ out+=p; out+="\n"; }
  void println(){ out+="\n"; }
  template<class...A> void printf(const char*f,A...a){ char b[512]; snprintf(b,sizeof b,f,a...); out+=b; }
  void feed(const std::string&t){ for(char c:t) in.push_back(c); }
  void clear(){ out.clear(); }
};
extern SerialStub Serial;
extern SerialStub Serial1;

// ---- Servo ----
class Servo { public: int last=-1; void attach(int){} void write(int v){ last=v; } };

#endif
