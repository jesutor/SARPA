#include "soc/rtc.h"
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <SPI.h>
#include <Arduino.h>
#include <math.h>
#include <HX711.h>
/*****If you wanna to work with firebase uncomment this*****/
//#include <Blynk.h>
//#include <IOXhop_FirebaseESP32.h>
//#include <credentials.h>

//#define FIREBASE_HOST "acamados1.firebaseio.com"
//#define FIREBASE_AUTH "zzNFaVCQktpKMGeXF2UkuJNOgWjjro4HMGF8DnfP"


/****Personal functions****/
#include "funciones.h"
#include "variables.h"
#include "blynkv.h"

/****Pins that define the SPI communication in ESP-32****/
const byte hx711_data_pin = 25;
const byte hx711_clock_pin = 26;

/****Configuration of HX711****/
HX711 balanza(hx711_data_pin, hx711_clock_pin);

/****Section to verify the connectivity between the Esp-32 and the server****/
BlynkTimer timer;

void CheckConnection() {   // check every 11s if connected to Blynk server
  if (!Blynk.connected()) {
    //Serial.println("Not connected to Blynk server");
    Blynk.connect(5);  // try to connect to server with default timeout
  }
  else {
    //Serial.println("Connected to Blynk server");
  }
}
/****Main******/
void setup()
{
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);/*Real Time Clock, it is necessary for HX711*/

  /****Setup SPI****/
  pinMode(SpiCsPin, OUTPUT);/**Pin for transferens data from hx711 to esp32**/
  digitalWrite(SpiCsPin, HIGH);
  pinMode(15, INPUT);/**Pin for transferens data from Nucleo to esp32**/

  SPI.begin();
  SPI.beginTransaction(SPISettings(SpiSpeed, MSBFIRST, SPI_MODE0));/**SPI configuration ESP32-Nucleo --don't move**/
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass, IPAddress(192, 168, 137, 1), 8080); /*Connect to the server*/
  //Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Serial.print("ADC value:  ");
  Serial.println(balanza.read());
  Serial.println("Do not put any object on the scale");
  Serial.println("Taring...");
  balanza.set_scale(11746); //Default scale is 11746
  balanza.tare();  //Actual weight is considered tare.
  Serial.println("Ready to weight");
  // Setup a function to be called every second
  timerId = timer.setInterval(1500L, CheckConnection);
  /*timer.disable(timerId);
  Firebase.setFloat("Weight", 0.0);
  // handle error
  if (Firebase.failed()) {
    Serial.print("setting /number failed:");
    Serial.println(Firebase.error());
    return;
    }*/
}
/****Loop******/
void loop()
{
  if (play1 == 1) {/**Passive function**/

    digitalWrite(SpiCsPin, LOW);
    envia_por_tipo(1, menu_1, pos, v_1, f_1, c_1, pos_2);
    digitalWrite(SpiCsPin, HIGH);
    contador_2++;
  }
  else if (play_isometrica == 1) {/**Isometric function**/
    digitalWrite(SpiCsPin, LOW);
    envia_por_tipo(2, 0, 0, 0, 0, 0, 0);
    digitalWrite(SpiCsPin, HIGH);
  }
  else if (play_isotonica == 1) {/**Isotonic function**/
    digitalWrite(SpiCsPin, LOW);
    envia_por_tipo(3, S, 0, forca_isotonica, 0, 0, 0);
    digitalWrite(SpiCsPin, HIGH);
  }
  else if (play_isocinetica == 1) {/**Spring function**/
    digitalWrite(SpiCsPin, LOW);
    envia_por_tipo(4, 0, k, 0, C_m, 0, faixa_manual);
    digitalWrite(SpiCsPin, HIGH);
  }
  else if (play_isocinetica_d == 1) {/**Isokinetic function**/
    digitalWrite(SpiCsPin, LOW);
    envia_por_tipo(5, velocidade_isocinetica, 0, b, C_m, 0, 0);
    digitalWrite(SpiCsPin, HIGH);
  }
  /**Send and grhaphic the read from the loadcell**/
  celula_c =  balanza.read_average(1);
  if ((celula_c<12000) && (celula_c>-12000)){/**Reference never is zero completely**/
    celula_c = 0;
  }
  Serial.println((double)celula_c/11746);
  
  //Firebase.pushFloat("Weight", celula_c);
  if (digitalRead(15) == HIGH) {
    digitalWrite(SpiCsPin, LOW);
    SPI.transfer16(stop_isocinetica);/**This command is sent to stop any task**/
    delayMicroseconds(1);
    SPI.transfer16((celula_c >> 16) & 0xFFFF);
    delayMicroseconds(1);
    SPI.transfer16(celula_c & 0xFFFF);
    delayMicroseconds(1);
    digitalWrite(SpiCsPin, HIGH);
    /*// handle error
      if (Firebase.failed()) {
      Serial.print("setting /number failed:");
      Serial.println(Firebase.error());
      return;
      }*/
  }

  Blynk.run();
  timer.run();


}
