/****Blynk*******/

/****Every command here receive or send values through the interface into the ESP32*******/

/****Passive*******/
  BLYNK_WRITE(V0)
{
  menu_1 = param.asInt();
}
  BLYNK_WRITE(V1)
{
  pos = param.asInt();
}
  BLYNK_WRITE(V2)
{
  v_1 = param.asInt();
}  
  BLYNK_WRITE(V3)
{
  f_1 = param.asInt();
}  
  BLYNK_WRITE(V4)
{
  play1 = param.asInt();
}
  BLYNK_WRITE(V6)
{
  c_1 = param.asInt();
}
  BLYNK_WRITE(V5)
{
  pos_2 = param.asInt();
}
/****Isometric*****/
BLYNK_WRITE(V10)
{
  forca_isometrica = param.asInt();
}
BLYNK_WRITE(V12)
{
  play_isometrica = param.asInt();
}
/****Isotonic****/
BLYNK_WRITE(V20)
{
  S = param.asInt();
}
BLYNK_WRITE(V21)
{
  T = param.asInt();
}
BLYNK_WRITE(V22)
{
  forca_isotonica = param.asInt();
}
BLYNK_WRITE(V23)
{
  play_isotonica = param.asInt();
}

/****Isokinetic and spring****/
BLYNK_WRITE(V30)
{
  velocidade_isocinetica = param.asInt();
}
BLYNK_WRITE(V31)
{
  k = param.asInt();
}
BLYNK_WRITE(V32)
{
  b = param.asInt();
}
BLYNK_WRITE(V33)
{
  play_isocinetica = param.asInt();
}
BLYNK_WRITE(V34)
{
  C_m = param.asInt();
}
BLYNK_WRITE(V35)
{
  stop_isocinetica = param.asInt();
}
BLYNK_WRITE(V36)
{
  play_isocinetica_d = param.asInt();
}
BLYNK_WRITE(V37)
{
  faixa_manual = param.asInt();
}

/****Shown the weight in the HMI*******/
BLYNK_READ(V50) // Widget in the app READs Virtal Pin V50 with the certain frequency
{
  // This command writes Arduino's uptime in seconds to Virtual Pin V50
  Blynk.virtualWrite(50, (double)celula_c/11746);
}

