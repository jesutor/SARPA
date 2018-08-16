
/*********Variables and constants*********/
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "8ff630d7f30b4a9d8d2a1fd4a214ab9f";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "project";
char pass[] = "blynk123";

const int SpiCsPin = 5;/**SS SPI pin-don't move**/
const int SpiSpeed = 10000000;/**Clock SPI speeed-don't move**/

int timerId;

/*********Variables e constantes*********/
/*******Menus*****/
int menu_1;
int menu_2;
/*******SubMenus*****/
int submenu_1;
/*******Variables*****/
int v_1=60,f_1=70,c_1=0,v_2=0,f_2=0,c_2=0,i=0;
int pos,pos_2,datos[4];   // for incoming serial data
int AngQuadril =0,S,T,l_1,forca_isometrica,forca_isotonica,velocidade_isocinetica;
int k,b,C_m;
uint16_t Ti = 0x00FF;
int contador = 0,contador_2 = 0,faixa_manual=50;
long celula_c;
/*******Ejecucion*****/
boolean play1;
boolean play_isometrica;
boolean play_isotonica;
boolean play_isocinetica;
boolean stop_isocinetica;
boolean play_isocinetica_d;
//boolean stop_isocinetica_d;

