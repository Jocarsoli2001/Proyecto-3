//***************************************************************************************************************************************
/* Librería para el uso de la pantalla ILI9341 en modo 8 bits
   Basado en el código de martinayotte - https://www.stm32duino.com/viewtopic.php?t=637
   Adaptación, migración y creación de nuevas funciones: Pablo Mazariegos y José Morales
   Con ayuda de: José Guerra
   IE3027: Electrónica Digital 2 - 2019
*/
//***************************************************************************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <TM4C123GH6PM.h>
#include <SPI.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "bitmaps.h"
#include "font.h"
#include "lcd_registers.h"
#include <SD.h>

//***************************************************************************************************************************************
// Definiciones de pines
//***************************************************************************************************************************************

#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1
#define Jugador_1 PE_4
#define Izquierda_J1 PA_7
#define Derecha_J1 PC_7
#define Gravedad 0.001

//***************************************************************************************************************************************
// Variables
//***************************************************************************************************************************************
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
int Estado_J1 = 0;
int presionado_J1 = 0;
int IZ_J1 = 0;
int DE_J1 = 0;
int flip = 0;
int volando = 0;
int Game_Over = 0;
int overlap_J1 = 0;
int region_obs1 = 0;
int vector_x1 = 0;
int vector_y1 = 0;

  //***********************************************************************************************************************************
  // Variables comunes
  //***********************************************************************************************************************************
  float t = 1.5;
  int cont_anim1 = 0;
  int flap = 0;
  int pintar_iz = 0;
  int pintar_der = 0;
  int pintar_arriba = 0;
  int pintar_abajo = 0;
  int parado = 0;
  int refresh = 0;
  
  //***********************************************************************************************************************************
  // Variables jugador (eje y)
  //***********************************************************************************************************************************
  float y = 100;
  float yf = 0;
  float Vy = 0;
  float y_prev = 0;
  float Ay = Gravedad;
  
  //***********************************************************************************************************************************
  // Variables jugador (eje x)
  //***********************************************************************************************************************************
  float x = 50;
  float xf = 0;
  float Vx = 0;
  float x_prev = 0;
  float Ax = 0;


//***************************************************************************************************************************************
// Functions Prototypes
//***************************************************************************************************************************************
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
void LCD_Clear(unsigned int c);
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c);
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c);
void LCD_Print(String text, int x, int y, int fontSize, int color, int background);
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]);
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset);
int Check_overlap(int posx_poligono, int posy_poligono, int posx_jugador, int posy_jugador, int ancho_poligono, int alto_poligono, int alto_jugador, int ancho_jugador);
int Regiones(int posx_J1, int posy_J1, int alto_J1, int ancho_J1, int posx_obstaculo, int posy_obstaculo, int ancho_obstaculo, int alto_obstaculo);

//***************************************************************************************************************************************
// Inicialización
//***************************************************************************************************************************************
void setup() {
  
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(115200);
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Inicio");
  SPI.setModule(0);
  LCD_Init();
  LCD_Clear(0x00);

  pinMode(Jugador_1, INPUT);
  pinMode(Izquierda_J1, INPUT);
  pinMode(Derecha_J1, INPUT);                             


  FillRect(0, 0, 319, 239, 0x0000);
  FillRect(106 + 4, 119 + 4, 107 - 6, 40 - 6, 0x2703);


}
//***************************************************************************************************************************************
// Loop Infinito
//***************************************************************************************************************************************
void loop() {
  
  while(Game_Over == 0) {
    //***********************************************************************************************************************************
    // Implementación de física (eje x)
    //***********************************************************************************************************************************
    IZ_J1 = digitalRead(Izquierda_J1);
    DE_J1 = digitalRead(Derecha_J1);
    
    if(IZ_J1 == HIGH)                                             // En el eje x, cuando se presiona el botón para mover a la izquierda, se agrega una velocidad en el eje x-, por lo va a la izquierda
    {
      Vx -= 0.003;                                                 
      Ax = -0.0007;                                                // Se genera una aceleración para un giro más suave
      flip = 0;                                                   // Se utiliza la variable flip para que el sprite haga un flip dependiendo de la dirección a la que vaya
    }
    if(DE_J1 == HIGH){                                            // En el eje x, cuando se presiona el botón para mover a la derecha, se agrega una velocidad en el eje x+, por lo va a la derecha
      Vx += 0.003;
      Ax = 0.0007;                                                 // Se genera una aceleración para un giro más suave
      flip = 1;                                                   // Se utiliza la variable flip para que el sprite haga un flip dependiendo de la dirección a la que vaya
    }

    if(Vx<=0.2 && Vx>=-0.2){                                      // Para evitar que el jugador comience a acelerar, se detiene la velocidad cuando esta llega a 1 o a -1 (movimiento hacia arriba)
      Vx += Ax*(t);
    }
    else if(Vx>0.2){
      Vx = 0.2;                                                   // Si la velocidad se vuelve mayor a 1, la velocidad se detiene en 1 para evitar la constante aceleración hacia abajo.
    }
    else if(Vx<-0.2){
      Vx = -0.2;                                                  // Si la velocidad se vuelve menor a -1, la velocidad se detiene en -1 para evitar la constante aceleración hacia arriba.
    }

    /* 
     * Para lograr que el sprite aparezca hacia arriba cuando este atraviesa la pared de abajo, se hizo que se fuera a la coordenada "x = 0" cuando la misma se volviera mayor a 320 (límite de pantalla).
     * Lo mismo sucede si supera la coordenada "x = 320", el sprite se mueve a la coordenada "x = 0".
     */
    if(x > 320){
      x = 0;
    }
    else if(x < 0){
      x = 320;
      FillRect(0, 0, 19, 239, 0x0000);                            // Si el jugador sobrepasa la coordenada "x = 0", entonces hacer un rectángulo vertical para que no queden marcas del sprite.
    }
    else{
      x += Vx*(t) + (0.5)*Ax*(t*t);
    }

    /*
     * Para implementar un chequeo de la posición futura que el sprite tendrá, será necesaria la fórmula de física: xf = xo + vot + (1/2)*A*t*t
     */
    xf = x + Vx*(t) + (0.5)*Ax*(t*t);

    /*
     * Si no se detecta cambios en los botones de movimiento, se suma una aceleración en dirección contraria a la que se lleva, para generar una desaceleración
     * y así llegar al punto de reposo en el aire
     */
    if(IZ_J1 == LOW && DE_J1 == LOW){ 
      Ax = 0;
      if(parado == 1){
        Vx = (0.9)*Vx;
      }
      else{
        Vx = (0.999)*Vx;
      }
    }
    
    //***********************************************************************************************************************************
    // Implementación de física (eje y)
    //***********************************************************************************************************************************
    Estado_J1 = digitalRead(Jugador_1);
    
    if(Estado_J1 == HIGH)                                         // Observar si J1 está presionado
    {
      presionado_J1 = 1;                                          // Cambiar estado de variable J1
    }
    if (Estado_J1 == LOW && presionado_J1 == 1 )                  // Detectar cuando el botón J1 se soltó
    {
      Vy -= 0.2;                                                  // Cuando se presiona un botón, este añade velocidad de hacia arriba, por lo que genera una desaceleración                             
      presionado_J1 = 0;                                          // Cambiar variable presionado_J1 como antirebote
    }
    
    if(Vy<=0.5 && Vy>=-0.5){                                          // Para evitar que el jugador comience a acelerar, se detiene la velocidad cuando esta llega a 1 o a -1 (movimiento hacia arriba)
      Vy += Ay*(t);
    }
    else if(Vy>0.5){
      Vy = 0.5;                                                     // Si la velocidad se vuelve mayor a 1, la velocidad se detiene en 1 para evitar la constante aceleración hacia abajo.
    }
    else if(Vy<-0.5){
      Vy = -0.5;                                                    // Si la velocidad se vuelve menor a -1, la velocidad se detiene en -1 para evitar la constante aceleración hacia arriba.
    }

    /* 
     * Para lograr que el sprite aparezca hacia arriba cuando este atraviesa la pared de abajo, se hizo que se fuera a la coordenada "y = 0" cuando la misma se volviera mayor a 240 (límite de pantalla).
     * Lo mismo sucede si supera la coordenada "y = 240", el sprite se mueve a la coordenada "y = 0".
     */
    if(y > 240){                                        
      y = 0;
    }
    else if(y < 0){
      y = 240;
      FillRect(0, 0, 319, 28, 0x0000);                            // Si el jugador sobrepasa la coordenada "x = 0", entonces hacer un rectángulo vertical para que no queden marcas del sprite.
    }
    else{
      y += Vy*(t) + (0.5)*Ay*(t*t);                               // Si el jugador se encuentra entre 0 y 240 en el eje y, la posición vertical está dada por esta fórmula
    }

    /*
     * Para implementar un chequeo de la posición futura que el sprite tendrá, será necesaria la fórmula de física: yf = yo + vot + (1/2)*A*t*t
     */
    yf = y + Vy*(t) + (0.5)*Ay*(t*t);

    //***********************************************************************************************************************************
    // Comparación de velocidad
    //***********************************************************************************************************************************
    if(presionado_J1 == 1){
      /*
       * Cuando el jugador presiona el botón para generar el impulso hacia arriba, la variable "presionado_J1" se vuelve 1 mientras el botón está presionado y hasta que no se suelta el botón, no se
       * genera el impulso hacia arriba. Por esto mismo se utilizó esto para generar la animación del aleteo. Cuando esta variable se vuelve 1, la animación se ve alterada y al estar corriendo el juego,
       * el personaje pareciera aletear para adquirir cierta altura.
       */
      LCD_Sprite(int(x), int(y), 16, 24, J1, 5, 2, flip, 0);      
    }
    else{
      LCD_Sprite(int(x), int(y), 16, 24, J1, 5, 3, flip, 0);
    }

    //***********************************************************************************************************************************
    // Colisiones
    //***********************************************************************************************************************************
    overlap_J1 = Check_overlap(106, 119, int(x), int(y), 107, 40, 24, 16);

    if(overlap_J1 == 1){
      region_obs1 = Regiones(int(x), int(y), 24, 16, 106, 119, 107, 40);
      
      // Si se encuentra en la región superior al obstáculo
      if(region_obs1 == 1){ 
        
        // Pintar borde superior, izquierdo y derecho.
        H_line(int(x)-2, int(y)-1, 20, 0x0000); 
        V_line(int(x)+17, int(y), 24, 0x0000);
        V_line(int(x)+18, int(y), 24, 0x0000);
        V_line(int(x)-1, int(y), 24, 0x0000);
        V_line(int(x)-2, int(y), 24, 0x0000);

        // Alteración a velocidades y aceleración                                   
        Vy = 0;
        Ay = 0;                                                   // Si no se setea la aceleración como 0, poco a poco el sprite se mete en el obstáculo
        parado = 1;
      }

      // Si se encuentra en la región central izquierda al obstáculo
      else if(region_obs1 == 2){

        // Pintar borde superior, izquierdo y posterior.
        V_line(int(x)-1, int(y), 24, 0x0000);
        H_line(int(x)-2, int(y)-1, 20, 0x0000);
        H_line(int(x)-2, int(y)+25, 20, 0x0000);

        // Alteración de la velocidad en x
        Vx = -Vx;
      }

      // Si se encuentra en la región central derecha al obstáculo
      else if(region_obs1 == 3){

        // Pintar borde inferior, derecho y superior.
        V_line(int(x)+17, int(y), 24, 0x0000);
        H_line(int(x)-2, int(y)-1, 20, 0x0000);
        H_line(int(x)-2, int(y)+25, 20, 0x0000);

        // Alteración de la velocidad en x y la aceleración en x
        Vx = -Vx;
        Ax = -Ax;
      }

      // Si se encuentra en la región posterior al obstáculo
      else if(region_obs1 == 4){

        // Pintar borde inferior, izquierdo y derecho.
        V_line(int(x)+17, int(y), 24, 0x0000);
        V_line(int(x)-1, int(y), 24, 0x0000);
        H_line(int(x)-2, int(y)-1, 20, 0x0000);

        // Alteración de la velocidad en y
        Vy = -Vy;
      }

      // Si se encuentra en una esquina
      else if (region_obs1 == 5){
        
        // Alterar la velocidad  "y"
        Vy = -1.2*Vy;
        Vx = -1.2*Vx;
      }

      /*
       * Para permitir que el jugador pueda caer libremente cuando se encuentra en una esquina, se chequea si está parado, de lo contrario, las esquinas repelen al jugador.
       */
      else {
        if(parado == 0){
          Vx = -Vx;
          Vy = -Vy;
        }
        else{
          V_line(int(x)+17, int(y), 24, 0x0000);
          V_line(int(x)+18, int(y), 24, 0x0000);
          V_line(int(x)+19, int(y), 24, 0x0000);
          V_line(int(x)-1, int(y), 24, 0x0000);
          V_line(int(x)-2, int(y), 24, 0x0000);
          V_line(int(x)-3, int(y), 24, 0x0000);
        }
      }
      
    }

    // Si no existe overlap, continuar con físicas de caida libre normal
    else{

      // Pintar todos los bordes exteriores para no dejar rastro en la LCD
      V_line(int(x)-1, int(y), 24, 0x0000);
      V_line(int(x)+17, int(y), 24, 0x0000);
      H_line(int(x)-2, int(y)-1, 20, 0x0000);
      H_line(int(x)-2, int(y)+25, 20, 0x0000);

      // Dejar la gravedad constante y la variable de si está parado, en 0
      Ay = Gravedad;
      parado = 0;
    }
    
  }

}
//***************************************************************************************************************************************
// Función para chequeo de overlap entre 1 superficie y el jugador
//***************************************************************************************************************************************
/*
 * Para esta función se busca encontrar si los obstáculos y el personaje se colocan uno sobre otro o si se da un "overlap". Se calcula a partir de indicar cuales son los 4 lados de los dos objetos  
 * para los cuales se desea encontrar su overlap. Lo bueno es que si este no detecta overlap en alguno de los dos ejes, el area siempre nos va a dar 0.
 */
int Check_overlap(int posx_poligono, int posy_poligono, int posx_jugador, int posy_jugador, int ancho_poligono, int alto_poligono, int alto_jugador, int ancho_jugador){
  int der_poligono = posx_poligono + ancho_poligono;              // Calcular lado derecho del poligono a utilizar
  int iz_poligono = posx_poligono;                                // Calcular lado izquierdo del poligono a utilizar
  int abajo_poligono = posy_poligono + alto_poligono;             // Calcular lado posterior del poligono a utilizar
  int arriba_poligono = posy_poligono;                            // Calcular lado superior del poligono a utilizar

  int der_jugador = posx_jugador + ancho_jugador;                 // Calcular lado derecho del sprite a utilizar
  int iz_jugador = posx_jugador;                                  // Calcular lado izquierdo del sprite a utilizar
  int abajo_jugador = posy_jugador + alto_jugador;                // Calcular lado posterior del sprite a utilizar
  int arriba_jugador = posy_jugador;                              // Calcular lado superior del sprite a utilizar
  
  int overlap_x = max(0, (min(der_poligono, der_jugador) - max(iz_poligono, iz_jugador)));                      // Cálculo del overlap en eje x
  int overlap_y = max(0, (min(abajo_poligono, abajo_jugador) - max(arriba_poligono, arriba_jugador)));          // Cálculo del overlap en eje y

  int area_overlap = overlap_x * overlap_y;                       // Cálculo del área total de overlap entre ambas figuras

  if(area_overlap > 0){                                           // Retornar 1 si existe un overlap y un 0 si no hay
    return 1;
  }
  else{
    return 0; 
  }
}

//***************************************************************************************************************************************
// Función para generar regiones en las que el jugador se puede mover
//***************************************************************************************************************************************
/*
 * Se creo una subrutina espcífica para generar 4 regiones en cada uno de los lados del obstáculo indicado. Esto nos permite saber de qué lado se encuentra el personaje y así descifrar en que lado
 * colisionará y como deberá reaccionar a esto.
 */
int Regiones(int posx_J1, int posy_J1, int alto_J1, int ancho_J1, int posx_obstaculo, int posy_obstaculo, int ancho_obstaculo, int alto_obstaculo){

  // Pestaña: Arriba
  if(posx_J1 >= (posx_obstaculo) && (posx_J1 + ancho_J1) <= (posx_obstaculo + ancho_obstaculo) && (posy_J1 + (0.5)*alto_J1) <= (posy_obstaculo)){
    return 1;                                                  
  }

  // Pestaña: Izquierda
  else if((posy_J1 + alto_J1) <= (posy_obstaculo + alto_obstaculo) && (posy_J1) >= (posy_obstaculo) && (posx_J1) <= posx_obstaculo){
    return 2;                                                    
  }

  // Pestaña: Derecha
  else if((posy_J1 + alto_J1) <= (posy_obstaculo + alto_obstaculo) && (posy_J1) >= (posy_obstaculo) && (posx_J1 + ancho_J1) >= (posx_obstaculo + ancho_obstaculo)){
    return 3;                                                    
  }

  // Pestaña: Abajo
  else if(posx_J1 >= (posx_obstaculo) && (posx_J1 + ancho_J1) <= (posx_obstaculo + ancho_obstaculo) && (posy_J1 + alto_J1) >= (posy_obstaculo + alto_obstaculo )){
    return 4;                                                    
  }

  else if((posx_J1 + 0.5*ancho_J1) > posx_obstaculo && (posx_J1 + 0.5*ancho_J1) < (posx_obstaculo + ancho_obstaculo) && (posy_J1 + 0.5*alto_J1) > posy_obstaculo && (posy_J1 + 0.5*alto_J1) < (posy_obstaculo + 0.5*alto_obstaculo)){
    return 5;
  } 

  // Caso no especificado
  else {
    return 6;
  }
}

//***************************************************************************************************************************************
// Función para inicializar LCD
//***************************************************************************************************************************************
void LCD_Init(void) {
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RD, OUTPUT);
  for (uint8_t i = 0; i < 8; i++) {
  pinMode(DPINS[i], OUTPUT);
  }
  //****************************************
  // Secuencia de Inicialización
  //****************************************
  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(20);
  digitalWrite(LCD_RST, HIGH);
  delay(150);
  digitalWrite(LCD_CS, LOW);
  //****************************************
  LCD_CMD(0xE9);  // SETPANELRELATED
  LCD_DATA(0x20);
  //****************************************
  LCD_CMD(0x11); // Exit Sleep SLEEP OUT (SLPOUT)
  delay(100);
  //****************************************
  LCD_CMD(0xD1);    // (SETVCOM)
  LCD_DATA(0x00);
  LCD_DATA(0x71);
  LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0xD0);   // (SETPOWER)
  LCD_DATA(0x07);
  LCD_DATA(0x01);
  LCD_DATA(0x08);
  //****************************************
  LCD_CMD(0x36);  // (MEMORYACCESS)
  LCD_DATA(0x40 | 0x80 | 0x20 | 0x08); // LCD_DATA(0x19);
  //****************************************
  LCD_CMD(0x3A); // Set_pixel_format (PIXELFORMAT)
  LCD_DATA(0x05); // color setings, 05h - 16bit pixel, 11h - 3bit pixel
  //****************************************
  LCD_CMD(0xC1);    // (POWERCONTROL2)
  LCD_DATA(0x10);
  LCD_DATA(0x10);
  LCD_DATA(0x02);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC0); // Set Default Gamma (POWERCONTROL1)
  LCD_DATA(0x00);
  LCD_DATA(0x35);
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x02);
  //****************************************
  LCD_CMD(0xC5); // Set Frame Rate (VCOMCONTROL1)
  LCD_DATA(0x04); // 72Hz
  //****************************************
  LCD_CMD(0xD2); // Power Settings  (SETPWRNORMAL)
  LCD_DATA(0x01);
  LCD_DATA(0x44);
  //****************************************
  LCD_CMD(0xC8); //Set Gamma  (GAMMASET)
  LCD_DATA(0x04);
  LCD_DATA(0x67);
  LCD_DATA(0x35);
  LCD_DATA(0x04);
  LCD_DATA(0x08);
  LCD_DATA(0x06);
  LCD_DATA(0x24);
  LCD_DATA(0x01);
  LCD_DATA(0x37);
  LCD_DATA(0x40);
  LCD_DATA(0x03);
  LCD_DATA(0x10);
  LCD_DATA(0x08);
  LCD_DATA(0x80);
  LCD_DATA(0x00);
  //****************************************
  LCD_CMD(0x2A); // Set_column_address 320px (CASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0x3F);
  //****************************************
  LCD_CMD(0x2B); // Set_page_address 480px (PASET)
  LCD_DATA(0x00);
  LCD_DATA(0x00);
  LCD_DATA(0x01);
  LCD_DATA(0xE0);
  //  LCD_DATA(0x8F);
  LCD_CMD(0x29); //display on
  LCD_CMD(0x2C); //display on

  LCD_CMD(ILI9341_INVOFF); //Invert Off
  delay(120);
  LCD_CMD(ILI9341_SLPOUT);    //Exit Sleep
  delay(120);
  LCD_CMD(ILI9341_DISPON);    //Display on
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar comandos a la LCD - parámetro (comando)
//***************************************************************************************************************************************
void LCD_CMD(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = cmd;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para enviar datos a la LCD - parámetro (dato)
//***************************************************************************************************************************************
void LCD_DATA(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_WR, LOW);
  GPIO_PORTB_DATA_R = data;
  digitalWrite(LCD_WR, HIGH);
}
//***************************************************************************************************************************************
// Función para definir rango de direcciones de memoria con las cuales se trabajara (se define una ventana)
//***************************************************************************************************************************************
void SetWindows(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) {
  LCD_CMD(0x2a); // Set_column_address 4 parameters
  LCD_DATA(x1 >> 8);
  LCD_DATA(x1);
  LCD_DATA(x2 >> 8);
  LCD_DATA(x2);
  LCD_CMD(0x2b); // Set_page_address 4 parameters
  LCD_DATA(y1 >> 8);
  LCD_DATA(y1);
  LCD_DATA(y2 >> 8);
  LCD_DATA(y2);
  LCD_CMD(0x2c); // Write_memory_start
}
//***************************************************************************************************************************************
// Función para borrar la pantalla - parámetros (color)
//***************************************************************************************************************************************
void LCD_Clear(unsigned int c) {
  unsigned int x, y;
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  SetWindows(0, 0, 319, 239); // 479, 319);
  for (x = 0; x < 320; x++)
    for (y = 0; y < 240; y++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);
    }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea horizontal - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void H_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + x;
  SetWindows(x, y, l, y);
  j = l;// * 2;
  for (i = 0; i < l; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una línea vertical - parámetros ( coordenada x, cordenada y, longitud, color)
//***************************************************************************************************************************************
void V_line(unsigned int x, unsigned int y, unsigned int l, unsigned int c) {
  unsigned int i, j;
  LCD_CMD(0x02c); //write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);
  l = l + y;
  SetWindows(x, y, x, l);
  j = l; //* 2;
  for (i = 1; i <= j; i++) {
    LCD_DATA(c >> 8);
    LCD_DATA(c);
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
void Rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  H_line(x  , y  , w, c);
  H_line(x  , y + h, w, c);
  V_line(x  , y  , h, c);
  V_line(x + w, y  , h, c);
}
//***************************************************************************************************************************************
// Función para dibujar un rectángulo relleno - parámetros ( coordenada x, cordenada y, ancho, alto, color)
//***************************************************************************************************************************************
/*void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  unsigned int i;
  for (i = 0; i < h; i++) {
    H_line(x  , y  , w, c);
    H_line(x  , y+i, w, c);
  }
  }
*/

void FillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int c) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + w;
  y2 = y + h;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = w * h * 2 - 1;
  unsigned int i, j;
  for (int i = 0; i < w; i++) {
    for (int j = 0; j < h; j++) {
      LCD_DATA(c >> 8);
      LCD_DATA(c);

      //LCD_DATA(bitmap[k]);
      k = k - 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar texto - parámetros ( texto, coordenada x, cordenada y, color, background)
//***************************************************************************************************************************************
void LCD_Print(String text, int x, int y, int fontSize, int color, int background) {
  int fontXSize ;
  int fontYSize ;

  if (fontSize == 1) {
    fontXSize = fontXSizeSmal ;
    fontYSize = fontYSizeSmal ;
  }
  if (fontSize == 2) {
    fontXSize = fontXSizeBig ;
    fontYSize = fontYSizeBig ;
  }

  char charInput ;
  int cLength = text.length();
  Serial.println(cLength, DEC);
  int charDec ;
  int c ;
  int charHex ;
  char char_array[cLength + 1];
  text.toCharArray(char_array, cLength + 1) ;
  for (int i = 0; i < cLength ; i++) {
    charInput = char_array[i];
    Serial.println(char_array[i]);
    charDec = int(charInput);
    digitalWrite(LCD_CS, LOW);
    SetWindows(x + (i * fontXSize), y, x + (i * fontXSize) + fontXSize - 1, y + fontYSize );
    long charHex1 ;
    for ( int n = 0 ; n < fontYSize ; n++ ) {
      if (fontSize == 1) {
        charHex1 = pgm_read_word_near(smallFont + ((charDec - 32) * fontYSize) + n);
      }
      if (fontSize == 2) {
        charHex1 = pgm_read_word_near(bigFont + ((charDec - 32) * fontYSize) + n);
      }
      for (int t = 1; t < fontXSize + 1 ; t++) {
        if (( charHex1 & (1 << (fontXSize - t))) > 0 ) {
          c = color ;
        } else {
          c = background ;
        }
        LCD_DATA(c >> 8);
        LCD_DATA(c);
      }
    }
    digitalWrite(LCD_CS, HIGH);
  }
}
//***************************************************************************************************************************************
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) Formato (Color 16bit R 5bits G 6bits B 5bits)
//***************************************************************************************************************************************
void LCD_Bitmap(unsigned int x, unsigned int y, unsigned int width, unsigned int height, unsigned char bitmap[]) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 = x + width;
  y2 = y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  unsigned int k = 0;
  unsigned int i, j;

  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      LCD_DATA(bitmap[k]);
      LCD_DATA(bitmap[k + 1]);
      //LCD_DATA(bitmap[k]);
      k = k + 2;
    }
  }
  digitalWrite(LCD_CS, HIGH);
}
//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta
//***************************************************************************************************************************************
void LCD_Sprite(int x, int y, int width, int height, unsigned char bitmap[], int columns, int index, char flip, char offset) {
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW);

  unsigned int x2, y2;
  x2 =   x + width;
  y2 =    y + height;
  SetWindows(x, y, x2 - 1, y2 - 1);
  int k = 0;
  int ancho = ((width * columns));
  if (flip) {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width - 1 - offset) * 2;
      k = k + width * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k - 2;
      }
    }
  } else {
    for (int j = 0; j < height; j++) {
      k = (j * (ancho) + index * width + 1 + offset) * 2;
      for (int i = 0; i < width; i++) {
        LCD_DATA(bitmap[k]);
        LCD_DATA(bitmap[k + 1]);
        k = k + 2;
      }
    }


  }
  digitalWrite(LCD_CS, HIGH);
}
