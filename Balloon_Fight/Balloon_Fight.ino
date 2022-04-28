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

File Archivo;

//***************************************************************************************************************************************
// Definiciones de pines
//***************************************************************************************************************************************
#define LCD_RST PD_0
#define LCD_CS PD_1
#define LCD_RS PD_2
#define LCD_WR PD_3
#define LCD_RD PE_1
#define Izquierda_J1 PA_7
#define Derecha_J1 PC_7
#define Gravedad 0.0026
#define Game PC_6
#define Game_over PC_7
#define Menu PC_5
#define Start PE_4

//***************************************************************************************************************************************
// Structs
//***************************************************************************************************************************************
typedef struct{
  float Vx;
  float Vy;
  float Ax;
  float Ay;
}
vel_acel;

typedef struct{
  float Px;
  float Py;
  int Ancho;
  int Alto;
  float Vx;
  float Vy;
  float Ax;
  float Ay;
  int flip;
  int parado;
  int impulso;
  int Num_globos;
  int Muerto;
}
Jugador;

typedef struct{
  int Px;
  int Py;
  int Ancho;
  int Alto;
}
Obstaculo;

typedef struct{
  int Izquierda;
  int Derecha;
  int Impulso;
}
Control;

//***************************************************************************************************************************************
// Variables
//***************************************************************************************************************************************
int DPINS[] = {PB_0, PB_1, PB_2, PB_3, PB_4, PB_5, PB_6, PB_7};
int volando = 0;
int overlap_J1 = 0;
int region_obs1 = 0;
int vector_x1 = 0;
int vector_y1 = 0;

  //***********************************************************************************************************************************
  // Variables comunes
  //***********************************************************************************************************************************
  int Game_Over = 0;
  float t = 1.5;
  int cont_anim1 = 0;
  int cont_anim2 = 0;
  int anim1 = 0;
  int anim2 = 0;
  int anim3 = 0;
  int Datos_control_binario = 0b000000;
  int datos = 0;
  int Suma_vel = 0;
  int Menu_activo = 1;
  int agua = 0;
  

  //***********************************************************************************************************************************
  // Declaración de objetos
  //***********************************************************************************************************************************
  Obstaculo obs1 = {120, 96, 90, 25};                                     // Se crea un objeto tipo obstáculo
  Obstaculo obs2 = {0, 198, 66, 44};                                      // Se crea un objeto tipo obstáculo
  Obstaculo obs3 = {253, 198, 66, 44};                                    // Se crea un objeto tipo obstáculo       
  Jugador J1 = {50, 100, 16, 24, 0, 0, 0, Gravedad, 0, 0, 0, 2, 0};       // Objeto tipo "Jugador" con todos los parámetros para el mismo
  Jugador J2 = {257, 100, 16, 24, 0, 0, 0, Gravedad, 1, 0, 0, 2, 0};      // Objeto tipo "Jugador" con todos los parámetros para el mismo
  Control CTRL1 = {0, 0, 0};                                              // Objeto tipo control que guarda todos los parámetros mandados por el control 1
  Control CTRL2 = {0, 0, 0};                                              // Objeto tipo control que guarda todos los parámetros mandados por el control 2

//***************************************************************************************************************************************
// Functions Prototypes
//***************************************************************************************************************************************
void LCD_Init(void);
void LCD_CMD(uint8_t cmd);
void LCD_DATA(uint8_t data);
void LCD_BitmapSD(int x, int y, int width, int height, String TXT);

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
Jugador Colisiones(Jugador Jug, Obstaculo obs);
Jugador Fisicas_x(Jugador Jug, int Boton_Iz, int Boton_Der);
Jugador Fisicas_y(Jugador Jug, int Boton_impulso);
Jugador Fisicas(Jugador Jug, int Boton_impulso, int Boton_Iz, int Boton_Der);
Jugador Fisica_muerte(Jugador Jug);
void Write_Text(String Texto, int CoordX, int CoordY, int ColorTexto, int ColorFondo);
void LCD_TextCS(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset, int ColorTexto, int ColorFondo);
void Write_Num(int Numero, byte NoDigitos, int CoordX, int CoordY, int ColorTexto, int ColorFondo);
void LCD_SpriteCS(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset, char colorswap, int OldColor, int NewColor);
void Animaciones(Jugador Jug, int Num_Jugador);

//***************************************************************************************************************************************
// Setup: Menú principal
//***************************************************************************************************************************************
void setup() {
  
  SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  Serial.begin(115200);
  Serial3.begin(9600);
  Serial2.begin(115200);
  
  GPIOPadConfigSet(GPIO_PORTB_BASE, 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
  Serial.println("Inicio");
  SPI.setModule(0);
  LCD_Init();
  LCD_Clear(0x00);

  if (!SD.begin(PA_3)) Serial.println("Inicialización SD: Inicialización fallida!");
  else Serial.println("Inicialización SD: Inicialización existosa!");

  pinMode(Start, INPUT);

  pinMode(Game, OUTPUT);
  pinMode(Game_over, OUTPUT);
  pinMode(Menu, OUTPUT);


  FillRect(0, 0, 319, 239, 0x0000);
  
  //*************************************************************************************************************************************
  // Menú principal
  //*************************************************************************************************************************************
  /*
   * Se creó una variable llamada "Menu activo", la cual hace que el menú principal permanezca colocado durante todo el tiempo que el usuario
   * no presione el botón indicado de inicio.
   */
  while(Menu_activo == 1){
    
    // Se realiza la subrutina LCD_BitmapSD, para que se extraiga el título y el símbolo de globo de ahí 
    LCD_BitmapSD(50, 35, 224, 104, "Titulo.txt");
    LCD_BitmapSD(100, 165, 16, 16, "Globito.txt");
    
    // Escritura del texto "Press start" y las credenciales a Nintendo 
    Write_Text("PRESS START", 120, 170, 0xffff, 0x0000);
    Write_Text("c", 105, 220, 0xffff, 0x0000);
    Write_Num(1984, 4, 113, 220, 0xffff, 0x0000);
    Write_Text(" NINTENDO", 145, 220, 0xffff, 0x0000);

    
    // Chequear el valor de los controles para saber si se presionó el botón "A" para ingresar al juego
    while(Serial2.available()){
      
      // Guardar las variables que el control envia
      Datos_control_binario = Serial2.read();

      // Si el botón de impulso del jugador 1 es presionado, el menú dejará de estar disponible y se podrá jugar 
      if(Datos_control_binario == 8){
        Menu_activo = 0;
      }
    }
    
  }
  

  //*************************************************************************************************************************************
  // Impresión del campo de batalla entre jugadores
  //*************************************************************************************************************************************
  LCD_Clear(0x00);

  /*
   * Se realizó un for para cada uno de los obstaculos impresos. Esto se debe a que los obstáculos son de tamaño reducido, por lo que hay que dibujarlos cada ciertos espacios
   * para alcanzar el tamaño de obstáculo requerido.
   */

  // Dibujar obstaculo central aereo
  for(int i = 0; i < 13; i++){
    LCD_Bitmap((obs1.Px + 4) + 6*i, obs1.Py + 4, 6, 19, Obstaculo_aire);                
  }

  // Dibujar obstaculo de la esquina inferior izquierda
  for(int j = 0; j < 9; j++){
    LCD_Bitmap((obs2.Px) + 7*j, obs2.Py + 4, 7, 37, Obstaculo_esquina);
  }

  // Dibujar obstaculo de la esquina inferior derecha
  for(int h = 0; h < 9; h++){
    LCD_Bitmap((250 + 7) + 7*h, obs3.Py + 4, 7, 37, Obstaculo_esquina);
  }

  // Dibujar el agua central
  for(int a = 0; a < 20; a++){
    LCD_Bitmap(61 + 10*a, 217, 10, 22, Agua);
  }


}
//***************************************************************************************************************************************
// Loop Infinito
//***************************************************************************************************************************************
void loop() {

  // Se inicializa al jugador como vivo
  J1.Muerto = 0;

  //***********************************************************************************************************************************
  // Loop mientras ningún jugador ha muerto
  //***********************************************************************************************************************************
  while(Game_Over == 0) {
    
    //***********************************************************************************************************************************
    // Musica
    //***********************************************************************************************************************************
    digitalWrite(Game, LOW);
    
    //***********************************************************************************************************************************
    // Lectura de botones provenientes de los controles (ESP32)
    //***********************************************************************************************************************************
    while(Serial2.available()){

      // Guardar las variables que el control envia
      Datos_control_binario = Serial2.read();
    }

    Serial.print(" Datos control: ");
    Serial.println(Datos_control_binario);
    
    // Verificar cual es el botón presionado
    if((Datos_control_binario & 0b100000) == 0b100000){ CTRL1.Izquierda = 1; } else { CTRL1.Izquierda = 0; }
    if((Datos_control_binario & 0b010000) == 0b010000){ CTRL1.Derecha = 1; } else { CTRL1.Derecha = 0; }
    if((Datos_control_binario & 0b001000) == 0b001000){ CTRL1.Impulso = 1; } else { CTRL1.Impulso = 0; }
    if((Datos_control_binario & 0b000100) == 0b000100){ CTRL2.Izquierda = 1; } else { CTRL2.Izquierda = 0; }
    if((Datos_control_binario & 0b000010) == 0b000010){ CTRL2.Derecha = 1; } else { CTRL2.Derecha = 0; }
    if((Datos_control_binario & 0b000001) == 0b000001){ CTRL2.Impulso = 1; } else { CTRL2.Impulso = 0; }

    
    //***********************************************************************************************************************************
    // Implementación de físicas (ambos ejes) para ambos jugadores. Esto depende de si el jugador está muerto o no
    //***********************************************************************************************************************************
    if(J1.Muerto == 0 && J2.Muerto == 0){
      J1 = Fisicas(J1, CTRL1.Impulso, CTRL1.Izquierda, CTRL1.Derecha);
      J2 = Fisicas(J2, CTRL2.Impulso, CTRL2.Izquierda, CTRL2.Derecha);
    }
    else{
      if(J1.Muerto == 1){ 

        // Impulso inicial para jugador 1
        if(Suma_vel == 0){                                                      
          J1.Vy = -0.3;
          J1.Py += 8;
          Suma_vel = 1;
        }
      
        J1 = Fisica_muerte(J1);

        if((J1.Py + J1.Alto - 8) > 250){
          J1.Py = 240;
          J1.Muerto = 5;
  
          for(int e = 0; e < 20; e++){
            LCD_Bitmap(61 + 10*e, 217, 10, 22, Agua);
          }

          Game_Over = 1;
          FillRect(0, 0, 319, 239, 0x0000);
          break;
        }
      }
      else{
        
        // Impulso inicial para jugador 2
        if(Suma_vel == 0){
          J2.Vy = -0.3;
          J2.Py += 8;
          Suma_vel = 1;
        }
        
        J2 = Fisica_muerte(J2);
  
        if((J2.Py + J2.Alto - 8) > 250){
          J2.Py = 240;
          J2.Muerto = 5;
  
          for(int o = 0; o < 20; o++){
            LCD_Bitmap(61 + 10*o, 217, 10, 22, Agua);
          }

          Game_Over = 1;
          FillRect(0, 0, 319, 239, 0x0000);
          break;
        }
      }
    }
    
    //***********************************************************************************************************************************
    // Sprites y animaciones
    //***********************************************************************************************************************************
    /*
     * En este caso evaluamos el estado del jugador. Si este se encuentra sobre una superficie, si está parado sin moverse, genera una animación y si se mueve en el eje x, se genera una animación de  
     * caminata para que el jugador tenga movilidad. Ahora esto fue modulado para que 2 jugadores pudieran tener las mismas animaciones
     */
    Animaciones(J1, 1);
    Animaciones(J2, 2);

    //***********************************************************************************************************************************
    // Colisiones Primer nivel
    //***********************************************************************************************************************************
    /*
     * Se dividió en diferentes regiones la pantalla (en eje x) para que por región realizara las colisiones con el obstáculo que se encuentra en esa misma región. Esto se realizó para ambos jugadores. 
     */

    // Jugador 1 colisiones
    if((J1.Px + J1.Ancho) > 0 && J1.Px < (obs2.Px + obs2.Ancho + 18)){
      J1 = Colisiones(J1, obs2);
    }
    else if((J1.Px + J1.Ancho) > (obs2.Px + obs2.Ancho + 18) && J1.Px < (obs1.Px + obs1.Ancho + 18)){
      J1 = Colisiones(J1, obs1);
    }
    else if((J1.Px + J1.Ancho) > (obs1.Px + obs1.Ancho + 18) && J1.Px < 319){
      J1 = Colisiones(J1, obs3);
    }

    // Jugador 2 colisiones
    if((J2.Px + J2.Ancho) > 0 && J2.Px < (obs2.Px + obs2.Ancho + 18)){
      J2 = Colisiones(J2, obs2);
    }
    else if((J2.Px + J2.Ancho) > (obs2.Px + obs2.Ancho + 18) && J2.Px < (obs1.Px + obs1.Ancho + 18)){
      J2 = Colisiones(J2, obs1);
    }
    else if((J2.Px + J2.Ancho) > (obs1.Px + obs1.Ancho + 18) && J2.Px < 319){
      J2 = Colisiones(J2, obs3);
    }

    //***********************************************************************************************************************************
    // Muerte
    //***********************************************************************************************************************************
    if((J1.Py + J1.Alto) > 217){
      J1.Muerto = 1;
    }

    if((J2.Py + J2.Alto) > 217){
      J2.Muerto = 1;
    }
    
  }

  //*************************************************************************************************************************************
  // Entrar en pantalla de ganadores
  //*************************************************************************************************************************************
  while(Game_Over == 1){
    cont_anim1 += 1;

    /*
     * Agua para ambos escenarios de ganadores. Se utiliza la variable agua para pintar el agua una sola vez
     */
    if(agua == 0){
      for(int l = 0; l < 60; l++){
        LCD_Bitmap(0 + 10*l, 217, 10, 22, Agua);
      }
      
      for(int p = 0; p < 60; p++){
        LCD_Bitmap(0 + 10*p, 207, 10, 22, Agua);
      }
      
      for(int c = 0; c < 60; c++){
        LCD_Bitmap(0 + 10*c, 197, 10, 22, Agua);
      }

      agua = 1;
    }
    

    // Si el jugador 1 sigue vivo, pero el jugador 2 ya pasó por la animación de muerte, entonces generar una pantalla de 
    if(J1.Muerto == 0 && J2.Muerto == 5){
      for(int x = 0; x < 6; x++){
        LCD_Bitmap(70 + 6*x, 125, 6, 19, Obstaculo_aire);                
      }

      /*
       * Se dibuja al jugador ganador (J1)
       */
      anim1 = (cont_anim1 / 55) % 3;
      LCD_Sprite(80, 100, J1.Ancho, J1.Alto, Balloon_boy_parado, 3, anim1, 1, 0);
      Write_Text("PLAYER ONE WINS!", 130, 110, 0xffff, 0x0000);
    }

    if(J1.Muerto == 5 && J2.Muerto == 0){
      for(int x = 0; x < 6; x++){
        LCD_Bitmap(70 + 6*x, 125, 6, 19, Obstaculo_aire);                
      }

      /*
       * Se dibuja al jugador ganador (J2)
       */
      anim1 = (cont_anim1 / 55) % 3;
      LCD_SpriteCS(80, 100, J2.Ancho, J2.Alto, Balloon_boy_parado, 3, anim1, 1, 0, 1, 0x21dd, 0x2CC5);
      Write_Text("PLAYER TWO WINS!", 130, 110, 0xffff, 0x0000);
    }
  }

}

//---------------------------------------------------- Subrutinas -----------------------------------------------------------------------
//***************************************************************************************************************************************
// Función que generar físicas cuando el jugador murió
//***************************************************************************************************************************************
void Animaciones(Jugador Jug, int Num_Jugador){

  // Utilizar timers para animaciones
  cont_anim1 += 1;
  
  if(Jug.Num_globos == 2){
    
    // Si el jugador se encuentra sobre una superficie:
    if(Jug.parado == 1 && Jug.Muerto == 0){

      // Si la velocidad en x se aproxima a 0:
      if(abs(Jug.Vx) < 0.001){

        // Generar animación para estar parado
        anim1 = (cont_anim1 / 35) % 3;

        if(Num_Jugador == 1){
          LCD_Sprite(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy_parado, 3, anim1, Jug.flip, 0);
        }
        else{
          LCD_SpriteCS(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy_parado, 3, anim1, Jug.flip, 0, 1, 0x21dd, 0x2CC5);
        }
        
      }

      // Si la velocidad en x es mayor o menor a 0:
      else{

        // Generar animación para caminar
        anim2 = (cont_anim1 / 20) % 4;
        
        if(Num_Jugador == 1){
          LCD_Sprite(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy_caminando, 4, anim2, Jug.flip, 0);
        }
        else{
          LCD_SpriteCS(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy_caminando, 4, anim2, Jug.flip, 0, 1, 0x21dd, 0x2CC5);
        }

        // Si la velocidad es negativa, dibujar franjas para evitar ver desfase.
        if(Jug.Vx < 0){
          V_line(Jug.Px +15, Jug.Py, 24, 0x0000);
          V_line(Jug.Px +16, Jug.Py, 24, 0x0000);
        }
      }
    }

    // Si el jugador muere, entonces realizar la animación correspondiente
    else if(Jug.Muerto == 1){
      
      cont_anim2 += 1;
      anim3 = (cont_anim2/68) % 4;
      
      if(Num_Jugador == 1){
        LCD_Sprite(int(Jug.Px), int(Jug.Py), 15, 12, Muerte, 4, anim3, Jug.flip, 0);
      }
      else{
        LCD_SpriteCS(int(Jug.Px), int(Jug.Py), 15, 12, Muerte, 4, anim3, Jug.flip, 0, 1, 0x21dd, 0x2CC5);
      }
      
      V_line(Jug.Px -1, Jug.Py, 12, 0x0000);
      V_line(Jug.Px +15, Jug.Py, 12, 0x0000);
      H_line(Jug.Px -2, Jug.Py -1, 19, 0x0000);
      H_line(Jug.Px -2, Jug.Py +12, 19, 0x0000);
    }
    
    // Si el jugador no está sobre una superficie, realizar la animación de estar volando.
    else if(Jug.Muerto != 5){
      if(Jug.impulso == 1){

        // Si es el primer jugador, realizar la animación sin cambio de color
        if(Num_Jugador == 1){
          LCD_Sprite(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy, 5, 2, Jug.flip, 0);
        }

        // Si es el segundo jugador, realizar la animación con cambio de color
        else{
          LCD_SpriteCS(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy, 5, 2, Jug.flip, 0, 1, 0x21dd, 0x2CC5);
        }     
      }
      else{

        // Si es el primer jugador, realizar la animación sin cambio de color
        if(Num_Jugador == 1){
          LCD_Sprite(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy, 5, 3, Jug.flip, 0);
        }

        // Si es el segundo jugador, realizar la animación con cambio de color
        else{
          LCD_SpriteCS(int(Jug.Px), int(Jug.Py), Jug.Ancho, Jug.Alto, Balloon_boy, 5, 3, Jug.flip, 0, 1, 0x21dd, 0x2CC5);
        }
      }
    }
  }
}    
    
//***************************************************************************************************************************************
// Función que generar físicas cuando el jugador murió
//***************************************************************************************************************************************
Jugador Fisica_muerte(Jugador Jug){
  // Se genera una gravedad de 0.001 y velocidad inicial de -0.2
  Jug.Ay = 0.0018;
  
  if((Jug.Vy)<=1 && (Jug.Vy)>=-1){                          // Para evitar que el jugador comience a acelerar, se detiene la velocidad cuando esta llega a 1 o a -1 (movimiento hacia arriba)
    (Jug.Vy) += (Jug.Ay)*(t);
  }
  else if((Jug.Vy)>1){
    (Jug.Vy) = 1;                                             // Si la velocidad se vuelve mayor a 1, la velocidad se detiene en 1 para evitar la constante aceleración hacia abajo.
  }
  else if((Jug.Vy)<-1){
    (Jug.Vy) = -1;                                            // Si la velocidad se vuelve menor a -1, la velocidad se detiene en -1 para evitar la constante aceleración hacia arriba.
  }

  (Jug.Py) += (Jug.Vy)*(t) + (0.5)*(Jug.Ay)*(t*t);
  
  return(Jug);
}

//***************************************************************************************************************************************
// Función que genera el conjunto de físicas para 1 solo jugador
//***************************************************************************************************************************************
Jugador Fisicas(Jugador Jug, int Boton_impulso, int Boton_Iz, int Boton_Der){
  
  Jug = Fisicas_x(Jug, Boton_Iz, Boton_Der);
  Jug = Fisicas_y(Jug, Boton_impulso);

  return(Jug);
}

//***************************************************************************************************************************************
// Función para generar físicas en eje y
//***************************************************************************************************************************************
Jugador Fisicas_y(Jugador Jug, int Boton_impulso){
  if(Boton_impulso == 1)                                       // Observar si el botón de impulso está presionado
    {
      Jug.impulso = 1;                                            // Cambiar estado de variable Jug.impulso
    }
    if (Boton_impulso == 0 && Jug.impulso == 1 )                // Detectar cuando el botón de impulso se soltó
    {
      (Jug.Vy) -= 0.3;                                            // Cuando se presiona un botón, este añade velocidad de hacia arriba, por lo que genera una desaceleración                             
      Jug.impulso = 0;                                            // Cambiar variable presionado_J1 como antirebote
    }
    
    if((Jug.Vy)<=1 && (Jug.Vy)>=-1){                          // Para evitar que el jugador comience a acelerar, se detiene la velocidad cuando esta llega a 1 o a -1 (movimiento hacia arriba)
      (Jug.Vy) += (Jug.Ay)*(t);
    }
    else if((Jug.Vy)>1){
      (Jug.Vy) = 1;                                             // Si la velocidad se vuelve mayor a 1, la velocidad se detiene en 1 para evitar la constante aceleración hacia abajo.
    }
    else if((Jug.Vy)<-1){
      (Jug.Vy) = -1;                                            // Si la velocidad se vuelve menor a -1, la velocidad se detiene en -1 para evitar la constante aceleración hacia arriba.
    }

    /* 
     * Para lograr que el sprite aparezca hacia arriba cuando este atraviesa la pared de abajo, se hizo que se fuera a la coordenada "y = 0" cuando la misma se volviera mayor a 240 (límite de pantalla).
     * Lo mismo sucede si supera la coordenada "y = 240", el sprite se mueve a la coordenada "y = 0".
     */
    if((Jug.Py) > 0){
      (Jug.Py) += (Jug.Vy)*(t) + (0.5)*(Jug.Ay)*(t*t);            // Si el jugador se encuentra entre 0 y 240 en el eje y, la posición vertical está dada por esta fórmula 
    }
    else {
      (Jug.Py) = 0.1;
      (Jug.Vy) = -(0.87)*(Jug.Vy);
    }
    
    return(Jug);
}

//***************************************************************************************************************************************
// Función para generar físicas en eje x
//***************************************************************************************************************************************
Jugador Fisicas_x(Jugador Jug, int Boton_Iz, int Boton_Der){
    if(Boton_Iz == 1)                                             // En el eje x, cuando se presiona el botón para mover a la izquierda, se agrega una velocidad en el eje x-, por lo va a la izquierda
    {
      (Jug.Vx) -= 0.004;                                                 
      (Jug.Ax) = -0.006;                                         // Se genera una aceleración para un giro más suave
      Jug.flip = 0;                                               // Se utiliza la variable flip para que el sprite haga un flip dependiendo de la dirección a la que vaya
    }
    if(Boton_Der == 1){                                           // En el eje x, cuando se presiona el botón para mover a la derecha, se agrega una velocidad en el eje x+, por lo va a la derecha
      (Jug.Vx) += 0.004;
      (Jug.Ax) = 0.006;                                          // Se genera una aceleración para un giro más suave
      Jug.flip = 1;                                               // Se utiliza la variable flip para que el sprite haga un flip dependiendo de la dirección a la que vaya
    }

    if((Jug.Vx)<=0.5 && (Jug.Vx)>=-0.5){                          // Para evitar que el jugador comience a acelerar, se detiene la velocidad cuando esta llega a 1 o a -1 (movimiento hacia arriba)
      (Jug.Vx) += (Jug.Ax)*(t);
    }
    else if((Jug.Vx)>0.5){
      (Jug.Vx) = 0.5;                                             // Si la velocidad se vuelve mayor a 1, la velocidad se detiene en 1 para evitar la constante aceleración hacia abajo.
    }
    else if(Jug.Vx<-0.5){
      (Jug.Vx) = -0.5;                                            // Si la velocidad se vuelve menor a -1, la velocidad se detiene en -1 para evitar la constante aceleración hacia arriba.
    }

    /* 
     * Para lograr que el sprite aparezca hacia arriba cuando este atraviesa la pared de abajo, se hizo que se fuera a la coordenada "x = 0" cuando la misma se volviera mayor a 320 (límite de pantalla).
     * Lo mismo sucede si supera la coordenada "x = 320", el sprite se mueve a la coordenada "x = 0".
     */
    if((Jug.Px) > 320){
      (Jug.Px) = 0;
    }
    else if((Jug.Px) < 0){
      (Jug.Px) = 320;
      FillRect(0, 0, 19, 200, 0x0000);                            // Si el jugador sobrepasa la coordenada "x = 0", entonces hacer un rectángulo vertical para que no queden marcas del sprite.
    }
    else{
      (Jug.Px) += (Jug.Vx)*(t) + (0.5)*(Jug.Ax)*(t*t);
    }

    /*
     * Si no se detecta cambios en los botones de movimiento, se suma una aceleración en dirección contraria a la que se lleva, para generar una desaceleración
     * y así llegar al punto de reposo en el aire
     */
    if(Boton_Iz == 0 && Boton_Der == 0){ 
      (Jug.Ax) = 0;
      if(Jug.parado == 1){
        (Jug.Vx) = (0.9)*(Jug.Vx);
      }
      else{
        (Jug.Vx) = (0.995)*(Jug.Vx);
      }
    }

    return(Jug);
    
}

//***************************************************************************************************************************************
// Función para generar reacciones para colisiones
//***************************************************************************************************************************************
Jugador Colisiones(Jugador Jug, Obstaculo obs){
  
  // Se chequea el overlap entre el jugador indicado y el obstáculo indicado
  int overlap = Check_overlap(obs.Px, obs.Py, Jug.Px, Jug.Py, obs.Ancho, obs.Alto, Jug.Alto, Jug.Ancho);
  
  // Si existe overlap
  if(overlap == 1){ 

      // Chequeo de que region es en la que se encuentra
      int variable_region = Regiones(Jug.Px, Jug.Py, Jug.Alto, Jug.Ancho, obs.Px, obs.Py, obs.Ancho, obs.Alto);
       
      // Si se encuentra en la región superior al obstáculo
      if(variable_region == 1){ 
            
        // Pintar borde superior, izquierdo y derecho.
        H_line(Jug.Px -2, Jug.Py -1, 20, 0x0000); 
        V_line(Jug.Px +17, Jug.Py, 24, 0x0000);
        V_line(Jug.Px +18, Jug.Py, 24, 0x0000);
        V_line(Jug.Px -1, Jug.Py, 24, 0x0000);
        V_line(Jug.Px -2, Jug.Py, 24, 0x0000);
    
        // Alteración a velocidades y aceleración                                   
        Jug.Vy = 0;
        Jug.Ay = 0;                                                   // Si no se setea la aceleración como 0, poco a poco el sprite se mete en el obstáculo
        Jug.parado = 1;
      }
    
      // Si se encuentra en la región central izquierda al obstáculo
      else if(variable_region == 2){
    
        // Pintar borde superior, izquierdo y posterior.
        V_line(Jug.Px -1, Jug.Py, 24, 0x0000);
        H_line(Jug.Px -2, Jug.Py -1, 20, 0x0000);
        H_line(Jug.Px -2, Jug.Py +25, 20, 0x0000);
      
        // Alteración de la velocidad en x
        Jug.Vx = ((-0.88)*(Jug.Vx));
      }
    
      // Si se encuentra en la región central derecha al obstáculo
      else if(variable_region == 3){
    
        // Pintar borde inferior, derecho y superior.
        V_line(Jug.Px +17, Jug.Py, 24, 0x0000);
        H_line(Jug.Px -2, Jug.Py -1, 20, 0x0000);
        H_line(Jug.Px -2, Jug.Py +25, 20, 0x0000);
    
        // Alteración de la velocidad en x y la aceleración en x
        Jug.Vx = ((-0.88)*(Jug.Vx));
        Jug.Ax = ((-0.88)*(Jug.Ax));
      }
    
      // Si se encuentra en la región posterior al obstáculo
      else if(variable_region == 4){
    
        // Pintar borde inferior, izquierdo y derecho.
        V_line(Jug.Px +17, Jug.Py, 24, 0x0000);
        V_line(Jug.Px -1, Jug.Py, 24, 0x0000);
        H_line(Jug.Px -2, Jug.Py -1, 20, 0x0000);
    
        // Alteración de la velocidad en y
        Jug.Vy = ((-0.88)*(Jug.Vy));
      }
    
      // Si se encuentra en una esquina
      else if (variable_region == 5){
        
        // Alterar la velocidad  "y"
        Jug.Vy = ((-0.88)*(Jug.Vy));
        Jug.Vx = ((-0.88)*(Jug.Vx));;
      }
    
      /*
       * Para permitir que el jugador pueda caer libremente cuando se encuentra en una esquina, se chequea si está parado, de lo contrario, las esquinas repelen al jugador.
       */
      else {
        if(Jug.parado == 0){
          Jug.Vx = ((-0.88)*(Jug.Vx));
          Jug.Vy = ((-0.88)*(Jug.Vy));
        }
        else{
          V_line(Jug.Px +17, Jug.Py, 24, 0x0000);
          V_line(Jug.Px +18, Jug.Py, 24, 0x0000);
          V_line(Jug.Px +19, Jug.Py, 24, 0x0000);
          V_line(Jug.Px -1, Jug.Py, 24, 0x0000);
          V_line(Jug.Px -2, Jug.Py, 24, 0x0000);
          V_line(Jug.Px -3, Jug.Py, 24, 0x0000);

          Jug.Vx = ((1)*(Jug.Vx));
          Jug.Vy = ((1)*(Jug.Vy));
          }
       }
  }
  // Si no existe overlap, continuar con físicas de caida libre normal
  else{

    // Pintar todos los bordes exteriores para no dejar rastro en la LCD
    V_line(Jug.Px -1, Jug.Py, 24, 0x0000);
    V_line(Jug.Px +17, Jug.Py, 24, 0x0000);
    H_line(Jug.Px -2, Jug.Py -1, 20, 0x0000);
    H_line(Jug.Px -2, Jug.Py -2, 20, 0x0000);
    H_line(Jug.Px -2, Jug.Py +25, 20, 0x0000);
    H_line(Jug.Px -2, Jug.Py +26, 20, 0x0000);

    // Dejar la gravedad constante y la variable de si está parado, en 0
    Jug.Ay = Gravedad;
    Jug.parado = 0; 
  }

  return(Jug);
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
// Función para dibujar en la LCD, el texto que querramos
//***************************************************************************************************************************************
void Write_Text(String Texto, int CoordX, int CoordY, int ColorTexto, int ColorFondo) {
 
  int NoCaracteres = Texto.length();                        // Se obtiene el número de caracteres en el String escrito
  char Caracteres[NoCaracteres + 1];                        // Se crea un array con el tamaño del número de caracteres
  Texto.toCharArray(Caracteres, NoCaracteres + 1);          // Se almacena cada caracter individual 

  byte Flip = 1;                                            // Si se ingresa 0, el caracter se "flipea" horizontalmente. Solo utilizado para el triángulo apuntando a la izquierda
  byte IndiceSprite = 10;                                   // Si se ingresa un valor válido, el algoritmo coloca como default un espacio
  for (int i = 0; i < NoCaracteres; i++){
    switch(Caracteres[i]){
      case 99:  IndiceSprite = 0; break;                     // ASCII: 'c'  |  Sprite: Copyright                           
      case 62:  IndiceSprite = 1; break;                     // ASCII: '>'  |  Sprite: ->
      case 42:  IndiceSprite = 2; break;                     // ASCII: '*'  |  Sprite: "
      case 41:  IndiceSprite = 3; break;                     // ASCII: ')'  |  Sprite: )
      case 40:  IndiceSprite = 4; break;                     // ASCII: '('  |  Sprite: (
      case 91:  IndiceSprite = 5; break;                     // ASCII: '['  |  Sprite: Guión pegado a la izquierda
      case 93:  IndiceSprite = 6; break;                     // ASCII: ']'  |  Sprite: Guión pegado a la derecha
      case 33:  IndiceSprite = 7; break;                     // ASCII: '!'  |  Sprite: !
      case 47:  IndiceSprite = 8; break;                     // ASCII: '/'  |  Sprite: /
      case 123: IndiceSprite = 9; Flip = 0; break;           // ASCII: '{'  |  Sprite: Triangulo apuntando a la izquierda (Triangulo derecho "flipeado") 
      case 125: IndiceSprite = 9; break;                     // ASCII: '}'  |  Sprite: Triangulo apuntando a la derecha
      case 39:  IndiceSprite = 10; break;                    // ASCII: '''  |  Sprite: '
      case 44:  IndiceSprite = 11; break;                    // ASCII: ','  |  Sprite: ,
      case 45:  IndiceSprite = 12; break;                    // ASCII: '-'  |  Sprite: -
      case 32:  IndiceSprite = 13; break;                    // ASCII: ' '  |  Sprite: Espacio
      default:                                              // ASCII: Letras
        IndiceSprite = 39 - (Caracteres[i] - 65);
        break;
    }
    // Parámetros(Coordenada X, Coordenada Y, Ancho, Alto, NombreSprite, Total de Sprites, No. de Sprite, Flip, Offset, Color Texto, Color Fondo) 
    LCD_TextCS(CoordX + 8*i, CoordY, 8, 8, Letras, 40, IndiceSprite, Flip, 0, ColorTexto, ColorFondo);
  }
}

//***************************************************************************************************************************************
// Función para dibujar numeros en la LCD
//***************************************************************************************************************************************
void Write_Num(int Numero, byte NoDigitos, int CoordX, int CoordY, int ColorTexto, int ColorFondo){
  
  int Divisor = 10;
  int Digitos[NoDigitos + 1];
  
  Digitos[0] = Numero % Divisor;

  for (int i = 1; i < NoDigitos; i++){
    Digitos[i] = (Numero % (Divisor * 10) - Numero % Divisor) / Divisor; 
    Divisor = Divisor * 10;
  }

  for (int i = 0; i < NoDigitos; i++){
    int IndiceSprite = 9 - Digitos[i];
    LCD_TextCS(CoordX + 8*(NoDigitos - 1) - 8*i, CoordY, 8, 8, Numeros, 10, IndiceSprite, 1, 0, ColorTexto, ColorFondo);
  }

}

//***************************************************************************************************************************************
// Función para dibujar una letra o caracter especial a partir de el sprite "Letras"
//***************************************************************************************************************************************
void LCD_TextCS(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset, int ColorTexto, int ColorFondo){
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 

  unsigned int x2, y2;
  x2 =   x+width;
  y2=    y+height;
  SetWindows(x, y, x2-1, y2-1);
  int k = 0;
  int ancho = ((width*columns));
  
  if(flip){
    for (int j = 0; j < height; j++){
      k = (j*(ancho) + index*width -1 - offset)*2;
      k = k+width*2;
      for (int i = 0; i < width; i++){
        if (bitmap[k] == 0xff && bitmap[k+1] == 0xff){
          LCD_DATA(ColorTexto >> 8);
          LCD_DATA(ColorTexto);
        }
        else if(bitmap[k] == 0x00 && bitmap[k+1] == 0x00){
          LCD_DATA(ColorFondo >> 8);
          LCD_DATA(ColorFondo);
        }
        k = k - 2;
      } 
    }
  }
  
  else{
    for (int j = 0; j < height; j++){
      k = (j*(ancho) + index*width + 1 + offset)*2;
      for (int i = 0; i < width; i++){
        if (bitmap[k] == 0xff && bitmap[k+1] == 0xff){
          LCD_DATA(ColorTexto >> 8);
          LCD_DATA(ColorTexto);
        }
        else if(bitmap[k] == 0x00 && bitmap[k+1] == 0x00){
          LCD_DATA(ColorFondo >> 8);
          LCD_DATA(ColorFondo);
        }
        k = k + 2;
      }
    } 
  }
  
  digitalWrite(LCD_CS, HIGH);
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
// Función para dibujar una imagen a partir de un arreglo de colores (Bitmap) extraido de la memoría SD
//***************************************************************************************************************************************
void LCD_BitmapSD(int x, int y, int width, int height, String TXT){
  
  /*
   * Función para poder extraer algún archivo de la memoria SD y de esa forma ahorrar espacio en la memoria RAM de la TIVA C
   */
  
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 

  int Num_Caracteres = TXT.length();                          // Se obtiene el número de caracteres en el String escrito
  char Nombre_archivo[Num_Caracteres + 1];                      // Se crea un array con el tamaño del número de caracteres
  TXT.toCharArray(Nombre_archivo, Num_Caracteres + 1);          // Se almacena cada caracter individual 
  Archivo = SD.open(Nombre_archivo);  
  
  char NibbleH = 0; 
  char NibbleL = 0;
  byte ConversionH = 0;
  byte ConversionL = 0;
  int DatosSD[2];

  unsigned int x2, y2;
  x2 =   x+width;
  y2=    y+height;
  SetWindows(x, y, x2-1, y2-1);

  if (Archivo){
    for (int j = 0; j < height; j++){
      for (int i = 0; i < width; i++){
        for (int k = 0; k < 2; k++){
          while (Archivo.read() != 'x');
          NibbleH = Archivo.read();
          NibbleL = Archivo.read();
          if (NibbleH > 96) ConversionH = 87; else ConversionH = 48;
          if (NibbleL > 96) ConversionL = 87; else ConversionL = 48;
          DatosSD[k] = (NibbleH - ConversionH)*16 + (NibbleL - ConversionL);
          LCD_DATA(DatosSD[k]);
        }
      }
    } 
  }
  
  Archivo.close();
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

//***************************************************************************************************************************************
// Función para dibujar una imagen sprite - los parámetros columns = número de imagenes en el sprite, index = cual desplegar, flip = darle vuelta, colorswap == si se quiere un cambio de color.
//***************************************************************************************************************************************
void LCD_SpriteCS(int x, int y, int width, int height, unsigned char bitmap[],int columns, int index, char flip, char offset, char colorswap, int OldColor, int NewColor){
  LCD_CMD(0x02c); // write_memory_start
  digitalWrite(LCD_RS, HIGH);
  digitalWrite(LCD_CS, LOW); 

  unsigned int x2, y2;
  x2 =   x+width;
  y2=    y+height;
  SetWindows(x, y, x2-1, y2-1);
  int k = 0;
  int ancho = ((width*columns));
  
  if(flip){
    for (int j = 0; j < height; j++){
      k = (j*(ancho) + index*width -1 - offset)*2;
      k = k+width*2;
      for (int i = 0; i < width; i++){
        if (colorswap == 1 && highByte(OldColor) == bitmap[k] && lowByte(OldColor) == bitmap[k+1]){
          LCD_DATA(NewColor >> 8);
          LCD_DATA(NewColor);
        }
        else{
          LCD_DATA(bitmap[k]);
          LCD_DATA(bitmap[k+1]);
        }
        k = k - 2;
      } 
    }
  }
  
  else{
    for (int j = 0; j < height; j++){
      k = (j*(ancho) + index*width + 1 + offset)*2;
      for (int i = 0; i < width; i++){
        if (colorswap == 1 && highByte(OldColor) == bitmap[k] && lowByte(OldColor) == bitmap[k+1]){
          LCD_DATA(NewColor >> 8);
          LCD_DATA(NewColor);
        }
        else{
          LCD_DATA(bitmap[k]);
          LCD_DATA(bitmap[k+1]);
        }
        k = k + 2;
      }
    } 
  }
  
  digitalWrite(LCD_CS, HIGH);
}
