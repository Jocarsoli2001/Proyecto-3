
//-------------------------------- Definición de pines --------------------------------

// Jugador 1
#define IZ_J1 15
#define DE_J1 4
#define IMP_J1 5

// Jugador 2
#define IZ_J2 18
#define DE_J2 19
#define IMP_J2 21

#define Buzzer_pin 22
#define ledChannel 0
#define resolucion 8

//------------------------------------- Variables -------------------------------------
int I_J1 = 0;
int D_J1 = 0;
int Im_J1 = 0;
int I_J2 = 0;
int D_J2 = 0;
int Im_J2 = 0;
int izquierda_J1 = 0;
int derecha_J1 = 0;
int impulso_J1 = 0;
int izquierda_J2 = 0;
int derecha_J2 = 0;
int impulso_J2 = 0;
int dato_enviado = 0b000000;

//------------------------------ Prototipo de funciones -------------------------------
void Retorno_botones(int boton);


//*************************************************************************************
// Setup
//*************************************************************************************
void setup() {
  Serial2.begin(115200);                                            // Inicialización del puerto serial 2

  pinMode(IZ_J1, INPUT);                                            // Pin 15 como input
  pinMode(DE_J1, INPUT);                                            // Pin 4 como input
  pinMode(IMP_J1, INPUT);                                           // Pin 5 como input
  pinMode(IZ_J2, INPUT);                                            // Pin 18 como input
  pinMode(DE_J2, INPUT);                                            // Pin 19 como input
  pinMode(IMP_J2, INPUT);                                           // Pin 21 como input
}


//*************************************************************************************
// Main Loop
//*************************************************************************************
void loop() {
  // ------------------------- Lectura de pines digitales -----------------------------
  I_J1 = digitalRead(IZ_J1);
  D_J1 = digitalRead(DE_J1);
  Im_J1 = digitalRead(IMP_J1);
  I_J2 = digitalRead(IZ_J2);
  D_J2 = digitalRead(DE_J2);
  Im_J2 = digitalRead(IMP_J2);
  
  // ---------------------------------- Botones ---------------------------------------

  // Botón de la izquierda en jugador 1 
  if(I_J1 == HIGH){
    dato_enviado = dato_enviado | 0b100000;
  }
  else if(I_J1 == LOW && (dato_enviado & 0b100000) == 0b100000){
    dato_enviado = dato_enviado ^ 0b100000;
  }

  // Botón de la derecha en jugador 1 
  if(D_J1 == HIGH){
    dato_enviado = dato_enviado | 0b010000;
  }
  else if(D_J1 == LOW && (dato_enviado & 0b010000) == 0b010000){
    dato_enviado = dato_enviado ^ 0b010000;
  }

  // Botón de impulso en jugador 1 
  if(Im_J1 == HIGH){
    dato_enviado = dato_enviado | 0b001000;
  }
  else if(Im_J1 == LOW && (dato_enviado & 0b001000) == 0b001000){ 
    dato_enviado = dato_enviado ^ 0b001000;
  }

  // Botón de la izquierda en jugador 2 
  if(I_J2 == HIGH){
    dato_enviado = dato_enviado | 0b000100;
  }
  else if(I_J2 == LOW && (dato_enviado & 0b000100) == 0b000100){
    dato_enviado = dato_enviado ^ 0b000100;
  }

  // Botón de la derecha en jugador 2 
  if(D_J2 == HIGH){
    dato_enviado = dato_enviado | 0b000010;
  }
  else if(D_J2 == LOW && (dato_enviado & 0b000010) == 0b000010){
    dato_enviado = dato_enviado ^ 0b000010;
  }

  // Botón de impulso en jugador 2 
  if(Im_J2 == HIGH){
    dato_enviado = dato_enviado | 0b000001;
  }
  else if(D_J2 == LOW && (dato_enviado & 0b000001) == 0b000001){
    dato_enviado = dato_enviado ^ 0b000001;
  }

  Serial2.write(dato_enviado);
}
