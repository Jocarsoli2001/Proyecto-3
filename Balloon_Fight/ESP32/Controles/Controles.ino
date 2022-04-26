//-------------------------------- Definición de pines --------------------------------

// Jugador 1
#define IZ_J1 15
#define DE_J1 4
#define IMP_J1 5

// Jugador 2
#define IZ_J2 18
#define DE_J2 19
#define IMP_J2 21

//------------------------------------- Variables -------------------------------------
int I_J1 = 0;
int D_J1 = 0;
int Im_J1 = 0;
int I_J2 = 0;
int D_J2 = 0;
int Im_J2 = 0;

//------------------------------ Prototipo de funciones -------------------------------
void Retorno_botones(int boton);


//*************************************************************************************
// Setup
//*************************************************************************************
void setup() {
  Serial.begin(115200);
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
  Serial.write("|");                                                // Escribir "|" como indicador que a partir de ese momento comenzarán a llegar 6 datos consecutivos

  // ------------------------- Lectura de pines digitales -----------------------------
  I_J1 = digitalRead(IZ_J1);
  D_J1 = digitalRead(DE_J1);
  Im_J1 = digitalRead(IMP_J1);
  I_J2 = digitalRead(IZ_J2);
  D_J2 = digitalRead(DE_J2);
  Im_J2 = digitalRead(IMP_J2);
  
  // ---------------------------------- Botones ---------------------------------------
  Serial.println("|");
  Serial.print(" Botón izquierdo J1: ");
  Retorno_botones(I_J1);
  Serial.print(" Botón derecho J1: ");
  Retorno_botones(D_J1);
  Serial.print(" Botón impulso J1: ");
  Retorno_botones(Im_J1);
  Serial.print(" Botón izquierdo J2: ");
  Retorno_botones(I_J2);
  Serial.print(" Botón derecho J2: ");
  Retorno_botones(D_J2);
  Serial.print(" Botón impulso J2: ");
  Retorno_botones(Im_J2);

}

void Retorno_botones(int boton){
  switch(boton){
    case LOW:
      Serial.println(0);
      break;
    case HIGH:
      Serial.println(1);
      break;
    default:
      Serial.println(0);
      break;
  }
}
