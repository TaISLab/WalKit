/* ================================================================

   Proyecto: Modulo de encoder en andador para UMA
   Placa de desarrollo: Arduino
   Función: Gira entre 0 y 90 grados la posición del servo empujando
			el cable de freno, tiene un led indicador y envía los datos a
			través del BUS CAN.
			
   Autor: Justo Barroso Fontalba
   Fecha: 19/03/2019

   ================================================================ */

/* ================================================================
   Librerías
   ================================================================ */

#include <SPI.h>
#include <Servo.h>
#include <mcp_can.h>

/* ================================================================
   Definición de pines
   ================================================================ */

//ATMega328 TQFP
//                    Pin IDE //Pin puerto, Pin Físico
#define LED_GATE        7     //PD7, 11
#define MCP2515_STBY    8     //PB0, 12
#define MCP2515_CS      4     //PD4, 2
#define MCP2515_INT     3     //PD3, 1
#define SERVO           16    //PC2, 25
#define SENSE           17    //PC3, 26
#define ID_CHG          14    //PC0, 23
//#define MCP2515_SI      11    //PB3, 15
//#define MCP2515_SO      12    //PB4, 16
//#define MCP2515_SCK     13    //PB5, 17

//Test en Arduino UNO
//#define MCP2515_CS      10
//#define MCP2515_INT     2
//#define SERVO           3       //PD3, 5  (Arduino UNO)

//Constantes
#define ID_RIGHT      0x013   //Identificador del modulo derecho
#define ID_LEFT       0x023   //Identificador del modulo izquierdo
#define MAX_CURRENT   715     //Valor máximo de corriente en el servo 1.75A

/* ================================================================
   Definición de funciones
   ================================================================ */

/* ================================================================
   Definición de estructuras
   ================================================================ */

/* ================================================================
   Definición de variables
   ================================================================ */

static long unsigned int rxId; //Identificador del mensaje
static byte len = 0; //Longitud del mensaje
static byte rxBuf[8]; //Array datos de recepción
static byte txBuf[8] = { 0x00, 0x00, 0x00, 0xDD, 0x00, 0xFF, 0x00, 0x00 }; //Array datos de envió
static byte id_device = ID_LEFT;

/* ================================================================
   Drivers
   ================================================================ */

Servo servoMotor;

/* ================================================================
   Inicialización
   ================================================================ */

MCP_CAN CAN0( MCP2515_CS ); // Establece el CS en el pin 10

void setup() {
  //Configuración de pines
  pinMode( LED_GATE, OUTPUT );
  pinMode( SENSE, INPUT );
  pinMode( MCP2515_INT, INPUT_PULLUP ); // Interrupción MCP2515
  pinMode( MCP2515_STBY, OUTPUT ); // CS MCP2515  
  pinMode( ID_CHG, INPUT_PULLUP ); // Selección izquierda/derecha
  digitalWrite( MCP2515_STBY, LOW ); // Modo normal activo MCP2562

  //Establece la posición del modulo en el andador
  if ( digitalRead( ID_CHG ) == 0 ) {
    id_device = ID_RIGHT;
  }

  //Inicialización de puerto serie
  Serial.begin( 115200 );

  //Inicialización MCP2515, mascara y filtros activados, velocidad 500kb/s, cristal 16MHz
  if ( CAN0.begin( MCP_STDEXT, CAN_500KBPS, MCP_16MHZ ) == CAN_OK ) {
    Serial.println( "MCP2515 Initialized Successfully!" );
  }
  else {
    Serial.println( "Error Initializing MCP2515..." );
  }

  //Establece mascaras y filtros para los dispositivos
  CAN0.init_Mask( 0, 0, 0x001F0000 ); // Iniciando primera mascara
  CAN0.init_Filt( 0, 0, 0x00100000 ); // Iniciando primer filtro, ID 0x10 (rPi)

  CAN0.init_Mask( 1, 0, 0x001F0000 ); // Iniciando segunda mascara

  //Configuración del MCP2515
  CAN0.setMode( MCP_NORMAL ); // Modo normal para permitir el envio de mensajes

  //Configuración de pines
  servoMotor.attach( SERVO );

  //Establecer el angulo a 0
  servoMotor.write( 150 );

  //Configurando la interrupción por pin
  attachInterrupt( digitalPinToInterrupt( MCP2515_INT ), readCAN, LOW );

  for (int i = 0; i < 3; i++)
  {
    digitalWrite( LED_GATE, LOW );
    delay(200);
    digitalWrite( LED_GATE, HIGH );
    delay(200);
  }
}

/* ================================================================
   Bucle principal
   ================================================================ */

void loop() {
  
}

/* ================================================================
   Interrupciones
   ================================================================ */

void readCAN() {
  CAN0.readMsgBuf( &rxId, &len, rxBuf ); // Leer datos: len = longitud de los datos, buf = bytes de datos
  if ( rxBuf[0] == id_device ) {
    if ( ( rxBuf[1] & 0b00000001 ) == 0b00000001 ) {
      //if ( ( PIND & B10000000 ) == B00000000 ) {
        digitalWrite( LED_GATE, LOW ); //Enciende el LED
        Serial.println( "Led encendido" );
      //}
    }
    if ( ( rxBuf[1] & 0b00000001 ) == 0b00000000 ) {
      //if ( ( PIND & B10000000 ) == B00000100 ) {
        digitalWrite( LED_GATE, HIGH ); //Apaga el LED
        Serial.println( "Led apagado" );
      //}
    }
    //Rango operativo del servo entre 70 y 150 o 0x46 (90 grados) y 0x96 (0 grados)
    if ( ( rxBuf[1] & 0b00000010 ) == 0b00000010 ) {
      if ( rxBuf[2] <= 0x96 && rxBuf[2] >= 0x46 ) {
        servoMotor.write( rxBuf[2] ); //valor al servo
        //Serial.println( analogRead( SENSE ) );
        while ( analogRead( SENSE ) >= MAX_CURRENT ) {
          servoMotor.write( rxBuf[2] -= 0x05 );
          Serial.println( "Se ha reducido la fuerza ejercida sobre el freno." );
        }        
      }
      Serial.println( "Valor establecido: " );
      Serial.println( rxBuf[2] );
    }
  }
}

/* ================================================================
   Funciones
   ================================================================ */

/* ================================================================
   Fin
   ================================================================ */
