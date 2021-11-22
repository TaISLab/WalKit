/* ================================================================

   Proyecto: Modulo de encoder en andador para UMA
   Placa de desarrollo: Arduino
   Función: Cuenta las vueltas que ha girado la rueda del andador
            en base a la posición del encoder, tiene un led indicador
            y recibe/envía los datos a través del BUS CAN

   Autor: Justo Barroso Fontalba
   Fecha: 18/03/2019

   ================================================================ */

/* ================================================================
   Librerías
   ================================================================ */

#include <SPI.h>
#include <AS5601.h>
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
#define ID_CHG          14    //PC0, 23
//#define MCP2515_SI      11    //PB3, 15
//#define MCP2515_SO      12    //PB4, 16
//#define MCP2515_SCK     13    //PB5, 17

//Test en Arduino UNO
//#define MCP2515_CS      10
//#define MCP2515_INT     2
//#define LED_GATE        3     //PD3, 5  (Arduino UNO)

//Constantes
#define ID_RIGHT      0x012   //Identificador del modulo derecho
#define ID_LEFT       0x022   //Identificador del modulo izquierdo

/* ================================================================
   Definición de funciones
   ================================================================ */

float readAS5601(); //Lee el valor del sensor y devuelve el valor en grados
//void readAS5601Serie(); //Prueba encoder
void checkTurn(); //Cuenta las vueltas
void sendAS5601(); //Compone mensaje y envía

/* ================================================================
   Definición de estructuras
   ================================================================ */

typedef union
{
  float value;
  uint8_t bytes[4];
} FLOATUNION_t;

typedef union
{
  uint16_t value;
  uint8_t bytes[2];
} UINT16UNION_t;

FLOATUNION_t degree;
UINT16UNION_t turn;

/* ================================================================
   Definición de variables
   ================================================================ */

static long unsigned int rxId; //Identificador del mensaje
static byte len = 0; //Longitud del mensaje
static byte rxBuf[8]; //Array datos de recepción
static byte txBuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //Array datos de envió
static uint16_t pos = 0; //Posición actual
static uint16_t oldPos = 0; //Posición anterior
static byte id_device = ID_LEFT;

/* ================================================================
   Drivers
   ================================================================ */

AS5601 Sensor;

/* ================================================================
   Inicialización
   ================================================================ */

MCP_CAN CAN0( MCP2515_CS ); // Establece el CS en el pin 10

void setup() {
  /*
    // TIMER 1, frecuencia de interrupción 100 Hz:
    cli(); //Interrupciones deshabilitadas
    TCCR1A = 0; // Registro TCCR1A a 0
    TCCR1B = 0; // Registro TCCR1B a 0
    TCNT1  = 0; // Valor del contador a 0
    //Configuración del registro de comparación para 100Hz
    OCR1A = 19999; // = 16000000 / (8 * 1000) - 1 (debe ser <65536)
    //CTC mode habilitado
    TCCR1B |= (1 << WGM12);
    //Configuración del los registros CS22, CS21 y CS20 para prescaler 8
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
    //Configuración del registro para interrupción
    TIMSK1 |= (1 << OCIE1A);
    sei(); //Interrupciones habilitadas
  */

  //Configuración de pines
  pinMode( LED_GATE, OUTPUT );
  
  pinMode( MCP2515_INT, INPUT_PULLUP ); // Interrupción MCP2515
  pinMode( MCP2515_STBY, OUTPUT ); // CS MCP2515
  pinMode( ID_CHG, INPUT_PULLUP ); // Selección izquierda/derecha
  digitalWrite( MCP2515_STBY, LOW ); // Modo normal activo MCP2562

  //Establece la posicion del modulo en el andador
  if ( digitalRead( ID_CHG ) == 0 ) {
    id_device = ID_RIGHT;
  }

  //Inicialización de puerto serie, velocidad 115200 baudios
  Serial.begin( 115200 );

  //Inicialización MCP2515, mascara y filtros activados, velocidad 500kb/s, cristal 16MHz
  if ( CAN0.begin( MCP_STDEXT, CAN_500KBPS, MCP_16MHZ ) == CAN_OK ) {
    Serial.println( "MCP2515 Initialized Successfully!" );
    //digitalWrite( LED_GATE, HIGH ); //Enciende el LED
  }
  else {
    Serial.println( "Error Initializing MCP2515..." );
  }

  //Establece mascaras y filtros para los dispositivos
  CAN0.init_Mask( 0, 0, 0x001F0000 ); // Iniciando primera mascara
  CAN0.init_Filt( 0, 0, 0x00100000 ); // Iniciando primer filtro, ID 0x10 (rPi)

  CAN0.init_Mask( 1, 0, 0x001F0000 ); // Iniciando segunda mascara

  //Configuración del MCP2515
  CAN0.setMode( MCP_NORMAL ); // Modo normal para permitir el envió de mensajes

  //Establecer posición a 0
  Sensor.setZeroPosition();
  Serial.println( F("Posición establecida en 0!") );

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
  //Test de lectura de los parámetros del encoder
  //readAS5601Serie(); //Lee el valor de posición del encoder a través de I2C

  //Lee la posición del sensor constantemente a máxima velocidad y cuenta el numero de vueltas
  degree.value = readAS5601();
  checkTurn() ; //Cuenta las vueltas que ha girado la rueda
  
  //Si alguno de los dos modos esta activo envía por CAN la posición y el numero de vueltas
  //En modo normal solo envía un vez, en modo continuo envía constantemente
  if ( txBuf[1] == 0b0000100 || txBuf[1] == 0b0001000) {   
    sendAS5601();
    if ( txBuf[1] == 0b0000100 ) {
      txBuf[1] = 0b0000000;
    }
  }
}

/* ================================================================
   Interrupciones
   ================================================================ */

//Rutina de interrupción del timer 1
ISR( TIMER1_COMPA_vect ) {
  //Esta configurada para enviar cada 10 ms, pendiente de confirmación
}

void readCAN() {
  CAN0.readMsgBuf( &rxId, &len, rxBuf ); // Leer datos: len = longitud de los datos, buf = bytes de datos
  if ( rxBuf[0] == id_device ) {
    if ( ( rxBuf[1] & 0b00000001 ) == 0b00000001 ) {
      //if ( ( PIND & B00001000 ) == B00000000 ) {
      digitalWrite( LED_GATE, LOW ); //Enciende el LED
      Serial.println( "Led encendido" );
      //}
    }
    if ( ( rxBuf[1] & 0b00000001 ) == 0b00000000 ) {
      //if ( ( PIND & B00001000 ) == B00001000 ) {
      digitalWrite( LED_GATE, HIGH ); //Apaga el LED
      Serial.println( "Led apagado" );
      //}
    }
    if ( ( rxBuf[1] & 0b00001100 ) == 0b00000100 ) {
      txBuf[1] = 0b00000100;
      Serial.println( "Modo Normal" );

    }
    if ( ( rxBuf[1] & 0b00001100 ) == 0b00001000 ) {
      txBuf[1] = 0b0001000;
      Serial.println( "Modo Continuo" );
    }
  }
}

/* ================================================================
   Funciones
   ================================================================ */

float readAS5601() {
  pos = Sensor.getAngle(); //Lee el valor en corregido a 0 del AS5601.
  //Serial.println( pos );
  return ( 360 * ( float ) pos ) / 4095; //Lee el valor del sensor y devuelve la posición en grados.
}

/*
  void readAS5601Serie() {
  Serial.print( F( "Magnitud: " ) );
  Serial.print( Sensor.getMagnitude() );

  Serial.print( F( " | Angulo: " ) );
  Serial.print( Sensor.getRawAngle() );

  Serial.print( F( " | Angulo corregido: " ) );
  Serial.print( Sensor.getAngle() );

  Serial.println();
  }
*/

//Si la posición anterior es 3/4 de vueltas y el nuevo valor es 0,
//aumenta el numero de vuelta en 1 y resta en caso contrario.
void checkTurn() {
  if ( oldPos > 3071 && pos < 1023 ) {
    turn.value ++ ;
    /*Serial.print( F(" Posición en grados: ") );
      Serial.println( degree.value );
      Serial.print( F(" Vueltas: ") );
      Serial.println( turn.value );*/
  }
  else if ( oldPos < 1023 && pos > 3071 ) {
    turn.value -- ;
    /*Serial.print( F(" Posición en grados: ") );
      Serial.println( degree.value );
      Serial.print( F(" Vueltas: ") );
      Serial.println( turn.value );*/
  }
  oldPos = pos;
}

//Construcción de la trama de envió, vuelca los valores en txBuf.
void sendAS5601() {
  memcpy( txBuf + 2, degree.bytes, sizeof( degree.bytes ) );
  memcpy( txBuf + 2 + sizeof( degree.bytes ), turn.bytes, sizeof( turn.bytes ) );

  // Envió de trama: id_device, Estructura CAN estándar, Longitud del mensaje = 8 bytes, 'txBuf' = Array de datos a enviar
  byte sndStat = CAN0.sendMsgBuf( id_device, 0, 8, txBuf );
  if ( sndStat == CAN_OK ) {
    Serial.println( "Mensaje enviado!" );
  }
  else {
    Serial.println( "Error al enviar el mensaje..." );
  }
}

/* ================================================================
   Fin
   ================================================================ */


   
