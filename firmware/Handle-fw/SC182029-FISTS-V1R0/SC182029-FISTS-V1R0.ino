/* ================================================================

   Proyecto: Modulo de empuñadura en andador para UMA
   Placa de desarrollo: Arduino
   Función: Mide el peso ejercido sobre la empuñadura del andador,
            establece un color, una vibración y recibe/envía los 
			datos a través del BUS CAN

   Autor: Justo Barroso Fontalba
   Fecha: 14/03/2019

   ================================================================ */

/* ================================================================
  Librerías
   ================================================================ */

#include <SPI.h>
#include <HX711.h>
#include <mcp_can.h>

/* ================================================================
   Definición de pines
   ================================================================ */

//ATMega328 TQFP
//                    Pin IDE //Pin puerto, Pin Físico
#define RGB_RED_GATE    10    //PB2, 14
#define RGB_GREEN_GATE  6     //PD6, 10
#define RGB_BLUE_GATE   5     //PD5, 9
#define MOTOR_GATE      9     //PB1, 13
#define HX711_DATA      17    //PC3, 26
#define HX711_SCK       16    //PC2, 25
#define MCP2515_STBY    8     //PB0, 12
#define MCP2515_CS      4     //PD4, 2
#define MCP2515_INT     3     //PD3, 1
#define ID_CHG          14    //PC0, 23
//#define MCP2515_SI      11    //PB3, 15
//#define MCP2515_SO      12    //PB4, 16
//#define MCP2515_SCK     13    //PB5, 17

//Arduino UNO
//#define HX711_DATA      3
//#define HX711_SCK       4
//#define MCP2515_CS      10
//#define MCP2515_INT     2
//#define MOTOR_GATE      1     //PD4, 2

//Constantes
//#define MOTOR_PWM_50    77
//#define MOTOR_PWM_75    115
//#define MOTOR_PWM_100   153
#define ID_RIGHT      0x011   //Identificador del modulo derecho
#define ID_LEFT       0x021   //Identificador del modulo izquierdo

/* ================================================================
   Definición de funciones
   ================================================================ */

void setColor( uint8_t red, uint8_t green, uint8_t blue ); //Establece el color RBG del LED
void setMotor( uint8_t motor ); //Establece la tensión media de funcionamiento en el Motor
float readHX711(); //Lee el valor del sensor
//void readHX711Serie(); //Prueba del sensor
void sendHX711(); //Compone mensaje y envía

/* ================================================================
   Definición de estructuras
   ================================================================ */

typedef union
{
  float value;
  uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t weight;

/* ================================================================
   Definición de variables
   ================================================================ */

static long unsigned int rxId; //Identificador del mensaje
static byte len = 0; //Longitud del mensaje
static byte rxBuf[8]; //Array datos de recepción
static byte txBuf[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //Array datos de envió
static byte id_device = ID_LEFT;

/* ================================================================
   Drivers
   ================================================================ */

HX711 loadcell;

/* ================================================================
   Inicialización
   ================================================================ */

MCP_CAN CAN0( MCP2515_CS ); // Establece el CS en el pin 10

void setup() {
  /*
    //TIMER 1, frecuencia de interrupción 50 Hz:
    cli(); //Interrupciones deshabilitadas
    TCCR1A = 0; // Registro TCCR1A a 0
    TCCR1B = 0; // Registro TCCR1B a 0
    TCNT1  = 0; // Valor del contador a 0
    //Configuración del registro de comparación para 50Hz
    OCR1A = 39999; // = 16000000 / ( 8 * 50 ) - 1 ( debe ser <65536 )
    //CTC mode habilitado
    TCCR1B |= ( 1 << WGM12 );
    //Configuración del los registros CS12, CS11 y CS10 para prescaler 8
    TCCR1B |= ( 0 << CS12 ) | ( 1 << CS11 ) | ( 0 << CS10 );
    //Configuración del registro para interrupción
    TIMSK1 |= ( 1 << OCIE1A );
    sei(); //Interrupciones habilitadas
  */

  //Configuración de pines
  pinMode( RGB_RED_GATE, OUTPUT );
  pinMode( RGB_GREEN_GATE, OUTPUT );
  pinMode( RGB_BLUE_GATE, OUTPUT );
  pinMode( MOTOR_GATE, OUTPUT );
  pinMode( MCP2515_INT, INPUT_PULLUP ); // Interrupción MCP2515
  pinMode( MCP2515_STBY, OUTPUT ); // CS MCP2515
  pinMode( ID_CHG, INPUT_PULLUP ); // Selección izquierda/derecha
  digitalWrite( MCP2515_STBY, LOW ); // Modo normal activo MCP2562

  //Establece la posición del modulo en el andador
  if ( digitalRead( ID_CHG ) == 0 ) {
    id_device = ID_RIGHT;
  }

  //Inicialización de puerto serie, velocidad 115200 baudios
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
  CAN0.setMode( MCP_NORMAL ); // Modo normal para permitir el envió de mensajes

  //Configuración del sensor HX711
  //Variables de configuración del HX711
  //const uint32_t LOADCELL_OFFSET = 50682624;
  //const uint32_t LOADCELL_DIVIDER = 5895655;

  //Inicialización de HX711 con el pin de datos como entrada, el pin de reloj como salida y el factor de ganancia (64 o 128 Canal A y 32 Canal B)
  loadcell.begin( HX711_DATA, HX711_SCK );
  loadcell.set_scale( 2280.f ); // Este valor es obtenido de calibrar la escala con diferentes pesos
  //loadcell.set_offset( LOADCELL_OFFSET ); //Factor de corrección
  loadcell.tare(); //Establece la escala en 0

  //Configurando la interrupción por pin
  attachInterrupt( digitalPinToInterrupt( MCP2515_INT ), readCAN, LOW );

  for (int i = 0; i < 3; i++)
  {
    digitalWrite( RGB_GREEN_GATE, HIGH );
    delay(200);
    digitalWrite( RGB_GREEN_GATE, LOW );
    delay(200);
  }
}

/* ================================================================
   Bucle principal
   ================================================================ */

void loop() {
  //Test de escala de color
  //setColor( 255, 255, 255 ); //Rojo, Verde, Azul. Establece un color

  //Test de velocidad de giro
  //setMotor( 153 ); //La tensión máxima del motor DC es 3V, entonces 255*(3V/5V) = 153, superar el valor hace que el motor trabaje por encima de su tensión máxima

  //Test lectura de célula de carga
  //readHX711Serie(); //Lee el canal analógico y envía por el puerto serie el valor pesado
  if ( txBuf[1] == 0b0000100 || txBuf[1] == 0b0001000) {
    weight.value = readHX711(); // Llamada a función para obtención del valor del peso
    sendHX711();
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
  //Esta configurada para enviar cada 20 ms, pendiente de confirmación
}

void readCAN() {
  CAN0.readMsgBuf( &rxId, &len, rxBuf ); // Leer datos: len = longitud de los datos, buf = bytes de datos
  if ( rxBuf[0] == id_device ) {
    if ( ( rxBuf[1] & 0b00000001 ) == 0b00000001 ) {
      setColor( rxBuf[2], rxBuf[3], rxBuf[4]);
      Serial.print( "LED: " );
      for ( int i = 0; i < 3; i++) {
        Serial.print( rxBuf[i + 2] );
        Serial.print( " " );
      }
      Serial.println();
    }
    if ( ( rxBuf[1] & 0b00000010 ) == 0b00000010 ) {
      setMotor( ( rxBuf[5] * 0x64 ) / 0xAE );
      Serial.print( "Motor: " );
      Serial.println( ( rxBuf[5] * 0x64 ) / 0xAE );
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

void setColor( uint8_t red, uint8_t green, uint8_t blue ) {
  analogWrite( RGB_RED_GATE, red ); //Pin PWM y valor entre 0 y 255 para establecer el color en escala RGB.
  analogWrite( RGB_GREEN_GATE, green );
  analogWrite( RGB_BLUE_GATE, blue );
}

void setMotor( uint8_t motor ) {
  analogWrite( MOTOR_GATE, motor ); //Pin PWM y valor entre 0 y 255, a través de la tensión media de salida se establece la velocidad de giro.
}

float readHX711() {
  static float loadcellValue;
  loadcell.power_up(); // Célula activa
  loadcellValue = loadcell.get_units();
  loadcell.power_down(); // Célula en reposo
  return loadcellValue;
}

/*
  void readHX711Serie() {
  loadcell.power_up();
  Serial.print( "one reading:\t" );
  Serial.print( loadcell.get_units(), 1 );
  Serial.print( "\t| average:\t" );
  Serial.println( loadcell.get_units(10), 1 );
  loadcell.power_down();
  }
*/

//Construcción de la trama de envió, vuelca los valores en txBuf.
void sendHX711() {
  // Copia un array en otro: array1 + posición de inicio, array2 + posición de inicio, tamaño de los datos a copiar
  memcpy( txBuf + 2, weight.bytes, sizeof( weight.bytes) );

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
