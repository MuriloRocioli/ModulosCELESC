/* 
*   Projeto CELESC - UFU
*   Codigo referente ao Robo Base Móvel
*
*   Autor: Murilo Rocioli
*/

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"


#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

//Variáveis
int16_t txNumber;
int16_t Rssi,rxSize;
bool lora_idle=true;
int pwmValueX = 0; // Valor recebido do PWM

// Variável para rastrear o estado do eletroímã
bool eletroimaState = true; // Começa como HIGH (ativado)

// Variável para rastrear o estado do motor D
bool motorDState = false; // Começa no estado inicial (LOW, HIGH)

//Motor movimentação
#define MOTOR_A_PIN1 6
#define MOTOR_A_PIN2 7

//Eletroima
#define ELETROIMA 5


bool com_recuar = 0;
bool com_avancar = 0;

//Configuração display
//SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    pinMode(MOTOR_A_PIN1, OUTPUT);
    pinMode(MOTOR_A_PIN2, OUTPUT);

    pinMode(ELETROIMA, OUTPUT);
    
    digitalWrite(MOTOR_A_PIN1, LOW);
    digitalWrite(MOTOR_A_PIN2, LOW);

    digitalWrite(ELETROIMA, HIGH);    
    
    
    txNumber=0;
    Rssi=0;
  
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  /*Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );*/

  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

}



void loop()
{
  if(lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);

    
  }
  Radio.IrqProcess( );

}

void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    
    //Eletroima
    if (strcmp(rxpacket, "A") == 0) {
        eletroimaState = !eletroimaState; // Alterna o estado

        if (eletroimaState) {
            digitalWrite(ELETROIMA, HIGH);
            Serial.println("Eletroímã Ativado");
        } else {
            digitalWrite(ELETROIMA, LOW);
            Serial.println("Eletroímã Desativado");
        }
    }
          
    //Robô não faz nada
    else if (strcmp(rxpacket, "0") == 0) {
        analogWrite(MOTOR_A_PIN1, 0);
        analogWrite(MOTOR_A_PIN2, 0);

        Serial.printf("Aguardando...\n");  
    } 

    else {
      String rxString = String(rxpacket); // Converte rxpacket para String
      pwmValueX = rxString.toInt();       // Usa toInt para converter para inteiro
      
      Serial.print("Valor recebido: ");
      Serial.println(pwmValueX);
      
      //Controlar motor
      controlMotor(pwmValueX);

    } 
    
    
    lora_idle = true;
}

void controlMotor(int pwmValue) {
    Serial.print("Valor recebido no controlMotor: ");
    Serial.println(pwmValue); // Depuração: mostra o valor recebido
    
    if (pwmValue >= 30 && pwmValue <= 255) {
        // Avançar
        Serial.print("Avançando: ");
        Serial.println(pwmValue);

        analogWrite(MOTOR_A_PIN1, pwmValue);
        analogWrite(MOTOR_A_PIN2, 0);
    } else if (pwmValue <= -30 && pwmValue >= -255) {
        // Recuar
        Serial.print("Avançando: ");
        Serial.println(abs(pwmValue));

        analogWrite(MOTOR_A_PIN1, 0);
        analogWrite(MOTOR_A_PIN2, abs(pwmValue));
    } else {
        // Parar o motor se pwmValue estiver dentro da zona neutra
        Serial.println("Parando motor (zona neutra).");
        
        // Parar o motor
        analogWrite(MOTOR_A_PIN1, 0);
        analogWrite(MOTOR_A_PIN2, 0);
    }
}