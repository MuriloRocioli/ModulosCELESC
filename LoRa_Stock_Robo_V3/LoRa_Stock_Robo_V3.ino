/* 
*   Projeto CELESC - UFU
*   Codigo referente ao Robo para remocao de Detritos
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

int tempoMotorB = 6000;//Motor translação amortecedor
int tempoMotorC = 20000;//Motorredutor
int tempoMotorD = 2000;//Motor pinças
int tempoMotorE = 10000;//Motor translação motorredutor

// Variável para rastrear o estado do eletroímã
bool eletroimaState = true; // Começa como HIGH (ativado)

// Variável para rastrear o estado do motor D
bool motorDState = false; // Começa no estado inicial (LOW, HIGH)

//Motor movimentação
#define MOTOR_A_PIN1 12
#define MOTOR_A_PIN2 13

//Motor translação amortecedor
#define MOTOR_B_PIN1 0
#define MOTOR_B_PIN2 22

//Motorredutor
#define MOTOR_C_PIN1 21
#define MOTOR_C_PIN2 32

//Motor pinças
#define MOTOR_D_PIN1 33
#define MOTOR_D_PIN2 25

//Motor translação motorredutor
#define MOTOR_E_PIN1 2
#define MOTOR_E_PIN2 17

//Eletroima
#define ELETROIMA 23


bool com_recuar = 0;
bool com_avancar = 0;

//Configuração display
//SSD1306Wire  factory_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    pinMode(MOTOR_A_PIN1, OUTPUT);
    pinMode(MOTOR_A_PIN2, OUTPUT);

    pinMode(MOTOR_B_PIN1, OUTPUT);
    pinMode(MOTOR_B_PIN2, OUTPUT);

    pinMode(MOTOR_C_PIN1, OUTPUT);
    pinMode(MOTOR_C_PIN2, OUTPUT);

    pinMode(MOTOR_D_PIN1, OUTPUT);
    pinMode(MOTOR_D_PIN2, OUTPUT);

    pinMode(MOTOR_E_PIN1, OUTPUT);
    pinMode(MOTOR_E_PIN2, OUTPUT);

    pinMode(ELETROIMA, OUTPUT);
    
    digitalWrite(MOTOR_A_PIN1, LOW);
    digitalWrite(MOTOR_A_PIN2, LOW);

    digitalWrite(MOTOR_B_PIN1, LOW);
    digitalWrite(MOTOR_B_PIN2, LOW);

    digitalWrite(MOTOR_C_PIN1, LOW);
    digitalWrite(MOTOR_C_PIN2, LOW);

    digitalWrite(MOTOR_D_PIN1, LOW);
    digitalWrite(MOTOR_D_PIN2, LOW);

    digitalWrite(MOTOR_E_PIN1, LOW);
    digitalWrite(MOTOR_E_PIN2, LOW);

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
      
    
    //Instalação do amortecedor
    if (strcmp(rxpacket, "C") == 0) {
        //Avanço Motor translação amortecedor
        digitalWrite(MOTOR_B_PIN1, HIGH);
        digitalWrite(MOTOR_B_PIN2, LOW);

        delay(tempoMotorB);

        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, LOW);
        Serial.println("Motor Translação Amortecedor Avançado");

        delay(1000);

        //Avanço Motor translação motorredutor
        digitalWrite(MOTOR_E_PIN1, HIGH);
        digitalWrite(MOTOR_E_PIN2, LOW);

        delay(tempoMotorE);

        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, LOW);
        Serial.println("Motor Translação Motorredutor Avançado");

        delay(1000);

        //Giro Motorredutor para Instalação
        digitalWrite(MOTOR_C_PIN1, HIGH);
        digitalWrite(MOTOR_C_PIN2, LOW);

        delay(tempoMotorC);

        digitalWrite(MOTOR_C_PIN1, LOW);
        digitalWrite(MOTOR_C_PIN2, LOW);
        Serial.println("Motorredutor girou");

        delay(1000);

        //Recuo Motor translação motorredutor
        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, HIGH);

        delay(tempoMotorE);

        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, LOW);
        Serial.println("Motor Translação Motorredutor Recuado");

        //Avanço Motor pinças para liberar amortecedor
        digitalWrite(MOTOR_D_PIN1, LOW);
        digitalWrite(MOTOR_D_PIN2, HIGH);

        delay(tempoMotorD);

        digitalWrite(MOTOR_D_PIN1, LOW);
        digitalWrite(MOTOR_D_PIN2, LOW);
        Serial.println("Amortecedor liberado");
        
        delay(1000);

        //Recuo Motor translação amortecedor
        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, HIGH);

        delay(tempoMotorB);

        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, LOW);
        Serial.println("Motor Translação Amortecedor Recuado");

        delay(1000);

        Serial.printf("Amortecedor Instalado \n");
    } 

    //Avancar
    else if (strcmp(rxpacket, "B") == 0) {
       motorDState = !motorDState; // Alterna o estado

        if (motorDState) {
            // Apertar pinças
            digitalWrite(MOTOR_D_PIN1, HIGH);
            digitalWrite(MOTOR_D_PIN2, LOW);

            delay(tempoMotorD);

            digitalWrite(MOTOR_D_PIN1, LOW);
            digitalWrite(MOTOR_D_PIN2, LOW);

            delay(1000);

            //Recua 0.5 s motor translação amortecedor
            digitalWrite(MOTOR_B_PIN1, LOW);
            digitalWrite(MOTOR_B_PIN2, HIGH);

            delay(500);

            digitalWrite(MOTOR_B_PIN1, LOW);
            digitalWrite(MOTOR_B_PIN2, LOW);

            Serial.println("Apertar pinças");
        } else {
            // Soltar pinças
            digitalWrite(MOTOR_D_PIN1, LOW);
            digitalWrite(MOTOR_D_PIN2, HIGH);

            delay(tempoMotorD);

            digitalWrite(MOTOR_D_PIN1, LOW);
            digitalWrite(MOTOR_D_PIN2, LOW);

            Serial.println("Soltar pinças");
        }
    }
    
    //Desinstalação do amortecedor
    else if (strcmp(rxpacket, "D") == 0) {
        //Avanço Motor translação amortecedor
        digitalWrite(MOTOR_B_PIN1, HIGH);
        digitalWrite(MOTOR_B_PIN2, LOW);

        delay(tempoMotorB);

        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, LOW);
        Serial.println("Motor Translação Amortecedor Avançado");

        delay(1000);

        //Recuo Motor pinças para prender amortecedor
        digitalWrite(MOTOR_D_PIN1, HIGH);
        digitalWrite(MOTOR_D_PIN2, LOW);

        delay(tempoMotorD);

        digitalWrite(MOTOR_D_PIN1, LOW);
        digitalWrite(MOTOR_D_PIN2, LOW);

        //Avanço Motor translação motorredutor
        digitalWrite(MOTOR_E_PIN1, HIGH);
        digitalWrite(MOTOR_E_PIN2, LOW);

        delay(tempoMotorE);

        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, LOW);
        Serial.println("Motor Translação Motorredutor Avançado");

        delay(1000);

        //Giro Motorredutor para Desinstalação
        digitalWrite(MOTOR_C_PIN1, LOW);
        digitalWrite(MOTOR_C_PIN2, HIGH);

        delay(tempoMotorC);

        digitalWrite(MOTOR_C_PIN1, LOW);
        digitalWrite(MOTOR_C_PIN2, LOW);

        delay(1000);

        //Recuo

        //Recuo Motor translação motorredutor
        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, HIGH);

        delay(tempoMotorE);

        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, LOW);
        Serial.println("Motor Translação Motorredutor Recuado");

        delay(1000);

        //Recuo Motor translação amortecedor
        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, HIGH);

        delay(tempoMotorB);

        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, LOW);
        Serial.println("Motor Translação Amortecedor Recuado");

        delay(1000);

        Serial.printf("Amortecedor Desinstalado \n");  
    } 

    else if (strcmp(rxpacket, "A2") == 0) {
      //Recuo Motor translação amortecedor
        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, HIGH);

        delay(1000);

        digitalWrite(MOTOR_B_PIN1, LOW);
        digitalWrite(MOTOR_B_PIN2, LOW);
        Serial.println("Motor Translação Amortecedor Recuado");

    }

    else if (strcmp(rxpacket, "B2") == 0) {
      //Recuo Motor translação motorredutor
        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, HIGH);

        delay(1000);

        digitalWrite(MOTOR_E_PIN1, LOW);
        digitalWrite(MOTOR_E_PIN2, LOW);
        Serial.println("Motor Translação Motorredutor Recuado");      

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

        analogWrite(MOTOR_A_PIN1, pwmValue*0.8);
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