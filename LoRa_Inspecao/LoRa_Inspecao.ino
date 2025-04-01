#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>  
#include "HT_SSD1306Wire.h"
#include <ESP32Servo.h>
#include <Adafruit_MLX90614.h>

#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 5        // dBm
#define LORA_BANDWIDTH 0        
#define LORA_SPREADING_FACTOR 7  
#define LORA_CODINGRATE 1        
#define LORA_PREAMBLE_LENGTH 8   
#define LORA_SYMBOL_TIMEOUT 0    
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 

double dataBuffer[3];
double dataCount;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ); 

int16_t txNumber;
int16_t Rssi, rxSize;
bool lora_idle = true;
int pwmValueX = 0;

bool eletroimaState = true; 

#define ELETROIMA 23

Servo servoLeft;
Servo servoRight;

#define SERVO_LEFT_PIN 6  
#define SERVO_RIGHT_PIN 20 

Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //Sensor de temperatura

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    mlx.begin();

    pinMode(ELETROIMA, OUTPUT);
    digitalWrite(ELETROIMA, HIGH);

    servoLeft.attach(SERVO_LEFT_PIN);
    servoRight.attach(SERVO_RIGHT_PIN);

    servoLeft.write(60);  
    servoRight.write(60);  

    txNumber = 0;
    Rssi = 0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

     Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
}

void loop() {
    if (lora_idle) {
        lora_idle = false;
        Serial.println("into RX mode");
        Radio.Rx(0);
    }

    /*Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
    Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
    Serial.println();
    delay(500);

    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //Envia o pacote de dados*/

    Radio.IrqProcess();
}

void OnTxDone(void) {
    Serial.println("TX done......");
    lora_idle = true;
}

void OnTxTimeout(void) {
    Radio.Sleep();
    Serial.println("TX Timeout......");
    lora_idle = true;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    rssi = rssi;
    rxSize = size;
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Radio.Sleep();
    Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n", rxpacket, rssi, rxSize);

    if (strcmp(rxpacket, "A") == 0) {
        eletroimaState = !eletroimaState;
        digitalWrite(ELETROIMA, eletroimaState ? HIGH : LOW);
        Serial.println(eletroimaState ? "Eletroímã Ativado" : "Eletroímã Desativado");

    } else if (strcmp(rxpacket, "0") == 0) {
        stopMotors();
        Serial.println("Aguardando...");

    } else {
        pwmValueX = String(rxpacket).toInt();
        Serial.print("Valor recebido: ");
        Serial.println(pwmValueX);
        controlMotors(pwmValueX);
    }
    lora_idle = true;
}

void controlMotors(int pwmValue) {
    if (pwmValue > 30) {  
        servoLeft.write(0);
        servoRight.write(120);
        Serial.println("Avançando");
        

    } else if (pwmValue < -30) {  
        servoLeft.write(120);
        servoRight.write(0);
        Serial.println("Recuando");

    } else {  
        stopMotors();
    }
}

void stopMotors() {
    servoLeft.write(60);  
    servoRight.write(60); 
    Serial.println("Motores Parados.");

    /*dataCount += 1.0;
    dataBuffer[0] = dataCount;
    dataBuffer[1] = mlx.readAmbientTempC();
    dataBuffer[2] = mlx.readObjectTempC();

    delay(250);

    Serial.printf("%0.0f; %0.4f; %0.4f;\t length: %d\n",
                              (float) dataBuffer[0], 
                              (float) dataBuffer[1],
                              (float) dataBuffer[2],
                              sizeof(dataBuffer));  //start a package


    Radio.Send( (uint8_t *)dataBuffer, sizeof(dataBuffer) );*/

    int AmbientTempC = mlx.readAmbientTempC();
    Serial.print("Ambient = "); Serial.print(AmbientTempC);
    Serial.print("*C\t");
    //Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
    Serial.println();

    sprintf(txpacket, "%d",AmbientTempC);
    Serial.printf("\r\nSending packet \"%s\" , length %d\r\n", txpacket, strlen(txpacket));
    Radio.Send((uint8_t *)txpacket, strlen(txpacket)); // Send the package out
    delay(500);
}
