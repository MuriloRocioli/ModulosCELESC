/* Heltec Automation send communication test example
 *
 * Function:
 * 1. Send data from a esp32 device over hardware 
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#include "Arduino.h"

#include <Wire.h>
#include <Adafruit_MLX90614.h>


#define RF_FREQUENCY                                868000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              1         // [0: 125 kHz,
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

//char txpacket[BUFFER_SIZE];
//char rxpacket[BUFFER_SIZE];

double dataBuffer[3];

Adafruit_MLX90614 mlx_temp = Adafruit_MLX90614();

double dataCount;

bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    mlx_temp.begin();
	
    dataCount=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   }



void loop()
{
	if(lora_idle == true)
	{
    dataCount += 1.0;
    dataBuffer[0] = dataCount;
    dataBuffer[1] = mlx_temp.readAmbientTempC();
    dataBuffer[2] = mlx_temp.readObjectTempC();

    delay(250);
		// sprintf(txpacket,"%0.0f; %0.4f; %0.4f",
    //                           (float) dataBuffer[0], 
    //                           (float) dataBuffer[1],
    //                           (float) dataBuffer[2]);  //start a package
   
		//Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

    Serial.printf("%0.0f; %0.4f; %0.4f;\t length: %d\n",
                              (float) dataBuffer[0], 
                              (float) dataBuffer[1],
                              (float) dataBuffer[2],
                              sizeof(dataBuffer));  //start a package


    Radio.Send( (uint8_t *)dataBuffer, sizeof(dataBuffer) ); //send the package out	

		//Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out	
    lora_idle = false;
	}
  Radio.IrqProcess( );
}

void OnTxDone( void )
{
	//Serial.println("TX done......");
	lora_idle = true;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}


