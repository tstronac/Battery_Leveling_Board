// -------------------------------------------------------------
// CANtest for Teensy 3.1
// by teachop
//
// This test is talking to a single other echo-node on the bus.
// 6 frames are transmitted and rx frames are counted.
// Tx and rx are done in a way to force some driver buffering.
// Serial is used to print the ongoing status.
//
#include <Arduino.h>
#include <Metro.h>
#include <FlexCAN.h>
#include "Flex_BATT_HEADER.h"

Metro sysTimer = Metro(1);// milliseconds

int led = 13;

FlexCAN CANbus(500000);

static CAN_message_t tx_msg, rx_msg;

static uint8_t hex[17] = "0123456789abcdef";

int txCount,rxCount;
unsigned int txTimer,rxTimer;

static int ee_batt_state; //Pin 10
static int motor_batt_state; //Pin 6
static float voltages[NUM_OF_BATTERIES];
static uint8_t batt_volt_message [8] = {0x8, 0x8, 0,0,0,0,0,0 };






/* swaps batt src and updates src field */
static void
batt_swap(void)
{
	switch (ee_batt_state) {
	case 0:
		digitalWrite(MOTOR_BATT_CONT, ~motor_batt_state); //bitwise Compliment
		digitalWrite(EE_BATT_CONT, ~ee_batt_state);
		break;
	case 1:
		digitalWrite(EE_BATT_CONT, ~ee_batt_state);
		digitalWrite(MOTOR_BATT_CONT, ~motor_batt_state);
	}

	ee_batt_state = ~ee_batt_state; //changes the state to the current one
	motor_batt_state = ~motor_batt_state; 
}

/* returns 1 if v_1 src and v_2 src should be swapped */
static int
voltcmp(float v_1, float v_2)
{
	return abs(v_1 - v_2) > V_DELTA;
}


/* store voltage level in voltage array */
static void
store_voltage_info(uint8_t i, float f)
{
	voltages[i] = f;
}




//*****************CAN TRANMISSION**************************

//send and recieve data, alot of this needs to go

void can_tx(void)
{
  // service software timers based on Metro tick
  if ( sysTimer.check() ) {
    if ( txTimer ) {
      --txTimer;
    }
  }

  // if not time-delayed, read CAN messages and print 1st byte

  
  // insert a time delay between transmissions
  if ( !txTimer ) {
    // if frames were received, print the count
    if ( rxCount ) {
      Serial.write('=');
      Serial.print(rxCount);
      rxCount = 0;
    }

    //This will need to be changed. 
    //Need a proper ID
    //Proper Message structure 
    txTimer = 100;//milliseconds
    tx_msg.len = 8;
    tx_msg.id = 0x222;
    

//Send Voltage Levels
    for(int idx=0; idx<4; ++idx ) {
      tx_msg.buf[idx] = (uint8_t)voltages[idx];
    }

//SEND Battery ID's
    // for( int idx=4; idx<8; idx++ ) {
    //   tx_msg.buf[idx] = idx-4;
    // }

    
    digitalWrite(led, 1);
    Serial.println(".");
    
    CANbus.write(tx_msg);

    Serial.println("Sent");  
    digitalWrite(led, 0);
    // time delay to force some rx data queue use
    rxTimer = 3;//milliseconds
  }
}
// -------------------------------------------------------------



// *******************CAN recieving***************************** 

// CANbus.read will continue reading until the ring buffer is empty


void can_rx(void){


  // service software timers based on Metro tick
if ( sysTimer.check() ) {
    if ( rxTimer ) {
      --rxTimer;
    }
  }


  if ( !rxTimer ) {
    while ( CANbus.read(rx_msg) ) {
      
      hexDump( sizeof(rx_msg), (uint8_t *)&rx_msg );//does a hex dump of the message
      
      Serial.write(rx_msg.buf[0]);// for debugging (might leave forever)
      
     for( int idx=0; idx<8; ++idx ) {
     store_voltage_info(idx, rx_msg.buf[idx]); //stores voltage in voltage array
    }

      rxCount++; //Counts number of messages 
    }
  }

}

// -------------------------------------------------------------



//********Dumps data and prints it. good for debuging*********

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}

// -------------------------------------------------------------







// *****************Setup Start***********************
void setup(void)
{

  Serial.begin(9600);
  Serial.println("Setup start");

  
  pinMode(led, OUTPUT);
  digitalWrite(led, 1);
  
    CANbus.begin();

  delay(1000);
  
    ee_batt_state = digitalRead(EE_BATT_CONT);
	motor_batt_state = digitalRead(MOTOR_BATT_CONT);

	/* make sure the pins start out in opposite states */
	if (ee_batt_state == motor_batt_state) {
		ee_batt_state = ~motor_batt_state;
		digitalWrite(EE_BATT_CONT, ee_batt_state);
	}

	/* make sure ee batt is starting voltage src */
	if (!ee_batt_state) {
		Serial.println("Swapping to EE battery");
		batt_swap();
	}

	Serial.println("Setup Complete");

  sysTimer.reset();
}

// -------------------------------------------------------------


void loop(void)
{
	int i;
    can_rx();

	delay(500);// is this need? If we have a timer anyways, what is this for?

	if (voltages[0] > EE_MIN_VOLTAGE)
		return;

	for (i = 1; i < NUM_OF_BATTERIES; i++) {
		if (voltcmp(voltages[0], voltages[i]))
			batt_swap();
            can_tx();
    }
}


