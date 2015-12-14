/*
 * Nav-v1.0 Firmware
 *
 * Created: 12/10/2015 9:00:32 PM
 * Author : Sahil Patel
 */ 

#include "main.h"

//#include "LUFA/GenericHID.h"

///** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
//static uint8_t PrevHIDReportBuffer[GENERIC_REPORT_SIZE];
//
///** LUFA HID Class driver interface configuration and state information. This structure is
 //*  passed to all HID Class driver functions, so that multiple instances of the same class
 //*  within a device can be differentiated from one another.
 //*/
//USB_ClassInfo_HID_Device_t Generic_HID_Interface =
	//{
		//.Config =
			//{
				//.InterfaceNumber              = INTERFACE_ID_GenericHID,
				//.ReportINEndpoint             =
					//{
						//.Address              = GENERIC_IN_EPADDR,
						//.Size                 = GENERIC_EPSIZE,
						//.Banks                = 1,
					//},
				//.PrevReportINBuffer           = PrevHIDReportBuffer,
				//.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
			//},
	//};
	
struct cRGB LEDArray[N_LEDS];	// LED Color Array

ISR(TIMER0_OVF_vect) {
	// Timer code
}

ISR(SPI_STC_vect){
	// SPI Byte Transmitted
}

int main(void){
	uint8_t sensor8bPacket = 0x00;
	uint8_t counter = 0;
	
	configurePorts();				// Set up I/O ports
	
	enableLEDs();
	enableSPI();
	_delay_ms(500);		// Allow 500ms to turn on 5v power block
	
	sensor8bPacket = spiRxTX(WHO_AM_I, SPI_READ, 0x00);
	if(sensor8bPacket != 0x49){
		while(1){
			printLED8bCode(0xAA);
			_delay_ms(300);
			printLED8bCode(sensor8bPacket);
			_delay_ms(1000);
		}
	}
	else{
		printLED8bCode(0xFF);
		_delay_ms(300);
	}
	
	setUpLSM();
	
	// Main loop
	while(1){		
		sensor8bPacket = spiRxTX(OUT_Y_H_M, SPI_READ, 0x00);
		setLEDColor(&LEDArray[4], LED_COLOR_RED);
		printLED8bCode(sensor8bPacket);
		_delay_ms(50);
    }
	//printLED8bCode(counter);
	//counter++;
}

void setUpLSM(){
	uint8_t sensorPacket;
	
	//spiRxTX(CTRL0, SPI_WRITE, 0B01000000);
	spiRxTX(CTRL0, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL1, SPI_WRITE, 0B01010111);
	spiRxTX(CTRL2, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL3, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL4, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL5, SPI_WRITE, 0B01110000);
	spiRxTX(CTRL6, SPI_WRITE, 0B01100000);
	spiRxTX(CTRL7, SPI_WRITE, 0B00000000);
	//spiRxTX(FIFO_CTRL, SPI_WRITE, 0B01000000);
}

uint8_t spiRxTX(uint8_t address, uint8_t RW, uint8_t data){
	uint8_t retVal;
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	
	writeByteToSPI(RW|address);
	retVal = writeByteToSPI(data);
	
	PORT_SPI_SS |= (1<<PIN_SPI_SS);
	
	return retVal;
}

void printLED8bCode(uint8_t code){
	setLEDColor(&LEDArray[0], code&0x3);
	setLEDColor(&LEDArray[1], (code>>2)&0x3);
	setLEDColor(&LEDArray[2], (code>>4)&0x3);
	setLEDColor(&LEDArray[3], (code>>6)&0x3);
	apa102_setleds(LEDArray,5);
}

void uint8ToRGB(uint8_t frac){
	struct cRGB result;
	result.fade = 0.1;
	
	if(frac < 0x80){
		result.r = (0x80 - frac) * 2;
		result.g = (frac) * 2;
		result.b = 0x00;
	}
	else{
		result.r = 0;
		result.g = (0xFF - frac) * 2;
		result.b = (frac - 0x80) * 2;
	}
	
	LEDArray[4] = result;
	//return result;
}

void int16ToRGB(int16_t frac){
	struct cRGB result;
	result.fade = 0.1;
	
	if(frac < 0){
		result.r = (uint8_t)((0 - frac) * 2)>>8;
		result.g = (uint8_t)((frac - 0xBFFF) * 2)>>8;
		result.b = 0x00;
	}
	else{
		result.r = 0;
		result.g = (uint8_t)((0x7FFF - frac) * 2)>>8;
		result.b = (uint8_t)((frac) * 2)>>8;
	}
	
	LEDArray[4] = result;
	//return result;
}

void setLEDColor(struct cRGB *LED, uint8_t color){
	switch (color){
		case LED_COLOR_WHITE:
		LED[0].r = 0xFF;
		LED[0].g = 0xFF;
		LED[0].b = 0xFF;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_GREEN:
		LED[0].r = 0x00;
		LED[0].g = 0xFF;
		LED[0].b = 0x00;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_BLUE:
		LED[0].r = 0x00;
		LED[0].g = 0x00;
		LED[0].b = 0xFF;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_RED:
		LED[0].r = 0xFF;
		LED[0].g = 0x00;
		LED[0].b = 0x00;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_YELLOW:
		LED[0].r = 0xFF;
		LED[0].g = 0xFF;
		LED[0].b = 0x00;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_BROWN:
		LED[0].r = 0xA5;
		LED[0].g = 0x2A;
		LED[0].b = 0x2A;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_ORANGE:
		LED[0].r = 0xFF;
		LED[0].g = 0xA5;
		LED[0].b = 0x00;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_PINK:
		LED[0].r = 0xFF;
		LED[0].g = 0xC0;
		LED[0].b = 0xCB;
		LED[0].fade = 0x01;
		break;
		
		case LED_COLOR_OFF:
		LED[0].r = 0x00;
		LED[0].g = 0x00;
		LED[0].b = 0x00;
		LED[0].fade = 0x01;
		break;
	
	}
}

void enableLEDs(){
	PORT_BOOST_EN |= (1<<PIN_BOOST_EN);
}

void disableLEDs(){
	PORT_BOOST_EN &= ~(1<<PIN_BOOST_EN);
}

void configurePorts(){
	// Configure PortB I/O directionality
	DDRB = (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB5) | (1<<DDB6);
	
	// Configure PortC I/O directionality
	DDRC = (1<<DDC6) | (1<<DDC7);
	
	// Configure PortD I/O directionality
	DDRD = (1<<DDD3) | (1<<DDD4);
	
	PORT_SPI_SS |= (1<<PIN_SPI_SS);
	PORT_SPI_SCK |= (1<<PIN_SPI_SCK);
}
