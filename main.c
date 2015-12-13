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
	int16_t sensor16bPacket = 0x00;
	uint8_t error = 0;
	
	configurePorts();				// Set up I/O ports
	//setupUSBHardware();
	
	//TCCR0B = (1<<CS00);			// 001 = CLK; 010 = CLK/8; 011 = CLK/64
									// 100 = CLK/256; 101 = CLK/1024
	//TIMSK0 = (1<<TOIE0);			// TOIE0: Timer/Counter0 Overflow Interrupt Enable
									// OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
									// OCIE0B: Timer/Counter0 Output Compare Match B Interrupt Enable
	//TCNT0 = 0x00;					// Timer/Counter Register - Set to 0 at start
	
	enableLEDs();
	enableSPI();
	//sei();
	
	_delay_ms(500);		// Allow 500ms to turn on 5v power block
	LEDArray[0].r=0x50;	LEDArray[0].g=0x50;	LEDArray[0].b=0xFF;	LEDArray[0].fade=0x01;
	LEDArray[1].r=0x50;	LEDArray[1].g=0x50;	LEDArray[1].b=0xFF;	LEDArray[1].fade=0x01;
	LEDArray[2].r=0x50;	LEDArray[2].g=0x50;	LEDArray[2].b=0xFF;	LEDArray[2].fade=0x01;
	LEDArray[3].r=0x50;	LEDArray[3].g=0x50;	LEDArray[3].b=0xFF;	LEDArray[3].fade=0x01;
	LEDArray[4].r=0x50;	LEDArray[4].g=0x50;	LEDArray[4].b=0xFF;	LEDArray[4].fade=0x01;
	apa102_setleds(LEDArray,5);
	
   // Main loop
    while(1){		
		readByteLSM(INT_SRC_M, &sensor8bPacket);
		if(sensor8bPacket){
			setLEDColor(&LEDArray[1], LED_COLOR_OFF);
			apa102_setleds(LEDArray,5);
			_delay_ms(200);
			
			printLEDCode(sensor8bPacket);
			_delay_ms(1000);
			error = 1;
		}
		
		readByteLSM(WHO_AM_I, &sensor8bPacket);
		if(sensor8bPacket != 0x49){
			setLEDColor(&LEDArray[2], LED_COLOR_OFF);
			apa102_setleds(LEDArray,5);
			_delay_ms(200);
			
			printLEDCode(sensor8bPacket);
			_delay_ms(1000);
			error = 1;
		}
		
		if(error == 0){
			setLEDColor(&LEDArray[1], LED_COLOR_GREEN);
			apa102_setleds(LEDArray,5);
			_delay_ms(200);
			
			readBytesLSM(OUT_X_L_A, &sensor16bPacket);
			printLEDCode(sensor16bPacket);
			_delay_ms(200);
		}
		
		error = 0;
    }
}

void printLEDTherm16bCode(int16_t therm){
	uint8_t r, g, b;
	
	if(therm < 0) therm = 0 - therm;
	
	if(therm > 0x0007){
		if(therm > 0x000F){
			if(therm > 0x007F){
				if(therm > 0x00FF){
					if(therm > 0x07FF){
						r=0xFF; g = 0x00; b = 0x00;
					}
					else{
						r=0x00; g = 0xFF; b = 0x00;
					}
				}
				else{
					r=0x00; g = 0x00; b = 0xFF;
				}
			}
			else{
				r=0xFF; g = 0xFF; b = 0x00;
			}
		}
		else{
			r=0xFF; g = 0x00; b = 0xFF;
		}
	}
	else{
		r=0x00; g = 0xFF; b = 0xFF;
	}
	
	LEDArray[0].r=r;	LEDArray[0].g=g;	LEDArray[0].b=b;	LEDArray[0].fade=0x01;
	LEDArray[1].r=r;	LEDArray[1].g=g;	LEDArray[1].b=b;	LEDArray[1].fade=0x01;
	LEDArray[2].r=r;	LEDArray[2].g=g;	LEDArray[2].b=b;	LEDArray[2].fade=0x01;
	LEDArray[3].r=r;	LEDArray[3].g=g;	LEDArray[3].b=b;	LEDArray[3].fade=0x01;
	LEDArray[4].r=r;	LEDArray[4].g=g;	LEDArray[4].b=b;	LEDArray[4].fade=0x01;
	apa102_setleds(LEDArray,5);
}

void printLEDCode(uint16_t code){
	setLEDColor(&LEDArray[0], code&0x3);
	setLEDColor(&LEDArray[1], (code>>3)&0x7);
	setLEDColor(&LEDArray[2], (code>>6)&0x7);
	setLEDColor(&LEDArray[3], (code>>9)&0x7);
	setLEDColor(&LEDArray[4], (code>>12)&0x7);
	apa102_setleds(LEDArray,5);
}

void setLEDColor(struct cRGB *LED, uint8_t color){
	switch (color){
		case LED_COLOR_WHITE:
		LED[0].r = 0xFF;
		LED[0].g = 0xFF;
		LED[0].b = 0xFF;
		break;
		
		case LED_COLOR_GREEN:
		LED[0].r = 0x00;
		LED[0].g = 0xFF;
		LED[0].b = 0x00;
		break;
		
		case LED_COLOR_BLUE:
		LED[0].r = 0x00;
		LED[0].g = 0x00;
		LED[0].b = 0xFF;
		break;
		
		case LED_COLOR_RED:
		LED[0].r = 0xFF;
		LED[0].g = 0x00;
		LED[0].b = 0x00;
		break;
		
		case LED_COLOR_YELLOW:
		LED[0].r = 0xFF;
		LED[0].g = 0xFF;
		LED[0].b = 0x00;
		break;
		
		case LED_COLOR_BROWN:
		LED[0].r = 0xA5;
		LED[0].g = 0x2A;
		LED[0].b = 0x2A;
		break;
		
		case LED_COLOR_ORANGE:
		LED[0].r = 0xFF;
		LED[0].g = 0xA5;
		LED[0].b = 0x00;
		break;
		
		case LED_COLOR_PINK:
		LED[0].r = 0xFF;
		LED[0].g = 0xC0;
		LED[0].b = 0xCB;
		break;
		
		case LED_COLOR_OFF:
		LED[0].r = 0x00;
		LED[0].g = 0x00;
		LED[0].b = 0x00;
		//LED[0].fade = 0x00;
		break;
	
	}
}

void readByteLSM(uint8_t address, uint8_t* data){
	address |= 0x80;
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	_NOP();
	
	writeByteToSPI(address);
	*data = writeByteToSPI(0x00);
	 
	PORT_SPI_SS |= PIN_SPI_SS;
}

// Read two bytes in consecutive addresses
void readBytesLSM(uint8_t address, int16_t* data){
	address |= 0x80 | (0x40);
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	_NOP();
	
	writeByteToSPI(address);
	*data = writeByteToSPI(0x00);
	*data |= ((int16_t)writeByteToSPI(0x00))<<8;
	
	PORT_SPI_SS |= PIN_SPI_SS;
}

void writeByteLSM(uint8_t address, uint8_t data){
	address &= ~(0x80);
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	_NOP();
	
	writeByteToSPI(address);
	writeByteToSPI(data);
	
	PORT_SPI_SS |= PIN_SPI_SS;
}

void writeBytesLSM(uint8_t address, uint8_t* data, uint8_t nBytes, uint8_t MS){
	address &= ~(0x80);
	address |= (0x40 & MS);
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	_NOP();
	
	writeByteToSPI(address);
	for(uint8_t i = 0; i < nBytes; i++){
		writeByteToSPI(data[i]);
	}
	
	PORT_SPI_SS |= PIN_SPI_SS;
}

void enableLEDs(){
	PORT_BOOST_EN |= (1<<PIN_BOOST_EN);
}

void disableLEDs(){
	PORT_BOOST_EN &= ~(1<<PIN_BOOST_EN);
}

void configurePorts(){
	// Configure PortB I/O directionality
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB5) | (1<<DDB6);
	DDRB &= ~(1<<DDB3);
	
	// Configure PortC I/O directionality
	DDRC |= (1<<DDC6) | (1<<DDC7);
	
	// Configure PortD I/O directionality
	//DDRD |= (1<<DDD3) | (1<<DDD4);
	DDRD &= ~((1<<DDD2) | (1<<DDD6) | (1<<DDD7));
}

//
///** Configures the board hardware and chip peripherals for the demo's functionality. */
//void setupUSBHardware(void){
	///* Disable watchdog if enabled by bootloader/fuses */
	//MCUSR &= ~(1 << WDRF);
	//wdt_disable();
//
	///* Disable clock division */
	//clock_prescale_set(clock_div_1);
	//
	///* Hardware Initialization */
	//USB_Init();
//}
//
///** Event handler for the library USB Connection event. */
//void EVENT_USB_Device_Connect(void)
//{
	//
//}
//
///** Event handler for the library USB Disconnection event. */
//void EVENT_USB_Device_Disconnect(void)
//{
	//
//}
//
///** Event handler for the library USB Configuration Changed event. */
//void EVENT_USB_Device_ConfigurationChanged(void)
//{
	//bool ConfigSuccess = true;
//
	//ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);
//
	//USB_Device_EnableSOFEvents();
//}
//
///** Event handler for the library USB Control Request reception event. */
//void EVENT_USB_Device_ControlRequest(void)
//{
	//HID_Device_ProcessControlRequest(&Generic_HID_Interface);
//}
//
///** Event handler for the USB device Start Of Frame event. */
//void EVENT_USB_Device_StartOfFrame(void)
//{
	//HID_Device_MillisecondElapsed(&Generic_HID_Interface);
//}
//
///** HID class driver callback function for the creation of HID reports to the host.
 //*
 //*  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 //*  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 //*  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 //*  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 //*  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 //*
 //*  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 //*/
//bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         //uint8_t* const ReportID,
                                         //const uint8_t ReportType,
                                         //void* ReportData,
                                         //uint16_t* const ReportSize)
//{
	//uint8_t* Data        = (uint8_t*)ReportData;
	////uint8_t  CurrLEDMask = LEDs_GetLEDs();
//
	//Data[0] = ((1) ? 1 : 0);
	//Data[1] = ((1) ? 1 : 0);
	//Data[2] = ((0) ? 1 : 0);
	//Data[3] = ((1) ? 1 : 0);
//
	//*ReportSize = GENERIC_REPORT_SIZE;
	//return false;
//}
//
///** HID class driver callback function for the processing of HID reports from the host.
 //*
 //*  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 //*  \param[in] ReportID    Report ID of the received report from the host
 //*  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 //*  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 //*  \param[in] ReportSize  Size in bytes of the received HID report
 //*/
//void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          //const uint8_t ReportID,
                                          //const uint8_t ReportType,
                                          //const void* ReportData,
                                          //const uint16_t ReportSize)
//{
	//uint8_t* Data       = (uint8_t*)ReportData;
	////uint8_t  NewLEDMask = LEDS_NO_LEDS;
//
	////if (Data[0])
	  ////NewLEDMask |= LEDS_LED1;
////
	////if (Data[1])
	  ////NewLEDMask |= LEDS_LED2;
////
	////if (Data[2])
	  ////NewLEDMask |= LEDS_LED3;
////
	////if (Data[3])
	  ////NewLEDMask |= LEDS_LED4;
//
	////LEDs_SetAllLEDs(NewLEDMask);
//}