/*
 * Nav-v1.0 Firmware
 *
 * Created: 12/10/2015 9:00:32 PM
 * Author : Sahil Patel
 */ 

#include "main.h"
	
struct cRGB LEDArray[N_LEDS];	// LED Color Array
struct XYZData magData, accelData;
struct IMUReadings usbIMUOutput;
volatile uint8_t timer0Hit = 0;
volatile uint8_t gpsHit = 0;
volatile uint8_t gpsData;
char uartString[90];
char completedUARTString[90];
volatile uint8_t usbTXPos = 0xFF;
volatile uint8_t dataSwitch = 0;
float targetLat = 3548.30481;
float targetLong = 78511.89468;

/** Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevHIDReportBuffer[GENERIC_REPORT_SIZE];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Generic_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber              = INTERFACE_ID_GenericHID,
				.ReportINEndpoint             =
					{
						.Address              = GENERIC_IN_EPADDR,
						.Size                 = GENERIC_EPSIZE,
						.Banks                = 1,
					},
				.PrevReportINBuffer           = PrevHIDReportBuffer,
				.PrevReportINBufferSize       = sizeof(PrevHIDReportBuffer),
			},
	};

ISR(USART1_RX_vect){
	gpsData = UDR1;
	gpsHit = 1;
}

ISR(TIMER0_COMPA_vect) {
	// Timer code
	timer0Hit++;
	//HID_Device_USBTask(&Generic_HID_Interface);
	//USB_USBTask();
}

ISR(SPI_STC_vect){
	// SPI Byte Transmitted
}

int main(void){
	//uint8_t sensor8bPacket = 0x00;
	//int16_t sensor16bPacket = 0x0000;
	//uint8_t uartStringBitPos = 0;
	struct IMUReadings lsmReadings;
	
	TCCR0B = (1<<CS02) | (1<<CS00);	// 001 = CLK; 010 = CLK/8; 011 = CLK/64
									// 100 = CLK/256; 101 = CLK/1024
	TIMSK0 = (1<<OCIE0A);			// TOIE0: Timer/Counter0 Overflow Interrupt Enable
									// OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
									// OCIE0B: Timer/Counter0 Output Compare Match B Interrupt Enable
	//OCR0A = 156;					// Compared to Timer0. Interrupt hit when matched
	OCR0A = 4;
	TCNT0 = 0x00;					// Timer/Counter Register - Set to 0 at start
	
	
	configurePorts();	// Set up I/O ports
	
	enableLEDs();
	_delay_ms(500);		// Allow 500ms to turn on 5v power block
	
	setUpLSM(&lsmReadings);
	//SetupUSBHardware();
	//setUpUART();
	
	setLEDColor(&LEDArray[0], LED_COLOR_TURQUOISE);
	setLEDColor(&LEDArray[1], LED_COLOR_TURQUOISE);
	setLEDColor(&LEDArray[2], LED_COLOR_TURQUOISE);
	setLEDColor(&LEDArray[3], LED_COLOR_TURQUOISE);
	setLEDColor(&LEDArray[4], LED_COLOR_TURQUOISE);
	apa102_setleds(LEDArray,5);
	
	sei();
	timer0Hit = 0;
	//gpsHit = 0;
	//uartString[0] = 0xFA;
	
	
	// Main loop
	while(1){		
		//HID_Device_USBTask(&Generic_HID_Interface);
		//USB_USBTask();

		// if(gpsHit){
		// 	setLEDColor(&LEDArray[4], LED_COLOR_RED);
		// 	apa102_setleds(LEDArray,5);
			
		// 	uartString[uartStringBitPos] = gpsData;
		// 	if(uartString[0] != '$') uartStringBitPos = 0;
		// 	else{
		// 		if(uartString[uartStringBitPos] == '\n'){
		// 			if(strcmp(uartString, "$GP")){
		// 				fillCompletedString();
		// 				uartStringBitPos = 0;
		// 			}
		// 			uartStringBitPos = 0;
		// 		}
		// 		if(uartStringBitPos == sizeof(uartString)-1){
		// 			uartStringBitPos = 0;
		// 			setLEDColor(&LEDArray[3], LED_COLOR_RED);
		// 			apa102_setleds(LEDArray,5);
		// 		}
		// 		else uartStringBitPos++;
		// 	}
		// 	setLEDColor(&LEDArray[4], LED_COLOR_BLUE);
		// 	apa102_setleds(LEDArray,5);
			
		// 	gpsHit = 0;
		// }
		
		if(timer0Hit){
			processCompass(&lsmReadings);
			usbIMUOutput = lsmReadings;
			floatTo360RGB(lsmReadings.heading, &LEDArray[0]);
			memcpy(&LEDArray[1], &LEDArray[0], sizeof(LEDArray));
			memcpy(&LEDArray[2], &LEDArray[0], sizeof(LEDArray));
			memcpy(&LEDArray[3], &LEDArray[0], sizeof(LEDArray));
			memcpy(&LEDArray[4], &LEDArray[0], sizeof(LEDArray));
			apa102_setleds(LEDArray,5);
			timer0Hit = 0;
		}
		

		
		//magData.x = spiRxTX16b(OUT_X_L_M, SPI_READ, 0x0000);
		//magData.y = spiRxTX16b(OUT_Y_L_M, SPI_READ, 0x0000);
		//magData.z = spiRxTX16b(OUT_Z_L_M, SPI_READ, 0x0000);
		//accelData.x = spiRxTX16b(OUT_X_L_A, SPI_READ, 0x0000);
		//accelData.y = spiRxTX16b(OUT_Y_L_A, SPI_READ, 0x0000);
		//accelData.z = spiRxTX16b(OUT_Z_L_A, SPI_READ, 0x0000);
		
    }
}

void fillCompletedString(){
	//uint8_t currentPos = 0;
	
	setLEDColor(&LEDArray[3], LED_COLOR_GREEN);
	setLEDColor(&LEDArray[1], LED_COLOR_GREEN);
	apa102_setleds(LEDArray,5);
	
	memcpy(completedUARTString, uartString, sizeof(uartString));
	for(uint8_t i = 0; i < sizeof(uartString); i++) uartString[i] = 0;
	usbTXPos = 0;
}

void setUpUART(){
	
	
	enableGPS();
	_delay_ms(30);	// Delay uC in order to allow GPS to power up
	
	UBRR1H = 0x00;	// Set high bit for UBRR (baud rate = 9600)
	UBRR1L = 0x33;	// Set low bit for UBRR (baud rate = 9600)
	
	UCSR1B |= (1<<RXCIE1) | (0<<TXCIE1) |	// Enable RX & TX transmission complete interrupts
				(1<<RXEN1) | (0<<TXEN1);	// Enable RX & TX modules
	UCSR1C |= (0<<UMSEL11) | (0<<UMSEL10)|	// Set for asynchronous TX/RX
				(0<<UPM11) | (0<<UPM10) |	// Disable parity bits
				(0<<USBS1) |				// 1 stop bit
				(1<<UCSZ11) | (1<<UCSZ10)|	// 8-bit data packet size
				(0<<UCPOL1);				// Set to zero in aynch mode	
}

void enableGPS(){
	PORT_EN_GPS |= (1<<PIN_EN_GPS);
}

void disableGPS(){
	PORT_EN_GPS &= ~(1<<PIN_EN_GPS);
}

void processCompass(struct IMUReadings *readings){
	float magDataf[3];
	float magDatafA[3];
	float magDataHf[2];
	float magBias[3] = {0.3082,0.0145,-0.1260};
	float magTMatrix[3][3] = {{7.5581,0.3554,0.0115}, {0.3554,7.3012,-0.1628}, {0.0115,-0.1628,7.3637}};
	float accelDataf[2];
	
	magData.x = spiRxTX16b(OUT_X_L_M, SPI_READ, 0x0000);
	magData.y = spiRxTX16b(OUT_Y_L_M, SPI_READ, 0x0000);
	magData.z = spiRxTX16b(OUT_Z_L_M, SPI_READ, 0x0000);
	accelData.x = spiRxTX16b(OUT_X_L_A, SPI_READ, 0x0000);
	accelData.y = spiRxTX16b(OUT_Y_L_A, SPI_READ, 0x0000);
	//accelData.z = spiRxTX16b(OUT_Z_L_A, SPI_READ, 0x0000);
	
	accelDataf[0] = ACL_RES * (float)accelData.x;
	accelDataf[1] = ACL_RES * (float)accelData.y;
	//accelDataf[2] = ACL_RES * (float)accelData.z;
	if(accelDataf[0] > -1 && accelDataf[0] < 1) (*readings).pitch = asinf(-accelDataf[0]);
	if(accelDataf[0] > -1 && accelDataf[0] < 1) (*readings).roll = asinf(accelDataf[1]/cos((*readings).pitch));
	
	magDataf[0] = (float)magData.x * MAG_RES - magBias[0];
	magDataf[1] = (float)magData.y * MAG_RES - magBias[1];
	magDataf[2] = (float)magData.z * MAG_RES - magBias[2];
	magDatafA[0] = magDataf[0]*magTMatrix[0][0] + magDataf[1]*magTMatrix[0][1] + magDataf[2]*magTMatrix[0][2];
	magDatafA[1] = magDataf[0]*magTMatrix[1][0] + magDataf[1]*magTMatrix[1][1] + magDataf[2]*magTMatrix[1][2];
	magDatafA[2] = magDataf[0]*magTMatrix[2][0] + magDataf[1]*magTMatrix[2][1] + magDataf[2]*magTMatrix[2][2];
	
	magDataHf[0] = magDatafA[0]*cos((*readings).pitch) + magDatafA[2]*sin((*readings).pitch);
	magDataHf[1] = magDatafA[0]*sin((*readings).roll)*sin((*readings).pitch)+magDatafA[1]*cos((*readings).roll)-magDatafA[2]*sin((*readings).roll)*cos((*readings).pitch);
	(*readings).heading = atan2f(magDataHf[1], magDataHf[0]) * 180.0/M_PI;
	
	(*readings).heading += 90.0 + 4.066;
	if((*readings).heading < 0) (*readings).heading += 360.0;
	//if((*readings).heading > 360) (*readings).heading -= 360.0;
}

void int16ToRGB(int16_t frac, int16_t min, int16_t max, struct cRGB *LED){
	int16_t middle = (max + min)>>2;
	int16_t r, g, b;
	
	(*LED).fade = 0x03;
	
	if(frac < middle){
		r = middle - frac;
		g = frac - min;
		b = 0x00;
	}
	else{
		r = 0x00;
		g = max - frac;
		b = frac - middle;
	}
	
	if(r > 0xFF) (*LED).r = 0xFF;
	else (*LED).r = r;
	if(g > 0xFF) (*LED).g = 0xFF;
	else (*LED).g = g;
	if(b > 0xFF) (*LED).b = 0xFF;
	else (*LED).b = b;
}

void doubleToRGB(double frac, double min, double max, struct cRGB *LED){
	double middle = (max + min)/2;
	double r, g, b;
	
	(*LED).fade = 0x03;
	
	if(frac < middle){
		r = middle - frac;
		g = frac - min;
		b = 0x00;
	}
	else{
		r = 0x00;
		g = max - frac;
		b = frac - middle;
	}
	
	if(r > 0xFF) (*LED).r = 0xFF;
	else (*LED).r = (uint8_t)r;
	if(g > 0xFF) (*LED).g = 0xFF;
	else (*LED).g = (uint8_t)g;
	if(b > 0xFF) (*LED).b = 0xFF;
	else (*LED).b = (uint8_t)b;
}

void floatTo360RGB(float frac, struct cRGB *LED){
	float ledSF = 3.1875;
	float r, g, b;
	
	if(frac < 5.0){
		r = 255.0;
		g = 0.0;
		b = 0.0;
	}
	else if(frac < 85.0 && frac > 5.0){
		r = (85.0 - frac) * ledSF;
		g = (frac - 5.0) * ledSF;
		b = 0.0;
	}
	else if(frac < 95.0 && frac > 85.0){
		r = 0.0;
		g = 255.0;
		b = 0.0;
	}
	else if(frac < 175.0 && frac > 95.0){
		r = 0.0;
		g = (175.0 - frac) * ledSF;
		b = (frac - 95.0) * ledSF;
	}
	else if(frac < 185.0 && frac > 175.0){
		r = 0.0;
		g = 0.0;
		b = 255.0;
	}
	else if(frac < 265.0 && frac > 185.0){
		r = 0.0;
		g = (frac - 185.0) * ledSF;
		b = (265.0 - frac) * ledSF;
	}
	else if(frac < 275.0 && frac > 265.0){
		r = 0.0;
		g = 255.0;
		b = 0.0;
	}
	else if(frac < 355.0 && frac > 275.0){
		r = (frac - 275.0) * ledSF;
		g = (355.0 - frac) * ledSF;
		b = 0.0;
	}
	else{
		r = 255.0;
		g = 0.0;
		b = 0.0;
	}
	
	if(r > 0xFF) (*LED).r = 0xFF;
	else (*LED).r = (uint8_t)r;
	if(g > 0xFF) (*LED).g = 0xFF;
	else (*LED).g = (uint8_t)g;
	if(b > 0xFF) (*LED).b = 0xFF;
	else (*LED).b = (uint8_t)b;
}

void setUpLSM(struct IMUReadings *readings){
	uint8_t sensor8bPacket;
	//int16_t sensor16bPacket;

	enableSPI();
	
	//spiRxTX(CTRL0, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL1, SPI_WRITE, 0B01010111);
	spiRxTX(CTRL2, SPI_WRITE, 0B11001000);
	//spiRxTX(CTRL3, SPI_WRITE, 0B00000000);
	//spiRxTX(CTRL4, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL5, SPI_WRITE, 0B01101100);
	spiRxTX(CTRL6, SPI_WRITE, 0B00000000);
	spiRxTX(CTRL7, SPI_WRITE, 0B00000000);
	
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
		//printLED8bCode(0xFF);
		setLEDColor(&LEDArray[0], LED_COLOR_OFF);
		setLEDColor(&LEDArray[1], LED_COLOR_OFF);
		setLEDColor(&LEDArray[2], LED_COLOR_OFF);
		setLEDColor(&LEDArray[3], LED_COLOR_OFF);
		setLEDColor(&LEDArray[4], LED_COLOR_GREEN);
		apa102_setleds(LEDArray,5);
		_delay_ms(300);
	}

	(*readings).pitch = 0.0;
	(*readings).roll = 0.0;
	(*readings).heading = 0.0;
}

uint8_t spiRxTX(uint8_t address, uint8_t RW, uint8_t data){
	uint8_t retVal;
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	
	writeByteToSPI(RW|address);
	retVal = writeByteToSPI(data);
	
	PORT_SPI_SS |= (1<<PIN_SPI_SS);
	
	return retVal;
}

int16_t spiRxTX16b(uint8_t address, uint8_t RW, int16_t data){
	int16_t retVal = 0x0000;
	
	PORT_SPI_SS &= ~(1<<PIN_SPI_SS);
	
	writeByteToSPI(RW|SPI_MULTIPLEACCESS|address);
	retVal = writeByteToSPI(data);
	retVal |= writeByteToSPI(data>>8)<<8;
	
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

void setLEDColor(struct cRGB *LED, uint8_t color){
	switch (color){
		case LED_COLOR_WHITE:
		LED[0].r = 0xFF;
		LED[0].g = 0xFF;
		LED[0].b = 0xFF;
		LED[0].fade = 0x10;
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
		
		case LED_COLOR_TURQUOISE:
		LED[0].r = 0x20;
		LED[0].g = 0xFF;
		LED[0].b = 0xEF;
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

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupUSBHardware(void){
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface);

	USB_Device_EnableSOFEvents();

	//LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Generic_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Generic_HID_Interface);
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
	uint8_t* Data        = (uint8_t*)ReportData;

	if(dataSwitch){
		Data[0] = 0xBB;
		Data[1] = (magData.x)>>8;
		Data[2] = magData.x;
		Data[3] = (magData.y)>>8;
		Data[4] = magData.y;
		Data[5] = (magData.z)>>8;
		Data[6] = magData.z;
		dataSwitch = 0;
	}
	else{
		Data[0] = 0xCC;
		Data[1] = (accelData.x)>>8;
		Data[2] = accelData.x;
		Data[3] = (accelData.y)>>8;
		Data[4] = accelData.y;
		Data[5] = (accelData.z)>>8;
		Data[6] = accelData.z;
		dataSwitch = 1;
	}
	
	
	// for(uint8_t i = 0; i < 7; i++){
	// 	if(usbTXPos != sizeof(completedUARTString)-1){
	// 		Data[i] = completedUARTString[usbTXPos];
	// 		if(completedUARTString[usbTXPos] == '\n'){
	// 			Data[i] = completedUARTString[usbTXPos];
	// 			usbTXPos = sizeof(completedUARTString)-1;
	// 			break;
	// 		}
	// 		else usbTXPos++;
	// 	}
	// }
	
	//memcpy(Data, uartString, sizeof(uartString));

	*ReportSize = GENERIC_REPORT_SIZE;
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
	//uint8_t* Data       = (uint8_t*)ReportData;

	//if (Data[0])
	  //NewLEDMask |= LEDS_LED1;
//
	//if (Data[1])
	  //NewLEDMask |= LEDS_LED2;
//
	//if (Data[2])
	  //NewLEDMask |= LEDS_LED3;
//
	//if (Data[3])
	  //NewLEDMask |= LEDS_LED4;
//
	//LEDs_SetAllLEDs(NewLEDMask);
}

