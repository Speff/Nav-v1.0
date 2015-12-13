/*
 * Nav-v1.0 Firmware
 *
 * Created: 12/10/2015 9:00:32 PM
 * Author : Sahil Patel
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "main.h"

#include "Light_apa102/apa102_config.h"
#include "Light_apa102/light_apa102.h"
//#include "LUFA/GenericHID.h"

// Port Usages
// Port B
// -B0 | Output	| Accelerometer SPI !SS
// -B1 | Output	| Accelerometer SPI CLK & ISP Header
// -B2 | Output | Accelerometer SPI MOSI & ISP Header
// -B3 | Input	| Accelerometer SPI MISO & ISP Header
// -B5 | Output	| Serial LED Data
// -B6 | Output	| Serial LED Clock
// Port C
// -C6 | Output	| Toggle USB max current flow (Active high - need thermistor installed)
// -C7 | Output | Activate/Deactivate 5v boost converter
// Port D
// -D2 | Input	| GPS UART RX
// -D3 | Output | GPS UART TX
// -D4 | Output	| Enable GPS
// -D6 | Input	| Accelerometer Interrupt #1
// -D7 | Input	| Accelerometer Interrupt #2

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



ISR(TIMER0_OVF_vect) {
	// Timer code
}

int main(void){
	struct cRGB LEDArray[N_LEDS];	// LED Color Array
	
	configurePorts();				// Set up I/O ports
	//setupUSBHardware();
	
	//TCCR0B = (1<<CS00);			// 001 = CLK; 010 = CLK/8; 011 = CLK/64
									// 100 = CLK/256; 101 = CLK/1024
	//TIMSK0 = (1<<TOIE0);			// TOIE0: Timer/Counter0 Overflow Interrupt Enable
									// OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
									// OCIE0B: Timer/Counter0 Output Compare Match B Interrupt Enable
	//TCNT0 = 0x00;					// Timer/Counter Register - Set to 0 at start
	
	enableLEDs();
	
	_delay_ms(500);		// Allow 500ms to turn on 5v power block
	LEDArray[0].r=0x00;	LEDArray[0].g=0x00;	LEDArray[0].b=0xFF;	LEDArray[0].fade=0x01;
	LEDArray[1].r=0xFF;	LEDArray[1].g=0xFF;	LEDArray[1].b=0xFF;	LEDArray[1].fade=0x01;
	LEDArray[2].r=0xFF;	LEDArray[2].g=0x00;	LEDArray[2].b=0x00;	LEDArray[2].fade=0x01;
	LEDArray[3].r=0xFF;	LEDArray[3].g=0xFF;	LEDArray[3].b=0xFF;	LEDArray[3].fade=0x01;
	LEDArray[4].r=0xFF;	LEDArray[4].g=0x00;	LEDArray[4].b=0x00;	LEDArray[4].fade=0x01;
	apa102_setleds(LEDArray,5);

	sei();
	
   // Main loop
    while(1){
			
    }
}

void enableLEDs(){
	PORT_BOOST_EN |= (1<<PIN_BOOST_EN);
}

void disableLEDs(){
	PORT_BOOST_EN &= ~(1<<PIN_BOOST_EN);
}

void configurePorts(){
	// Configure PortC I/O directionality
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