#include "il9341.h"
#include "il9341_commands.h"

int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	for (;;)
	{
		USB_USBTask();

		{
			/* Select the data Endpoint */
			Endpoint_SelectEndpoint(IL9341_OUT_EPADDR);

			PORTF&=~0x80; //CS
			PORTB|=0x40; //DC

			/* Check if Keyboard LED Endpoint contains a packet */
			while (Endpoint_IsOUTReceived())
			{
				/* Check to see if the packet contains data */
				while(Endpoint_BytesInEndpoint())
				{
					uint8_t data= Endpoint_Read_8();
					while(!(SPSR & _BV(SPIF)));
					SPDR = data;
				}
				/* Handshake the OUT Endpoint - clear endpoint and ready for next report */
				Endpoint_ClearOUT();
			}
			while(!(SPSR & _BV(SPIF)));
			PORTF|=0x80; //CS

		}
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	USB_Disable();

	IL9341_Init();

	// Wait two seconds for the USB detachment to register on the host
	Delay_MS(2000);

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host sets the current configuration
 *  of the USB device after enumeration, and configures the keyboard device endpoints.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoints */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(IL9341_OUT_EPADDR, EP_TYPE_BULK, IL9341_EPSIZE, 2);

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
		case 0:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_INTERFACE))
			{

				Endpoint_ClearSETUP();
				Endpoint_ClearStatusStage();
				// If USB is used, detach from the bus and reset it
				USB_Disable();
    
				// Disable all interrupts
				cli();

				// Wait two seconds for the USB detachment to register on the host
				Delay_MS(2000);

				uint16_t bootKey = 0x7777;
				volatile uint16_t *const bootKeyPtr = (volatile uint16_t *)0x0800;
				*bootKeyPtr=bootKey;
				wdt_enable(WDTO_250MS);
				for (;;);
			}

			break;
		case 1:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_INTERFACE))
			{

				Endpoint_ClearSETUP();
				while (!(Endpoint_IsOUTReceived()));

				uint16_t x= Endpoint_Read_16_LE();
				uint16_t y= Endpoint_Read_16_LE();
				uint16_t w= Endpoint_Read_16_LE();
				uint16_t h= Endpoint_Read_16_LE();

				Endpoint_ClearOUT();
				Endpoint_ClearStatusStage();
				IL9341_SetFrame(x,y,w,h);
			}

			break;
	}
}

static const uint8_t PROGMEM initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  , 0x80,                // Exit Sleep
  ILI9341_DISPON  , 0x80,                // Display on
  0x00                                   // End of list
};


void sendCommand(uint8_t command, uint8_t *args, size_t len)
{
	PORTF&=~0x80; //CS
	PORTB&=~0x40; //DC
	while(!(SPSR & _BV(SPIF)));
	SPDR = command;
	while(!(SPSR & _BV(SPIF)));
	PORTB|=0x40; //DC
	for(int i=0;i<len;i++)
	{
		while(!(SPSR & _BV(SPIF)));
		SPDR = args[i];
	}
	while(!(SPSR & _BV(SPIF)));
	PORTF|=0x80; //CS
}

void put16BE(uint16_t data,uint8_t *buff)
{
	buff[0] = data>>8;
	buff[1] = data;
}
static inline void sendData8(uint8_t data)
{
	while(!(SPSR & _BV(SPIF)));
	SPDR = data;
}

void IL9341_SetFrame(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	uint8_t buff[4];
	put16BE(x,&buff[0]);
	put16BE(x+w-1,&buff[2]);
	sendCommand(ILI9341_CASET,buff,4);
	put16BE(y,&buff[0]);
	put16BE(y+h-1,&buff[2]);
	sendCommand(ILI9341_PASET,buff,4);
	sendCommand(ILI9341_RAMWR,NULL,0);
}

void IL9341_Init(void)
{
	SPCR=(0<<CPOL)|(0<<CPHA)|(1<<SPE)|(0<<DORD)|(0<<SPR1)|(0<<SPR0)|(1<<MSTR)|(0<<SPIE);
	SPSR=(1<<SPI2X);
	DDRB|=0x06;
	PORTB|=0x40;
	DDRB|=0x40; //DC
	PORTF|=0x80;
	DDRF|=0x80; //CS

	SPDR=0x00; // dummy write so SPIF is set
	while(!(SPSR & _BV(SPIF)));

	sendCommand(ILI9341_SWRESET, NULL, 0); // Engage software reset
	Delay_MS(150);

	uint8_t        cmd, x, numArgs;
	uint8_t        commandargs[10];
	const uint8_t *addr = initcmd;
	while((cmd = pgm_read_byte(addr++)) > 0) {
		x = pgm_read_byte(addr++);
		numArgs = x & 0x7F;
		for(int i=0;i<numArgs;i++)
			commandargs[i]=pgm_read_byte(addr++);
		sendCommand(cmd, commandargs, numArgs);
		if(x & 0x80) Delay_MS(150);
	}

	IL9341_SetFrame(0,0,240,320);

	PORTF&=~0x80; //CS
	PORTB|=0x40; //DC

	for(uint16_t y=0;y<320;y++)
		for(uint8_t x=0;x<240;x++)
		{
			sendData8(0xff); //R
			sendData8(0xe0); //G
		}
	while(!(SPSR & _BV(SPIF)));
	PORTF|=0x80; //CS
}

