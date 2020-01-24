#include "il9341.h"
#include "st7789_commands.h"
#include "light_ws2812.h"

struct cRGB led[8];

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

	for (int i=0; i<8;i++)
	{
		led[i].r=0;
		led[i].g=0;
		led[i].b=0;
	}

	ws2812_setleds(led,8);

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
		case 2:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQREC_INTERFACE))
			{

				Endpoint_ClearSETUP();

				Endpoint_Write_16_LE(240);
				Endpoint_Write_16_LE(240);

				Endpoint_ClearIN();
				Endpoint_ClearStatusStage();
			}

			break;
		case 3:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_INTERFACE))
			{

				Endpoint_ClearSETUP();
				while (!(Endpoint_IsOUTReceived()));

				uint16_t lednr= Endpoint_Read_8();
				uint16_t r= Endpoint_Read_8();
				uint16_t g= Endpoint_Read_8();
				uint16_t b= Endpoint_Read_8();

				Endpoint_ClearOUT();
				Endpoint_ClearStatusStage();
				led[lednr].r=r;
				led[lednr].g=g;
				led[lednr].b=b;
				ws2812_setleds(led,8);
			}

			break;
	}
}

static const uint8_t PROGMEM initcmd[] = {
    10,                       				// 9 commands in list:
    ST7789_SWRESET,   ST_CMD_DELAY,  		// 1: Software reset, no args, w/delay
      150,                     				// 150 ms delay
    ST7789_SLPOUT ,   ST_CMD_DELAY,  		// 2: Out of sleep mode, no args, w/delay
      255,                    				// 255 = 500 ms delay
    ST7789_COLMOD , 1+ST_CMD_DELAY,  		// 3: Set color mode, 1 arg + delay:
      0x55,                   				// 16-bit color
      10,                     				// 10 ms delay
    ST7789_MADCTL , 1,  					// 4: Memory access ctrl (directions), 1 arg:
      0x00,                   				// Row addr/col addr, bottom to top refresh
    ST7789_CASET  , 4,  					// 5: Column addr set, 4 args, no delay:
      0x00, ST7789_240x240_XSTART,          // XSTART = 0
	  (ST7789_TFTWIDTH+ST7789_240x240_XSTART) >> 8,
	  (ST7789_TFTWIDTH+ST7789_240x240_XSTART) & 0xFF,   // XEND = 240
    ST7789_RASET  , 4,  					// 6: Row addr set, 4 args, no delay:
      0x00, ST7789_240x240_YSTART,          // YSTART = 0
      (ST7789_TFTHEIGHT+ST7789_240x240_YSTART) >> 8,
	  (ST7789_TFTHEIGHT+ST7789_240x240_YSTART) & 0xFF,	// YEND = 240
    ST7789_INVON ,   ST_CMD_DELAY,  		// 7: Inversion ON
      10,
    ST7789_NORON  ,   ST_CMD_DELAY,  		// 8: Normal display on, no args, w/delay
      10,                     				// 10 ms delay
    ST7789_DISPON ,   ST_CMD_DELAY,  		// 9: Main screen turn on, no args, w/delay
    255 };                                  // 255 = 500 ms delay


void sendCommand(uint8_t command, uint8_t *args, size_t len)
{
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
	sendCommand(ST7789_CASET,buff,4);
	put16BE(y,&buff[0]);
	put16BE(y+h-1,&buff[2]);
	sendCommand(ST7789_RASET,buff,4);
	sendCommand(ST7789_RAMWR,NULL,0);
}

void IL9341_Init(void)
{
	SPCR=(1<<CPOL)|(0<<CPHA)|(1<<SPE)|(0<<DORD)|(0<<SPR1)|(0<<SPR0)|(1<<MSTR)|(0<<SPIE);
	SPSR=(1<<SPI2X);
	DDRB|=0x06;
	PORTB|=0x40;
	DDRB|=0x40; //DC
	PORTF|=0x80;

	SPDR=0x00; // dummy write so SPIF is set
	while(!(SPSR & _BV(SPIF)));

	uint8_t        cmd;
	uint8_t        commandargs[10];
	const uint8_t *addr = initcmd;
	uint8_t  numCommands, numArgs;
  	uint16_t ms;
	numCommands = pgm_read_byte(addr++);   // Number of commands to follow
	while(numCommands--) {                 // For each command...
    	cmd=       (pgm_read_byte(addr++)); //   Read, issue command
    	numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    	ms       = numArgs & ST_CMD_DELAY;   //   If hibit set, delay follows args
    	numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    	for(int i=0;i<numArgs;i++)
			commandargs[i]=pgm_read_byte(addr++);
		sendCommand(cmd, commandargs, numArgs);

    	if(ms) {
     		ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      		if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      		Delay_MS(ms);
    	}
  }

	IL9341_SetFrame(0,0,240,240);

	PORTB|=0x40; //DC

	for(uint16_t y=0;y<240;y++)
		for(uint8_t x=0;x<240;x++)
		{
			sendData8(0xff); //R
			sendData8(0xe0); //G
		}
	while(!(SPSR & _BV(SPIF)));
}

