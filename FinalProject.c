
// RC522 RFID Reader           
// SDA (SS) : connected to GPC8 (SPI1_SS)
// SCK (SCK) : connected to GPC9 (SPI1_CLK)
// MOSI : connected to GPC11(SPI1_MOSI)
// MISO : connected to GPC10(SPI1_MISO)
// IRQ : no connection
// GND : connected to Gnd
// RST : connected to 3.3V / no connection
// VCC : connected to 3.3V

//
// HC05 Bluetooth module
// pin1 : KEY   N.C
// pin2 : VCC   +5V
// pin3 : GND   GND
// pin4 : TXD   --> NUC140 UART0-RX (GPB0)
// pin5 : RXD   --> NUC140 UART0-TX (GPB1)
// pin6 : STATE N.C

// Joystick
// Vx : GPA0 (ADC0)
// Vy : GPA1 (ADC1)

// SG5010 DC servo (Horizontal)
// pin1 : signal to PWM0/GPA12 
// pin2 : Vcc
// pin3 : Gnd

// SG5010 DC servo (Vertical)
// pin1 : signal to PWM0/GPA13 
// pin2 : Vcc
// pin3 : Gnd

// Timer Capture :
// GPB2 / RTS0 / T2EX (NUC140 pin34)
// GPB4 / RX1         (NUC140 pin19)

// SR04 UltraSonic Sensor 
// pin1 Vcc : to Vcc
// pin2 Trig: to GPB4      (NUC140VE3xN pin19)
// pin3 ECHO: to GPB2/T2EX (NUC140VE3xN pin34)
// pin4 Gnd : to GND

#include <stdio.h>
#include <string.h> 
#include "NUC1xx.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvADC.h"
#include "Driver\DrvSPI.h"
#include "Driver_PWM_Servo.h"
#include "SPI_RC522.h"
#include "NUC1xx-LB_002\LCD_Driver.h"
   
#define	_SR04A_ECHO		   (GPB_2)			//NUC140VE3xN, Pin19
#define	_SR04A_TRIG		   (GPB_4)			//NUC140VE3xN, Pin34
#define	_SR04A_TRIG_Low	 (GPB_4=0)
#define	_SR04A_TRIG_High (GPB_4=1)

#define PASSWORD_To_Start "SY1945"  // Change this to any desired password
#define MAX_LENGTH 20

#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again

#define Joystick_X_Center 3460 
#define Joystick_Y_Center 3330 
#define Joystick_XY_MAX 4095
#define Joystick_XY_MIN 0 
#define Joystick_bias 50

#define H_Axis_PWM 0
#define HITIME_MIN_Horizontal 16   
#define HITIME_MAX_Horizontal 124 
#define HITIME_MID_Horizontal  70
#define V_Axis_PWM 1
#define HITIME_MIN_Vertical 30   
#define HITIME_MAX_Vertical 118  
#define HITIME_MID_Vertical 72 

const uint8_t VALID_UID[] = {0xb0, 0x12, 0x5d, 0x7c};
int IsLogin = 0;
int IsLevelStr = 0;
int IsGameModeStr = 0;
uint8_t Level = 0;
uint8_t GameMode = 0;
uint32_t Timer_count =0;
uint32_t distance_mm;
unsigned char UID[4],Temp[4];
unsigned char RF_Buffer[18];
unsigned char Password_Buffer[6]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
int out_flag, read_flag;
char TEXT[16];
char TEXT2[16];
char Time[3];
const char *EndTime = "Your time is up, please try again next time\r\n";

volatile uint8_t comRbuf[MAX_LENGTH + 1]; // +1 for null terminator
volatile uint8_t comRbytes = 0;

volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag  = FALSE;


uint8_t HITIME_Vertical, HITIME_Horizontal;


void Init_SPI()
{
	DrvSPI_Open(eDRVSPI_PORT1, eDRVSPI_MASTER, eDRVSPI_TYPE1, 8);
	DrvSPI_SetEndian(eDRVSPI_PORT1, eDRVSPI_MSB_FIRST);
	DrvSPI_DisableAutoSS(eDRVSPI_PORT1);
	DrvSPI_SetClockFreq(eDRVSPI_PORT1, 50000, 0); // set SPI clock = 50KHz
}
// Timer0 initialize to tick every 1s
void InitTIMER0(void)
{
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR0_S = 0;	//Select 12Mhz for Timer0 clock source 
	SYSCLK->APBCLK.TMR0_EN =1;	//Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER0->TCSR.MODE = PERIODIC;		//Select once mode for operation mode

	/* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER0->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER0->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	//Timeout period = (1 / 12MHz) * ( 11 + 1 ) * 1,000,000 = 1 s

	/* Step 4. Enable interrupt */
	TIMER0->TCSR.IE = 1;
	TIMER0->TISR.TIF = 1;		//Write 1 to clear for safty		
	NVIC_EnableIRQ(TMR0_IRQn);	//Enable Timer0 Interrupt - Timer0 interrupt statement

	/* Step 5. Enable Timer module */
	TIMER0->TCSR.CRST = 1;	//Reset up counter
	TIMER0->TCSR.CEN = 0;		//Enable Timer0

}

void Init_TMR2(void)
{	
	//Step 1: T2EX pin Enable (PB.2, Pin34)
	SYS->GPBMFP.UART0_nRTS_nWRL = 1;	
	SYS->ALTMFP.PB2_T2EX = 1;
	
  //Step 2: Timer Controller Reset and Setting Clock Source
	SYS->IPRSTC2.TMR2_RST = 1;          //Timer Controller: Reset
	SYS->IPRSTC2.TMR2_RST = 0;          //Timer Controller: Normal
	SYSCLK->CLKSEL1.TMR2_S = 0;	        //Timer Clock = 12 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;         //Timer C lock Enable

	//Step 3: Timer Controller Setting
	TIMER2->TCMPR = 0xffffff;           //Timer Compare Value:  [0~16777215]
	TIMER2->TCSR.PRESCALE = 11;         //Timer Prescale:       [0~255]
	TIMER2->TCSR.MODE = 0;              //Timer Operation Mode: One-Shot

	//Step 4: External Capture Mode Setting
	TIMER2->TEXCON.TEXEN = 1;	          //External Capture Function Enable
	TIMER2->TEXCON.RSTCAPSEL = 0;	      //Capture Mode Select: Capture Mode
	TIMER2->TEXCON.TEX_EDGE = 2;	      //Capture Edge: Rising & Falling

	//Step 5: Timer Interrupt Setting
	TIMER2->TEXCON.TEXIEN = 1;		      //Capture Interrupt Enable
	TIMER2->u32TEXISR |= 0x01;		      //Clear Capture Flag (TEXIF)
	NVIC_EnableIRQ(TMR2_IRQn);		      //Timer NVIC IRQ Enable

}

// Ultrasonic Trigger
void SR04_Trigger(void)
{
	//Trigger of Ultrasonic Sensor
	_SR04A_TRIG_High;
	DrvSYS_Delay(10);							// 10us for TRIG width
	_SR04A_TRIG_Low;
	
  TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
}

void Init_GPIO_SR04(void)
{
	//Ultrasonic I/O Pins Initial
	GPIOB->PMD.PMD2 = 0;							//_SR04_ECHO pin, Input						
	GPIOB->PMD.PMD4 = 1;              //_SR04_TRIG pin, Output
  _SR04A_TRIG_Low;                  // set Trig output to Low
}

void Choose_Your_Level()
{
		const char *LevelStr = "Choose your level:\nA - 45 seconds to complete the maze\nB - 60 seconds to complete the maze\nC - 75 seconds to complete the maze\r\n";
		if(Level == 0)
		{			
						if(strcmp((char*)comRbuf, "A") == 0)
						{
								Timer_count = 46;
								Level = 1;
								TIMER0->TCSR.CEN = 1;
						}
						else if(strcmp((char*)comRbuf, "B") == 0)
						{
							Timer_count = 61;
							Level = 2;
							TIMER0->TCSR.CEN = 1;

						}
						else if(strcmp((char*)comRbuf, "C") == 0)
						{
							Timer_count = 76;
							Level = 3;
							TIMER0->TCSR.CEN = 1;
						}
						if(!IsLevelStr)
						{
							print_lcd(0,"Choose level:");
							print_lcd(1,"1 - 45 seconds");
							print_lcd(2,"2 - 60 seconds");
							print_lcd(3,"3 - 75 seconds");
							DrvUART_Write(UART_PORT0, (uint8_t*)LevelStr, strlen(LevelStr));
						}
						
						IsLevelStr = 1;

			}
}

void Choose_Game_Mode()
{
		const char *GameModeStr = "Choose the game mode you want to play:\n1 - With joystick\n2 - With your phone\r\n";
		if(GameMode == 0)
		{			
						if(strcmp((char*)comRbuf, "1") == 0)
								GameMode = 1;
						else if(strcmp((char*)comRbuf, "2") == 0)
							GameMode = 2;
						
						if(!IsGameModeStr)
						{
							print_lcd(0,"Choose game mode:");
							print_lcd(1,"1 - Joystick");
							print_lcd(2,"2 - Phone");
							DrvUART_Write(UART_PORT0, (uint8_t*)GameModeStr, strlen(GameModeStr));
						}
						
						IsGameModeStr = 1;
			}
}
void LoginByRFID(void)
{
	if(!IsLogin)
	{
		if(PcdRequest(0x52,Temp)==MI_OK)
		{
				if(PcdAnticoll(UID)==MI_OK)
				{ 
					read_flag =1;
					if(UID[0] == VALID_UID[0]  && UID[1] == VALID_UID[1] && UID[2] == VALID_UID[2] && UID[3] == VALID_UID[3])
					{
						IsLogin = 1;
					}
				}
		} 
	}	  
}

void LoginByPass()
{
		if(!IsLogin)
		{
			const char *GoodPas = "Good Password\r\n";
			const char *BadPas = "Bad Password\r\n";
			if(strcmp((char*)comRbuf, PASSWORD_To_Start) == 0)
			{
					DrvGPIO_ClrBit(E_GPC, 15); // GPC15 pin output Hi to turn off LED
					DrvUART_Write(UART_PORT0, (uint8_t*)GoodPas, strlen(GoodPas));
					IsLogin = 1;
			}
			else
					DrvUART_Write(UART_PORT0, (uint8_t*)BadPas,strlen(BadPas));
		}			
}

void Move_Servo(int angleX, int angleY)
{
		HITIME_Horizontal =  (-angleX / 90.0) * (HITIME_MAX_Horizontal - HITIME_MID_Horizontal) + HITIME_MID_Horizontal;
		HITIME_Vertical   =  (angleY / 90.0) * (HITIME_MAX_Vertical   - HITIME_MID_Vertical)   + HITIME_MID_Vertical;

		PWM_Servo(V_Axis_PWM, HITIME_Vertical);	
		PWM_Servo(H_Axis_PWM, HITIME_Horizontal);	 
}

void IsFinish()
{
	const char *FinishStr = "Congratulations! You have completed the maze\r\n";
		SR04_Trigger();                 // Trigger Ultrasound Sensor for 10us   		
	  DrvSYS_Delay(40000);            // Wait 40ms for Echo to trigger interrupt
		
		if(SR04A_Echo_Flag==TRUE)
		{
			SR04A_Echo_Flag = FALSE;			

			distance_mm = SR04A_Echo_Width * (340/2) / 1000;
		  if(distance_mm < 45) 
			{
					TIMER0->TCSR.CEN = 0;		
					DrvGPIO_ClrBit(E_GPB,11); // GPB11 = 0 to turn on Buzzer
					DrvSYS_Delay(100000);	    // Delay 
					DrvGPIO_SetBit(E_GPB,11); // GPB11 = 1 to turn off Buzzer
					GameMode = 0;
					Level = 0;	
					DrvUART_Write(UART_PORT0, (uint8_t*)FinishStr, strlen(FinishStr));				
			}
		}			
}
void Read_Joystick()
{
	int angleX, angleY;
	uint16_t Vx, Vy;
	DrvADC_StartConvert();                   // start A/D conversion
  while(DrvADC_IsConversionDone()==FALSE); // wait till conversion is done
	Vx = ADC->ADDR[0].RSLT & 0xFFF;
	Vy = ADC->ADDR[1].RSLT & 0xFFF;

	if(Vx > Joystick_X_Center + Joystick_bias) // left
			angleX = (Vx -(Joystick_X_Center + Joystick_bias )) * 15 / (Joystick_XY_MAX - (Joystick_X_Center + Joystick_bias));
	else if (Vx < Joystick_X_Center - Joystick_bias) // right
			angleX = (Vx -(Joystick_X_Center - Joystick_bias )) * 10 / (Joystick_X_Center - Joystick_bias - Joystick_XY_MIN);
	else
			angleX = 0;
	
	if(Vy < Joystick_Y_Center - Joystick_bias) // up
			angleY = (Vy -(Joystick_Y_Center + Joystick_bias)) * 8 / (Joystick_Y_Center - Joystick_bias - Joystick_XY_MIN);
	else if (Vy > Joystick_Y_Center + Joystick_bias) // down
			angleY = (Vy -(Joystick_Y_Center - Joystick_bias)) * 20 / (Joystick_XY_MAX - (Joystick_Y_Center + Joystick_bias));
	else
			angleY = 0;

	Move_Servo(angleX, angleY);
	IsFinish();	
}

void READ_BT_PAD()
{
	
						if(strcmp((char*)comRbuf, "U") == 0)
						{
								Move_Servo(0, 20);		     
								DrvSYS_Delay(1000000);
						}
						else if(strcmp((char*)comRbuf, "D") == 0)
						{
								Move_Servo(0, -8);		
								DrvSYS_Delay(1000000);
						}
						else if(strcmp((char*)comRbuf, "L") == 0)
						{
								Move_Servo(15, 0);	
								DrvSYS_Delay(1000000);
						}
						else if(strcmp((char*)comRbuf, "R") == 0)
						{
								Move_Servo(-10, 0);	
								DrvSYS_Delay(1000000);
						}
						else
							  Move_Servo(0, 0);
					  IsFinish();  						
} 
// UART0 Interrupt Handler
void UART0_INT_HANDLE(uint32_t u32UserData)
{
	char receivedByte;
    // While there is data in the UART0 RX FIFO
    while (UART0->ISR.RDA_IF == 1) 
    {
        // Read one byte
        receivedByte = UART0->DATA;
			  if (comRbytes < MAX_LENGTH) 
        {
					 comRbuf[comRbytes] = receivedByte;
           comRbytes++;
				}
				 // If newline (\r or \n) is received, process the command
          // If '#' is received, process the command
        if (receivedByte == '#') 
        {
            comRbuf[comRbytes - 1] = '\0'; // Null-terminate the string
           if (!IsLogin)
					 {
               LoginByPass();
					 }						 
						memset((char*)comRbuf, 0, sizeof(comRbuf));
						comRbytes = 0;
        }
				
     }
}

void TMR0_IRQHandler(void) // Timer0 interrupt subroutine 
{
		Timer_count--;
		clr_all_panel();
	  sprintf(Time,"      %02d:%02d      ", Timer_count / 60, Timer_count % 60);
		print_lcd(1, Time);	
		if(Timer_count == 0)
		{
			DrvUART_Write(UART_PORT0, (uint8_t*)EndTime, strlen(EndTime));
			TIMER0->TCSR.CEN = 0;		//Disable Timer0
			GameMode = 0;
			Level = 0;
		}
    TIMER0->TISR.TIF = 1;	   
}

// TMR2 Interrupt Handler
void TMR2_IRQHandler(void)
{
	TIMER2->TEXCON.RSTCAPSEL = 0;       // set back for falling edge to capture
	TIMER2->TCSR.CEN = 1;					      //Timer Start

	if(TIMER2->TEXISR.TEXIF == 1)	      //Capture Flag (TEXIF)
	{
	 	TIMER2->u32TEXISR |= 0x01;				//Clear Capture Flag (TEXIF)
		SR04A_Echo_Width = TIMER2->TCAP;	//Load Capture Value (Unit: us)
		SR04A_Echo_Flag  = TRUE;
	}
}
void Set_All_Initials()
{
	STR_UART_T sParam;
	UNLOCKREG();
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, 1); // Enable the 12MHz oscillator oscillation
  DrvSYS_SelectHCLKSource(0); // HCLK clock source. 0: external 12MHz; 4:internal 22MHz RC oscillator
	//System clock @ 48 MHz
  DrvSYS_Open(48000000);
	DrvSYS_Delay(5000); // wait till 12MHz crystal stable
	LOCKREG();
	
	
	DrvSYS_SetClockDivider(E_SYS_HCLK_DIV, 0); /* HCLK clock frequency = HCLK clock source / (HCLK_N + 1) */
	DrvGPIO_InitFunction(E_FUNC_SPI1);
	
	Init_SPI();
	InitTIMER0();
	Initial_panel(); 
	clr_all_panel();
	Init_TMR2();                      // initialize Timer2 Capture
  Init_GPIO_SR04();
	InitPWM(H_Axis_PWM);// initialize PWM0 Horizontal (Right - Left)
	InitPWM(V_Axis_PWM); // initialize PWM1 Vertical	(Up - Down)
	Move_Servo(0, 0);
	
	PcdReset();
	PcdAntennaOn();
	
  // UART0 pins: GPB0=RXD0, GPB1=TXD0
  DrvGPIO_InitFunction(E_FUNC_UART0);
  DrvADC_Open(ADC_SINGLE_END, ADC_SINGLE_CYCLE_OP, 0x03, INTERNAL_HCLK, 1); // ADC1 (GPA1) & ADC0 (GPA0)		
  // UART0 Config
  sParam.u32BaudRate        = 9600;
  sParam.u8cDataBits        = DRVUART_DATABITS_8;
  sParam.u8cStopBits        = DRVUART_STOPBITS_1;
  sParam.u8cParity          = DRVUART_PARITY_NONE;
  sParam.u8cRxTriggerLevel  = DRVUART_FIFO_1BYTES;

  // Open UART0 and enable RX interrupt
  if (DrvUART_Open(UART_PORT0, &sParam) != E_SUCCESS)
  {
        // Handle error if needed
  }
  DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART0_INT_HANDLE);
}



int main(void)
{
	Set_All_Initials();
	while(1)
	{
		while (!IsLogin) // if(IsLogin == 0)
		{
			LoginByRFID();  // call function for reading rfid tag
		}		
		if(GameMode == 0)
		{
				Choose_Game_Mode();
		}
		if (GameMode != 0)
		{
				if(Level ==0)
				{
					Choose_Your_Level();
				}
				if(Level !=0)
				{
					  while(GameMode == 1)
						{
								Read_Joystick();
						}
						 while(GameMode == 2)
						{
								READ_BT_PAD();
						}
				}
		}
	}
}
