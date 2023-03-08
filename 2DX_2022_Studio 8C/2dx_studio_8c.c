/*  
	Marcus Cohoon, cohoom1, 400297985 2DX3 Final Project Code
*/
#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                        						// 8) configure for 100 kbps clock
        
}

void PortN_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; //activate the clock for Port N
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};//allow time for clock to stabilize
GPIO_PORTN_DIR_R=0b00000010; //Make PN1 outputs, to turn on LED's
GPIO_PORTN_DEN_R=0b00000010; //Enable PN1
return;
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}
void PortH_Init(void){
  //Use PortM pins for output
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                // activate clock for Port H
  while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};    // allow time for clock to stabilize
  GPIO_PORTH_DIR_R |= 0xFF;                                        // make PH0 out (PN0 built-in LED1)
  GPIO_PORTH_AFSEL_R &= ~0xFF;                                     // disable alt funct on PH0
  GPIO_PORTH_DEN_R |= 0xFF;                                        // enable digital I/O on PH0
                                                                                                    // configure PN1 as GPIO
  
  GPIO_PORTH_AMSEL_R &= ~0xFF;                                     // disable analog functionality on PH0        
  return;
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long FallingEdges = 0;

// give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;     										// enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	enable weak pull up resistor
}


// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
	
		FallingEdges = 0;       // initialize counter

	
		GPIO_PORTJ_IS_R = 0;    	// (Step 1) PJ1 is edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;   	//     			PJ1 is not both edges 
		GPIO_PORTJ_IEV_R = 0;    	//     			PJ1 falling edge event 
		GPIO_PORTJ_ICR_R = 0x02;     // 					clear interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x02;    	// 					arm interrupt on PJ1 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;           // (Step 2) enable interrupt 51 in NVIC
	
		NVIC_PRI12_R = 0xA0000000; 				// (Step 4) set interrupt priority 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}




//	(Step 5) IRQ Handler (Interrupt Service Routine).  
//  				This must be included and match interrupt naming convention
//	 				in startup_msp432e401y_uvision.s 
//					(Note - not the same as Valvano textbook).
void GPIOJ_IRQHandler(void){
  FallingEdges = FallingEdges + 1;	// Increase the global counter variable ;Observe in Debug Watch Window
	GPIO_PORTJ_ICR_R = 0x02;     					// acknowledge flag by setting proper bit in ICR register
}


//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}
void spin(){
    for(int i=0; i<32; i++){
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(4);
        GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(4);
    }
	}
void rev_spin(){
    for(int i=0; i<512; i++){
         GPIO_PORTH_DATA_R = 0b00001001;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00000011;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00000110;
        SysTick_Wait10ms(5);
        GPIO_PORTH_DATA_R = 0b00001100;
        SysTick_Wait10ms(5);
    }
	}

//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	//initialize
	PLL_Init();	
	SysTick_Init();
	PortH_Init();
	I2C_Init();
	UART_Init();
	PortN_Init();					// Initialize the onboard LED on port N
	PortJ_Init();											// Initialize the onboard push button on PJ1
	PortJ_Interrupt_Init();	
	
	
	while(1){
		//wait for interupt from button to start
		WaitForInt();
		// Booting ToF chip
		while(sensorState==0){
			status = VL53L1X_BootState(dev, &sensorState);
			SysTick_Wait10ms(10);
		}

		status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
		/* This function must to be called to initialize the sensor with the default setting  */
		status = VL53L1X_SensorInit(dev);
		Status_Check("SensorInit", status);

		
		/* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

		status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

		
		//16 scans
		for(int i = 0; i < 16; i++) {
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);
			

			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			
			// print the resulted readings to UART
			sprintf(printf_buffer,"%u\r\n", Distance);
			UART_printf(printf_buffer);
			GPIO_PORTN_DATA_R = 0b00000010;
			SysTick_Wait10ms(30);
			GPIO_PORTN_DATA_R = 0b00000000;
			spin();
			SysTick_Wait10ms(150);
		}
		rev_spin();
		VL53L1X_StopRanging(dev);
	}
}

