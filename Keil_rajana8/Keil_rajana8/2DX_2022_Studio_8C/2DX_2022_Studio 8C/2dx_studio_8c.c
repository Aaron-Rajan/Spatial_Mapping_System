//	Aaron Rajan
//	400321812
//	rajana8

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"

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

//	Initialized I2C for communication between microcontroller and ToF sensor
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		
    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only
    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
                                                                            // 6) configure PB2,3 as I2C
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    				//TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
// Initialized Port G for ToF sensor
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                			// activate clock for Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    					// allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                     // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                    // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                      	// enable digital I/O on PG0
																																	// configure PG0 as GPIO
  GPIO_PORTG_AMSEL_R &= ~0x01;                                    // disable analog functionality on PG0

    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                     // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                             	//PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                   	// make PG0 input (HiZ)
    
}

//	Initialized Port J for onboard input button
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                    	// activate clock for Port J
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};    					// allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;                                    	// make PJ1 in 
  GPIO_PORTJ_DEN_R |= 0x02;                                     	// enable digital I/O on PJ1
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;                           	//  configure PJ1 as GPIO 
    GPIO_PORTJ_AMSEL_R &= ~0x02;                                	//  disable analog functionality on PJ1        
    GPIO_PORTJ_PUR_R |= 0x02;                                   	//  enable weak pull up resistor
}

// Initialized Port N1 for LED 1
void PortN1_Init(void){																						
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; 											//activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};							//allow time for clock to stabilize
	GPIO_PORTN_DIR_R=0b00000010; 																		//Make PN1 output, to turn on LED's
	GPIO_PORTN_DEN_R=0b00000010; 																		//Enable PN1
	return;
}

//	Initialized Port L for Waveforms
void PortL_Init(void){			
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10;												// activate clock for Port L
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){};							// allow time for clock to stabilize
	GPIO_PORTL_DIR_R &= 0x04;        																// make PL2 out 
	GPIO_PORTL_DIR_R |= 0x04; 
  GPIO_PORTL_AFSEL_R &= ~0x07;     																// disable alt funct on PL2
  GPIO_PORTL_DEN_R |= 0x07;        																// enable digital I/O on PL2
																																	// configure PL2 as GPIO
  GPIO_PORTL_AMSEL_R &= ~0x07;     																// disable analog functionality on PL2	
	return;
}

// Initialized Port M for stepper motor
void PortM_Init(void){				
	//Use PortM pins for output
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;												// similar initialize process to previous init functions, except different registers corresponding to Port M are used
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};							
	GPIO_PORTM_DIR_R |= 0xFF;        								
  GPIO_PORTM_AFSEL_R &= ~0xFF;     								
  GPIO_PORTM_DEN_R |= 0xFF;        								
  GPIO_PORTM_AMSEL_R &= ~0xFF;     									
	return;
}

void spin(int direction){
	// Created for loop to spin motor 5.625 degrees each turn, for a total of 5.625 * 64 = 360 degrees
    for(int i=0; i<8; i++){
			if(direction == 1) { //Counter-clockwise 
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1); // Delay sets speed for motor
        GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
      }
      else if(direction == 0) { //Clockwise
        GPIO_PORTM_DATA_R = 0b00001001;
        SysTick_Wait10ms(1);
        GPIO_PORTM_DATA_R = 0b00000011;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00000110;
        SysTick_Wait10ms(1);
				GPIO_PORTM_DATA_R = 0b00001100;
        SysTick_Wait10ms(1);
      }
      else {
        GPIO_PORTH_DATA_R = 0b00000000;
        SysTick_Wait10ms(1);
			}        
    }
        GPIO_PORTM_DATA_R = 0b00000000;
}

//*********************************************************************************************************
//*********************************************************************************************************
//**************************************					MAIN Function			***************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
	//	Initialized variables for ToF sensor
  uint8_t sensorState=0;
  uint16_t wordData;
  uint16_t Distance; 
  uint8_t dataReady;

	//	Initialized GPIO ports, system clock, I2C, UART
	PortJ_Init();
	PortL_Init();
	PortM_Init();
	PortN1_Init();
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
	//	I2C communicates between microcontroller and ToF sensor
	status = VL53L1X_GetSensorId(dev, &wordData);
	
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Getting ToF chip ready
	while(sensorState==0) {
		// Status code lines transmit UART data to microcontroller
		//	UART communicates between Python and microcontroller
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
	}
	
	FlashAllLEDs();
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
	/* This function must to be called to initialize the sensor with the default setting  */
	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);
	
	//	For Waveforms, uncommented when the bus speed needs to be checked with Waveforms
//	for(int i = 0; i<24000000; i++){
//    GPIO_PORTL_DATA_R ^= 0b00000100;
//		SysTick_Wait(500);
//   }
	
// Theoretical time for 1,000 cycles with a bus speed of 30MHz = 1,000/30,000,000 * 10^6
	
		while(1){
			status = VL53L1X_StartRanging(dev);   														//This function has to be called to enable the ranging
			Again:
				if((GPIO_PORTJ_DATA_R&0b00000010) == 0){
				
					for(int i = 0; i < 64; i++) {																		//	Getting the distance measurement 64 times
						int degrees = 5.625*i;
		
						while (dataReady == 0){																				//wait until the ToF sensor's data is ready
							status = VL53L1X_CheckForDataReady(dev, &dataReady);
							FlashLED3(1);
							VL53L1_WaitMs(dev, 5);
						}
						dataReady = 0;
	  
																																				//read the data values from ToF sensor
						status = VL53L1X_GetDistance(dev, &Distance);								//The Measured Distance value
					
						FlashLED1(1);
						
						status = VL53L1X_ClearInterrupt(dev); 											//clear interrupt has to be called to enable next interrupt
						double displacement = Distance;
																																				// print the resulted readings to UART
						sprintf(printf_buffer,"%f\r\n", displacement/1000); //distance is divided by 1,000 to get to m
						UART_printf(printf_buffer); //UART is 8 bits, communicates distance in m to PC
					
						spin(0); //Spins clockwise
						SysTick_Wait10ms(1);
						if((GPIO_PORTJ_DATA_R&0b00000010) == 0) { //	Checking if push button is pressed (Active low configuration)
                    for(int j = 0; j < i; j++) {
                        spin(1); //	Incriminanting back the amount the motor has spun
                    }
                    goto Again; //	If J1 button is held, the system returns to intial state (Motor spins back to start)
                }
					}
			//	Created for loop to spin back to original state
			for(int i = 0; i<64;i++){
				spin(1);
			}
			VL53L1X_StopRanging(dev);
			
			}
		}
}

