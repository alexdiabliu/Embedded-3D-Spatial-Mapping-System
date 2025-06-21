
#include "uart.h"
#include "tm4c1294ncpdt.h"
#include <stdint.h>
#include <stdio.h>
#include "onboardLEDs.h"        // Functions for controlling onboard LEDs


//Initialize UART0, based on textbook.  Clock code modified.



// ------------------ UART COMMUNICATION -------------------
// UART = Universal Asynchronous Receiver-Transmitter
// Used to send data serially from the microcontroller to a PC

// How we use UART in this project:
// - Microcontroller sends 3D scan data (distance + angle info) to PC via UART
// - PC visualizes this data (e.g., in Python or MATLAB)

// UART Setup Considerations:
// - Configure Baud Rate (e.g., 9600, 115200), Data Bits, Parity, and Stop Bits
// - Add Start and End markers if needed for data framing
// - Use status LED to indicate UART transmission blocks

// Benefit:
// - Enables real-time transmission and display of spatial mapping results
// ----------------------------------------------------------

void UART_Init(void) {
	SYSCTL_RCGCUART_R |= 0x0001; // activate UART0   
	SYSCTL_RCGCGPIO_R |= 0x0001; // activate port A   
	//UART0_CTL_R &= ~0x0001; // disable UART   

	while((SYSCTL_PRUART_R&SYSCTL_PRUART_R0) == 0){};
		
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 8;                     // IBRD = int(16,000,000 / (16 * 115,200)) = int(8.681)
  UART0_FBRD_R = 44;                    // FBRD = round(0.6806 * 64) = 44
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
                                        // UART gets its clock from the alternate clock source as defined by SYSCTL_ALTCLKCFG_R
  UART0_CC_R = (UART0_CC_R&~UART_CC_CS_M)+UART_CC_CS_PIOSC;
                                        // the alternate clock source is the PIOSC (default)
  SYSCTL_ALTCLKCFG_R = (SYSCTL_ALTCLKCFG_R&~SYSCTL_ALTCLKCFG_ALTCLK_M)+SYSCTL_ALTCLKCFG_ALTCLK_PIOSC;
  UART0_CTL_R &= ~UART_CTL_HSE;         // high-speed disable; divide clock by 16 rather than 8 (default)

	UART0_LCRH_R = 0x0070;		// 8-bit word length, enable FIFO   
	UART0_CTL_R = 0x0301;			// enable RXE, TXE and UART   
	GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011; // UART   
	GPIO_PORTA_AMSEL_R &= ~0x03;	// disable analog function on PA1-0   
	GPIO_PORTA_AFSEL_R |= 0x03;		// enable alt funct on PA1-0   
	GPIO_PORTA_DEN_R |= 0x03;			// enable digital I/O on PA1-0 
}

// Wait for new input, then return ASCII code 
	char UART_InChar(void){
		while((UART0_FR_R&0x0010) != 0);		// wait until RXFE is 0   
		return((char)(UART0_DR_R&0xFF));
	} 
	
	// Wait for buffer to be not full, then output 
	void UART_OutChar(char data){
		FlashLED3(1);
		while((UART0_FR_R&0x0020) != 0);	// wait until TXFF is 0   
		UART0_DR_R = data;
	} 
	void UART_printf(const char* array){
		FlashLED3(1);
		int ptr=0;
		while(array[ptr]){
			UART_OutChar(array[ptr]);
			ptr++;
		}
	}
	
	void Status_Check(char* array, int status){
			FlashLED3(1);
			if (status != 0){
				UART_printf(array);
				sprintf(printf_buffer," failed with (%d)\r\n",status);
				UART_printf(printf_buffer);
			}else
			{
				UART_printf(array);
				UART_printf(" Successful.\r\n");
			}
	}

	