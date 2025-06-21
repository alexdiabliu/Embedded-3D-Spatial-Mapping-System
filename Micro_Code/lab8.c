// Include standard and project-specific header files
#include <stdint.h>             // Standard integer types
#include "PLL.h"                // Phase-Locked Loop configuration (sets system clock)
#include "SysTick.h"            // SysTick timer for delays and timing control
#include "uart.h"               // UART communication functions
#include "onboardLEDs.h"        // Functions for controlling onboard LEDs
#include "tm4c1294ncpdt.h"      // Register definitions for the TM4C1294 microcontroller
#include "VL53L1X_api.h"        // API for VL53L1X Time-of-Flight sensor
#include <math.h>               // Math library (for trigonometric functions, etc.)

// I2C Master Control/Status Register Bit Definitions
#define I2C_MCS_ACK     0x00000008  // Acknowledge bit
#define I2C_MCS_DATACK  0x00000008  // Data Acknowledge
#define I2C_MCS_ADRACK  0x00000004  // Address Acknowledge
#define I2C_MCS_STOP    0x00000004  // Generate STOP condition
#define I2C_MCS_START   0x00000002  // Generate START condition
#define I2C_MCS_ERROR   0x00000002  // Error flag
#define I2C_MCS_RUN     0x00000001  // Enable Master operation
#define I2C_MCS_BUSY    0x00000001  // I2C bus busy
#define I2C_MCR_MFE     0x00000010  // I2C Master Function Enable

#define MAXRETRIES      5           // Maximum number of retries for I2C communication

// Global variables
int full = 0;           // Flag indicating whether a full scan or rotation is complete

// Variables to track repeated actions or loops
uint8_t times = 1;         // General-purpose counter (e.g., number of scans)
uint8_t times2 = 1;        // Another counter, likely for nested or separate loops


// Initializes I2C0 module and configures PB2 (SCL) and PB3 (SDA) for I2C communication
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;         // Enable clock for I2C0 module
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;       // Enable clock for GPIO Port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};        // Wait until Port B is ready

  GPIO_PORTB_AFSEL_R |= 0x0C;                    // Enable alternate functions on PB2 (SCL) and PB3 (SDA)
  GPIO_PORTB_ODR_R |= 0x08;                      // Enable open-drain for PB3 (SDA)

  GPIO_PORTB_DEN_R |= 0x0C;                      // Digital enable for PB2 and PB3
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200; // Configure PB2 & PB3 as I2C
  I2C0_MCR_R = I2C_MCR_MFE;                      // Enable I2C master function
  I2C0_MTPR_R = 0b0000000000000101000000000111011; // Set I2C clock period (based on bus speed from spec)
}
// ------------------- I2C COMMUNICATION -------------------
// I2C = Inter-Integrated Circuit, a 2-wire serial protocol (SDA for data, SCL for clock).
// Used to communicate between microcontroller and VL53L1X sensor.

// Why I2C?
// - Simple and efficient for low-speed communication
// - Supports multiple devices on the same bus (multi-slave)

// How we use I2C in this project:
// - VL53L1X sensor connects via SDA/SCL lines to microcontroller (e.g., TM4C1294NCPDT).
// - Microcontroller (master) sends commands to sensor (slave) to:
//    -> Start/stop measurements
//    -> Select distance mode (short/med/long)
//    -> Read distance data (in mm)

// I2C Protocol Details:
// - Master-Slave relationship: Microcontroller = master, VL53L1X = slave
// - Sensor default address: 0x52 (7-bit format)
// - Max I2C speed: 400 kHz (Fast Mode), but lower speeds may be used for stability

// ----------------------------------------------------------



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


// Initializes Port G, specifically for controlling the XSHUT pin of the VL53L1X sensor
void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;     // Enable clock for Port G
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){}; // Wait until Port G is ready
    GPIO_PORTG_DIR_R &= 0x00;                    // Configure PG0 as input (default)
    GPIO_PORTG_AFSEL_R &= ~0x01;                 // Disable alternate functions on PG0
    GPIO_PORTG_DEN_R |= 0x01;                    // Digital enable for PG0
    GPIO_PORTG_AMSEL_R &= ~0x01;                 // Disable analog function on PG0
    return;
}

// Resets the VL53L1X sensor by toggling XSHUT on PG0
void VL53L1X_XSHUT(void)
{
    GPIO_PORTG_DIR_R |= 0x01;                    // Set PG0 as output
    GPIO_PORTG_DATA_R &= 0b11111110;             // Pull PG0 low (XSHUT = 0), turn off sensor
    FlashAllLEDs();                               // Visual indicator (likely for debug)
    SysTick_Wait10ms(10);                         // Delay 100 ms
    GPIO_PORTG_DIR_R &= ~0x01;                   // Set PG0 back to input (XSHUT = Hi-Z)
}

// Initializes Port E: PE0 and PE1 as digital outputs (used for control or debugging)
void PortE_Init(void){  
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;     // Enable clock for Port E
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R4) == 0){}; // Wait until Port E is ready
    GPIO_PORTE_DIR_R = 0b00000011;               // Set PE0 and PE1 as outputs
    GPIO_PORTE_DEN_R = 0b00000011;               // Digital enable for PE0 and PE1
    return;
}

// Initializes Port M for buttons and possibly other control functions
void PortM_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;    // Enable clock for Port M
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){}; // Wait until Port M is ready
    GPIO_PORTM_DIR_R |= 0b00001111;              // PM0–PM3 as outputs
    GPIO_PORTM_AFSEL_R &= ~0x0F;                 // Disable alternate functions on PM0–PM3
    GPIO_PORTM_DEN_R |= 0x0F;                    // Digital enable PM0–PM3
    GPIO_PORTM_AMSEL_R &= ~0x0F;                 // Disable analog on PM0–PM3
    GPIO_PORTM_PUR_R |= 0x03;                    // Enable pull-ups on PM0 and PM1 (likely for buttons)
    return;
}

// Initializes Port H for stepper motor control (PH0–PH3 as digital outputs)
void PortH_Init(void)
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;     // Enable clock for Port H
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; // Wait until Port H is ready
    GPIO_PORTH_DIR_R |= 0x0F;                    // Set PH0–PH3 as outputs
    GPIO_PORTH_AFSEL_R &= ~0x0F;                 // Disable alternate functions
    GPIO_PORTH_DEN_R |= 0x0F;                    // Digital enable PH0–PH3
    GPIO_PORTH_AMSEL_R &= ~0x0F;                 // Disable analog functions
    return;
}

// Initializes Port N, commonly used for onboard LEDs or status outputs
void PortN_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;    // Enable clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){}; // Wait until Port N is ready
    GPIO_PORTN_DIR_R = 0b00000011;               // Set PN0 and PN1 as outputs
    GPIO_PORTN_DEN_R = 0b00000011;               // Digital enable PN0 and PN1
    return;
}

// Initializes Port F, often used for status LEDs
void PortF_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;     // Enable clock for Port F
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){}; // Wait until Port F is ready
    GPIO_PORTF_DIR_R = 0b00010001;               // Set PF0 and PF4 as outputs
    GPIO_PORTF_DEN_R = 0b00010001;               // Digital enable PF0 and PF4
    return;
}

// Initializes Port J for input buttons (e.g., Start/Stop)
void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;     // Enable clock for Port J
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){}; // Wait until Port J is ready
    GPIO_PORTJ_DIR_R &= ~0x03;                   // Set PJ0 and PJ1 as inputs
    GPIO_PORTJ_DEN_R |= 0x03;                    // Digital enable PJ0 and PJ1
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;            // Clear PCTL for PJ0 and PJ1
    GPIO_PORTJ_AMSEL_R &= ~0x03;                 // Disable analog on PJ0 and PJ1
    GPIO_PORTJ_PUR_R |= 0x03;                    // Enable pull-ups (for button inputs)
}

// Controls LED D1 connected to PN1
// order = 1 ? turn ON, order = 0 ? turn OFF
void D1(uint8_t order)
{
    if(order == 0)
    {
        GPIO_PORTN_DATA_R &= 0b11111101; // Clear PN1 (turn off LED)
    }
    if(order == 1)
    {
        GPIO_PORTN_DATA_R |= 0b00000010; // Set PN1 (turn on LED)
    }
}

// Controls LED D2 connected to PN0
void D2(uint8_t order)
{
    if(order == 0)
    {
        GPIO_PORTN_DATA_R &= 0b11111110; // Clear PN0
    }
    if(order == 1)
    {
        GPIO_PORTN_DATA_R |= 0b00000001; // Set PN0
    }
}

// Controls LED D3 connected to PF4
void D3(uint8_t order)
{
    if(order == 0)
    {
        GPIO_PORTF_DATA_R &= 0b11101111; // Clear PF4
    }
    if(order == 1)
    {
        GPIO_PORTF_DATA_R |= 0b00010000; // Set PF4
    }
}

// Controls LED D4 connected to PF0
void D4(uint8_t order)
{
    if(order == 0)
    {
        GPIO_PORTF_DATA_R &= 0b11111110; // Clear PF0
    }
    if(order == 1)
    {
        GPIO_PORTF_DATA_R |= 0b00000001; // Set PF0
    }
}

// Reads push button B0 on PJ0
// Returns 1 if pressed, 0 if not
uint8_t is_b0_pressed()
{
    if((GPIO_PORTJ_DATA_R & 0b1) == 0b1) // Button not pressed (logic high)
    {
        return 0b0;
    }

    if((GPIO_PORTJ_DATA_R & 0b1) == 0b0) // Button pressed (logic low due to pull-up)
    {
        return 0b1;
    }

    return 0b0; // Default return
}

// Reads push button B1 on PJ1
uint8_t is_b1_pressed()
{
    if((GPIO_PORTJ_DATA_R & 0b10) == 0b10)
    {
        return 0; // Not pressed
    }

    if((GPIO_PORTJ_DATA_R & 0b10) == 0b00)
    {
        return 1; // Pressed
    }

    return 0b0;
}

// Reads push button B3 on PM0
uint8_t is_b3_pressed()
{
    if((GPIO_PORTM_DATA_R & 0b1) == 0b1)
    {
        return 0; // Not pressed
    }

    if((GPIO_PORTM_DATA_R & 0b1) == 0b0)
    {
        return 1; // Pressed
    }

    return 0b0;
}

// Reads push button B2 on PM1
uint8_t is_b2_pressed()
{
    if((GPIO_PORTM_DATA_R & 0b10) == 0b10)
    {
        return 0; // Not pressed
    }

    if((GPIO_PORTM_DATA_R & 0b10) == 0b00)
    {
        return 1; // Pressed
    }

    return 0b0;
}

// Briefly flashes D4 LED for ~8ms (used as a signal or pulse)
void flashd4(void) 
{
    D4(1);                       // Turn on D4
    SysTick_Wait10us(800);      // Wait 800 × 10µs = 8000µs = 8ms
    D4(0);                       // Turn off D4
}


uint16_t dev = 0x29;
int status = 0;


int main(void) 
{
  // === VARIABLE DECLARATIONS ===

  // ToF sensor communication and buffer variables
  uint8_t byteData, sensorState = 0;
  uint8_t myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint16_t wordData;
  uint16_t Distance, SignalRate, AmbientRate, SpadNum; // ToF measurement outputs
  uint8_t RangeStatus, dataReady;
  uint8_t model_ID, model_Type; // Identifiers from the VL53L1X

  // === INITIALIZATION ===

  PLL_Init();               // Set up system clock
  SysTick_Init();           // Set up SysTick timer
  onboardLEDs_Init();       // Initialize onboard status LEDs
  I2C_Init();               // Initialize I2C communication
  UART_Init();              // Initialize UART for serial communication
  PortE_Init();             // Motor or debug control pins
  PortM_Init();             // User buttons (PM0, PM1)
  PortN_Init();             // Status LEDs (D1, D2)
  PortF_Init();             // More status LEDs (D3, D4)
  PortJ_Init();             // Onboard buttons (PJ0, PJ1)
  PortH_Init();             // Stepper motor control port
	
	
	// === Clock Configuration Debug Printout ===

	uint32_t clkcfg = SYSCTL_RSCLKCFG_R;
	uint32_t pll0 = SYSCTL_PLLFREQ0_R;
	uint32_t pll1 = SYSCTL_PLLFREQ1_R;
	uint32_t pllstat = SYSCTL_PLLSTAT_R;

	sprintf(printf_buffer, "Clock Registers:\r\n");
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "RSCLKCFG  = 0x%08X\r\n", clkcfg);
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "PLLFREQ0  = 0x%08X\r\n", pll0);
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "PLLFREQ1  = 0x%08X\r\n", pll1);
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "PLLSTAT   = 0x%08X\r\n", pllstat);
	UART_printf(printf_buffer);

	// Extract PLL settings
	uint32_t MINT = pll0 & 0x3FF;
	uint32_t MFRAC = (pll0 >> 10) & 0x3FF;
	uint32_t N = pll1 & 0x1F;
	uint32_t Q = (pll1 >> 8) & 0x1F;
	uint32_t sysdiv_val = clkcfg & 0x3FF;

	// Calculate derived frequencies
	float f_xtal = 25.0f;
	float f_vco = (f_xtal / (Q + 1) / (N + 1)) * (MINT + (MFRAC / 1024.0f));
	float sysclk = f_vco / (sysdiv_val + 1);

	sprintf(printf_buffer, 
		"MINT = %u, MFRAC = %u, N = %u, Q = %u, PSYSDIV = %u\r\n",
		MINT, MFRAC, N, Q, sysdiv_val);
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "Estimated VCO Frequency = %.2f MHz\r\n", f_vco);
	UART_printf(printf_buffer);

	sprintf(printf_buffer, "Estimated System Clock = %.2f MHz\r\n", sysclk);
	UART_printf(printf_buffer);


  // === STATUS MESSAGE ===

  UART_printf("Program Begins\r\n");
  int mynumber = 1;
  sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n", mynumber);
  UART_printf(printf_buffer);

  // === SENSOR IDENTIFICATION ===

  status = VL53L1_RdByte(dev, 0x010F, &model_ID);     // Read model ID
  status = VL53L1_RdByte(dev, 0x0110, &model_Type);   // Read model type
  status = VL53L1_RdWord(dev, 0x010F, &wordData);     // Read both as one word

  //sprintf(printf_buffer,"Model ID is 0x%x\r\n", model_ID); UART_printf(printf_buffer);
  //sprintf(printf_buffer,"Model Type is 0x%x\r\n", model_Type); UART_printf(printf_buffer);
  //sprintf(printf_buffer,"[Model ID, Model Type] is 0x%x\r\n", wordData); UART_printf(printf_buffer);

  float z = 0;  // Vertical height

  uint32_t delay = 400;  // Stepper motor delay (in 10us units = 4ms)
  float angle = 0;       // Tracks rotational angle from 0 to 360°
	int total_count = 0;
	
  // === OUTER LOOP: REPEAT ENTIRE SCAN SESSION IF times2 == 1 ===
  while(times2)
  {		

    // === Wait for ToF sensor to finish booting ===
    while(sensorState == 0)
    {
      status = VL53L1X_BootState(dev, &sensorState);
			// === Sensor setup ===
			status = VL53L1X_ClearInterrupt(dev);         // Clear any existing interrupts
			status = VL53L1X_SensorInit(dev);             // Initialize sensor settings
			Status_Check("SensorInit", status);           // Print result
			status = VL53L1X_StartRanging(dev);           // Begin continuous ranging
			sprintf(printf_buffer,"Status is 0x%x\r\n", status); UART_printf(printf_buffer);
      SysTick_Wait10ms(10);
    }


    // === INNER LOOP: 360° SCAN CYCLE ===
    while(times)
    {
			D1(0);
			D4(1);
      // Rotate 11.25° (16 motor steps)
      for(int i = 0; i < 16; i++)
      {
        GPIO_PORTM_DATA_R = 0b00000011; SysTick_Wait10us(delay);
        GPIO_PORTM_DATA_R = 0b00001001; SysTick_Wait10us(delay);
        GPIO_PORTM_DATA_R = 0b00001100; SysTick_Wait10us(delay);
        GPIO_PORTM_DATA_R = 0b00000110; SysTick_Wait10us(delay);
      }

      angle += 11.25; // Track current rotation angle

      // Wait for ToF sensor to have data ready
      while (dataReady == 0)
      {
        status = VL53L1X_CheckForDataReady(dev, &dataReady);
        VL53L1_WaitMs(dev, 5);
      }

      dataReady = 1; // Reset

      // Retrieve distance and other data from sensor
      status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
      status = VL53L1X_GetDistance(dev, &Distance);
      status = VL53L1X_GetSignalRate(dev, &SignalRate);
      status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
      status = VL53L1X_GetSpadNb(dev, &SpadNum);


      status = VL53L1X_ClearInterrupt(dev); // Clear interrupt for next reading

      // polar to Cartesian (X, Y)
      float x, y;
      float rad = angle * 3.1415926 / 180.0;  // Convert angle to radians
      x = Distance * cos(rad);
      y = Distance * sin(rad);

      // to serial monitor
      sprintf(printf_buffer,"[%f, %f, %f] \r\n", x, y, z);
      UART_printf(printf_buffer);

      if(angle == 360)
      {
				D4(0);
        // Delay before rewinding
        SysTick_Wait10us(delay);
				//SysTick_Wait10us(750000);
				
				
        // Rewind stepper motor (512 steps = 1 full revolution)
        for(int i = 0; i < 512; i++)
        {
          GPIO_PORTM_DATA_R = 0b00000011; SysTick_Wait10us(delay);
          GPIO_PORTM_DATA_R = 0b00000110; SysTick_Wait10us(delay);
          GPIO_PORTM_DATA_R = 0b00001100; SysTick_Wait10us(delay);
          GPIO_PORTM_DATA_R = 0b00001001; SysTick_Wait10us(delay);
        }

        // Reset state variables
        times = 0;       // Stop inner loop
        angle = 0;       // Reset angle for next
				total_count += 1;
				if (total_count%3==0) {
					z += 30;       // (Unused here — for 3D vertical stacking)
				}
				sensorState = 0;
        VL53L1X_StopRanging(dev); // Stop the sensor
				char buffer[100];
				sprintf(buffer, "Scan Complete, waiting for next cycle. Next Loop: %i, Next z: %i\r\n", total_count+1, (int)z);
				UART_printf(buffer);
				D1(1);
        break;
      }
    }
    // === Manual controls outside inner loop ===
	 
    // ?? Z-Level Display with LED

		
    if(is_b0_pressed()) //pj0
      times = 1;  // Restart scan on button B0

    if(is_b1_pressed())
    {
      times2 = 0;           // End outer loop on B1 press
      UART_printf("@");     // Print end symbol
    }
  }
}
