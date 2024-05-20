/***********************************************************************
 * ==========================================================================
 *
 * File: freertes_demo.c
 *
 * Author: Kiran Jojare, Ayswariya Kannan
 *
 * Project Name: Stop Sign Detection Bot on TIVA using FreeRTOS
 *
 * Description:
 * This file contains the implementation of a stop sign detection system
 * for a robotic vehicle controlled by the TIVA TM4C123GH6PM microcontroller,
 * utilizing FreeRTOS for task management. The project aims to demonstrate
 * real-time detection of stop signs, enabling the vehicle to make autonomous
 * navigation decisions.
 *
 * The code utilizes hardware interfacing examples provided by TIVAWare and
 * is designed to handle tasks such as image acquisition from a camera module,
 * image processing to detect stop signs, and motor control for vehicle movement.
 *
 * References:
 * - TIVAWare demo examples for initial hardware configuration and basic examples.
 * - UART communication on TM4C123: https://github.com/Reddimus/UARTcommunication-TM4C123
 * - PWM Control for TM4C123: https://github.com/Mohammed-AhmedAF/PWMController_TM4C123
 * - MotorLib Embedded for TIVA: https://github.com/GrandviewIoT/MotorLib_Embedded
 *
 *
 * Subject: ECEN - 5623 Real Time Operating Systems
 *
 * Professor: Tim Scherr
 *
 * University: University of Colorado, Boulder
 *
 * Academic Year: [2023-2024]
 *
 * ==========================================================================
 ***********************************************************************/


#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"          // Include main header for FreeRTOS.
#include "FreeRTOSConfig.h"    // Include FreeRTOS configuration settings.
#include "task.h"              // Include for task management utilities.
#include "queue.h"             // Include for queue management utilities.
#include "semphr.h"            // Include for semaphore management utilities.
#include "timers.h"            // Include for software timer utilities.
#include "inc/hw_types.h"      // Hardware specific header file to interact with the lower level hardware specific functions.
#include "inc/hw_memmap.h"     // Header file defining the memory map of the TIVA microcontroller.
#include "driverlib/sysctl.h"  // Include system control driver for system and clock control utilities.
#include "driverlib/interrupt.h" // Include for NVIC interrupt controller utilities.
#include "driverlib/timer.h"   // Include for hardware timer utilities.
#include "driverlib/gpio.h"    // Include for GPIO pin control utilities.
#include "driverlib/uart.h"    // Include for UART communication utilities.
#include "driverlib/pin_map.h" // Include for GPIO pin configuration that maps to specific functions.
#include "utils/uartstdio.h"   // Include for using UART functions for standard input and output.
#include "inc/hw_ints.h"       // Include for definitions of interrupt assignments.
#include "driverlib/rom.h"     // Include for ROM utility functions.
#include "driverlib/pwm.h"     // Include for Pulse Width Modulation signal generation utilities.
#include "driverlib/pin_map.h" // Include to define alternate functions for GPIO pins.

// Define constants for use in timing analysis and other features.
#define TIMING_ANALYSIS         1
#define FIBONACCI_ITERATIONS    5000 // Define the number of Fibonacci sequence iterations for test purposes.
#define MAX_SERVICE_EXECUTIONS  100  // Assuming a maximum of 180 executions for demonstration purposes.

// UART and motor configuration settings
#define MOTOR1_GPIO_PERIPH       SYSCTL_PERIPH_GPIOB
#define MOTOR1_GPIO_BASE         GPIO_PORTB_BASE
#define MOTOR1_PIN_A1            GPIO_PIN_0
#define MOTOR1_PIN_A2            GPIO_PIN_1

#define MOTOR2_GPIO_PERIPH       SYSCTL_PERIPH_GPIOC
#define MOTOR2_GPIO_BASE         GPIO_PORTC_BASE
#define MOTOR2_PIN_B1            GPIO_PIN_6           // Motor 2 Pin B1 connected to PC6
#define MOTOR2_PIN_B2            GPIO_PIN_7           // Motor 2 Pin B2 connected to PC7

#define PWM_FREQUENCY           20000  // Set PWM frequency in Hz.

#define UART1_RX_PERIPH         SYSCTL_PERIPH_UART1
#define UART1_RX_BASE           UART1_BASE
#define UART1_RX_PORT_PERIPH    SYSCTL_PERIPH_GPIOC
#define UART1_RX_PORT_BASE      GPIO_PORTC_BASE
#define UART1_RX_PIN            GPIO_PIN_4
#define UART1_RX_PIN_CONF       GPIO_PC4_U1RX

#define UART2_TX_PERIPH         SYSCTL_PERIPH_UART2
#define UART2_TX_BASE           UART2_BASE
#define UART2_TX_PORT_PERIPH    SYSCTL_PERIPH_GPIOD
#define UART2_TX_PORT_BASE      GPIO_PORTD_BASE
#define UART2_TX_PIN            GPIO_PIN_7
#define UART2_TX_PIN_CONF       GPIO_PD7_U2TX

#define UART2_BAUDRATE          (115200)    // Set baud rate for UART.

// System clock rate definition.
#define SYSTEM_CLOCK            50000000U   // System clock rate in Hz.

// LED configuration details.
#define LED_PERIPH              SYSCTL_PERIPH_GPIOF
#define LED_PORT                GPIO_PORTF_BASE
#define LED_PIN                 GPIO_PIN_2  // Blue LED on TIVA boards.

#define BYTES_PER_TRANS         8  // Define number of bytes per UART transmission.

// Flags to indicate whether a service should be aborted.
static bool abortS1=false, abortS2=false, abortS3=false, abortS4=false;

 //////////////////////////////////////////////////////////////////////////
 ///////////////////    Function Declarations     /////////////////////////
 //////////////////////////////////////////////////////////////////////////

 // System configuration functions.
 void SystemConfig();          // Configures the overall system settings, like clock and power.
 void GPIOConfig();            // Configures the general purpose input/output pins for the system.
 void UART0Config(void);       // Configures UART0 for serial communication.
 void TimerConfig();           // Sets up the hardware timers for timing and delays.
 void SemaphoresConfig(void);  // Initializes semaphores used for task synchronization.
 void TaskConfig();            // Sets up and creates FreeRTOS tasks.

 // ISR's and UART Function Prototypes
 void ConfigureUARTJetson(void);   // Sets up UART communication for a specific device, like a Jetson board.
 void UART1IntHandler(void);       // Interrupt handler for UART1, processes UART1 communication interrupts.
 void UART2IntHandler(void);       // Interrupt handler for UART2, handles interrupts from UART2.

 // Motor Function Prototypes
 void ConfigurePWM(uint32_t pwmPeriod);    // Configures PWM settings for motor control.
 void ConfigureMotorGPIO(void);            // Sets up GPIO for motor control.
 void MotorForward(void);                  // Commands motors to drive forward.
 void MotorReverse(void);                  // Commands motors to reverse direction.
 void MotorStop(void);                     // Stops the motors.
 void MotorStart(void);                    // Starts the motors.

 // Task Function Prototypes
 void CameraUARTService1(void *pvParameters);  // FreeRTOS task function for handling UART communication.
 void Motor1Service2(void *pvParameters);      // Task function for controlling Motor 1.
 void Motor2Service3(void *pvParameters);      // Task function for controlling Motor 2.
 void DiagnosticsLEDService4(void *pvParameters); // Task function for LED diagnostics.

 // Declaration of Semaphore Handles
 SemaphoreHandle_t semaphore1, semaphore2, semaphore3, semaphore4; // Semaphores for synchronizing tasks and ISR.

 volatile TickType_t maxExecutionTimeS1 = 0; // Tracks maximum execution time for service 1.
 volatile TickType_t maxExecutionTimeS2 = 0; // Tracks maximum execution time for service 2.
 volatile TickType_t maxExecutionTimeS3 = 0; // Tracks maximum execution time for service 3.
 volatile TickType_t maxExecutionTimeS4 = 0; // Tracks maximum execution time for service 4.

 SemaphoreHandle_t semaphoreUART; // Semaphore for UART communication synchronization.

 // Global Shared Data Between ISR and Service 1 of CameraUARTService
 volatile uint8_t lastReceivedByte;  // Last byte received from UART, shared globally.
 volatile bool newDataAvailable = false;  // Flag to indicate new data is available from UART.

 int seqCnt = 0;  // Counter used for sequencing in ISRs or timed events.

 // Data structure for tracking service execution timing.
 typedef struct {
     TickType_t* startTime;      // Array of start times for each execution.
     TickType_t* endTime;        // Array of end times for each execution.
     uint32_t serviceCount;      // Number of times the service has been executed.
     TickType_t wcet;            // Worst-case execution time.
 } ServiceData;

 // Initialize ServiceData for Service 1
 ServiceData serviceData1 = {0};
 ServiceData serviceData2 = {0};
 ServiceData serviceData3 = {0};
 ServiceData serviceData4 = {0};

 uint32_t idx = 0, jdx = 1;
 uint32_t fib = 0, fib0 = 0, fib1 = 1;  // Variables for Fibonacci calculation.

 #define FIB_TEST(seqCnt, iterCnt)   \
  for(idx=0; idx < iterCnt; idx++)   \
  {                                  \
      fib = fib0 + fib1;             \
      while(jdx < seqCnt)            \
      {                              \
          fib0 = fib1;               \
          fib1 = fib;                \
          fib = fib0 + fib1;         \
          jdx++;                     \
  }                                  \
 }

 // Initializes service data for dynamic allocation of timing data.
 void InitServiceData(ServiceData* serviceData, uint32_t maxExecutions) {
     serviceData->startTime = pvPortMalloc(maxExecutions * sizeof(TickType_t));
     serviceData->endTime = pvPortMalloc(maxExecutions * sizeof(TickType_t));
     serviceData->serviceCount = 0;
     serviceData->wcet = 0;
 }

 // Deinitializes service data, freeing allocated memory.
 void DeinitServiceData(ServiceData* serviceData) {
     vPortFree(serviceData->startTime);
     vPortFree(serviceData->endTime);
 }

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) { }
#endif

void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    while (1)
    {
    }
}

void Timer0AInterruptHandler(void) {

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Increment the sequencer count.
    seqCnt++;

    // Check if it's time to abort all tasks.
    // run for total of 10 seconds
    if (seqCnt >= 1000) {
        // Set abort flags for all tasks.
        abortS1 = true; abortS2 = true; abortS3 = true;abortS4 = true;

        // Release semaphores to unblock tasks for clean up.
        xSemaphoreGiveFromISR(semaphore1, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(semaphore2, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(semaphore3, &xHigherPriorityTaskWoken);
        xSemaphoreGiveFromISR(semaphore4, &xHigherPriorityTaskWoken);

        MotorStop();

    } else {
        // Service_1 - 5 Hz, every 50nd Sequencer loop
        if ((seqCnt % 20) == 0) {
            if(xSemaphoreGiveFromISR(semaphore1, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Handle error
                UARTprintf("Semaphore give for Service 1 failed!\n");
            }
        }

        // Service_2 - 100 Hz, every 1th Sequencer loop
        if ((seqCnt % 1) == 0) {
            if(xSemaphoreGiveFromISR(semaphore2, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Handle error
                UARTprintf("Semaphore give for Service 2 failed!\n");
            }
        }

        // Service_3 -100 Hz, every 1th Sequencer loop
        if ((seqCnt % 1) == 0) {
            if(xSemaphoreGiveFromISR(semaphore3, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Handle error
                UARTprintf("Semaphore give for Service 3 failed!\n");
            }
        }

        // Service_4 - 4 Hz, every 50th Sequencer loop
        if ((seqCnt % 25) == 0) {
            if(xSemaphoreGiveFromISR(semaphore4, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Handle error
                UARTprintf("Semaphore give for Service 4 failed!\n");
            }
        }

    }

    // Toggle the LED as a visual heartbeat indicator.
    // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, ~GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2));

    // Force a context switch if xHigherPriorityTaskWoken was set to true.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//////////////////////////////////////////////////////////////////////////
///////////////////       Main Entry Point       /////////////////////////
//////////////////////////////////////////////////////////////////////////

int main(void)
{
    SystemConfig();

    GPIOConfig();

    SemaphoresConfig();

    UART0Config();

    // Initialize PWM and GPIO configurations
    uint32_t pwmPeriod = ROM_SysCtlClockGet() / PWM_FREQUENCY;
    ConfigurePWM(pwmPeriod);
    ConfigureMotorGPIO();

    // Start the motor system
    MotorStart();

    // Drive the motor forward
    MotorForward();

    //Giving initial semaphore
    xSemaphoreGive(semaphoreUART);

    UARTprintf("STOP SIGN DETECTION BOT RUNNING.............\n");

    TimerConfig();

    TaskConfig();

    ConfigureUARTJetson();

    vTaskStartScheduler();

    while (1)
    {
        // Main loop does nothing but sleep
        // SysCtlSleep();

        ;
    }
}

//////////////////////////////////////////////////////////////////////////
///////////////////    Function Definitions      /////////////////////////
//////////////////////////////////////////////////////////////////////////

/**
 * Configures the system clock for the microcontroller.
 */
void SystemConfig()
{
    // Set the system clock to 50 MHz from PLL with a crystal reference of 16 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

/**
 * Configures general-purpose input/output settings for the system, particularly for LEDs.
 */
void GPIOConfig()
{
    // The following code is commented out but when enabled, it would:
    // Enable the GPIO port for the LED (PF2)
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Configure the GPIO pin for the LED as an output
    // GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
}

/**
 * Configures UART0 for basic serial communication.
 */
void UART0Config(void)
{
    // Enable GPIO Peripheral for UART0 pins and UART0 itself
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure the GPIO pins for UART mode
    GPIOPinConfigure(GPIO_PA0_U0RX);  // Configure PA0 as UART0 RX
    GPIOPinConfigure(GPIO_PA1_U0TX);  // Configure PA1 as UART0 TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Set UART clock source and configure the UART for 115200 baud rate
    // UART_CLOCK_PIOSC is often used to avoid conflicts with the system clock used by SysCtlClockSet
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000); // Use internal 16MHz oscillator for UART
}

/**
 * Configures the system timer to trigger an interrupt at a frequency of 100 Hz.
 */
void TimerConfig()
{
    // Enable Timer 0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Enable processor interrupts
    IntMasterEnable();

    // Configure Timer0 as a 32-bit timer in periodic mode
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Calculate and set the timer load value for a 100 Hz frequency
    uint32_t timerLoad = (SysCtlClockGet() / 100) - 1; // System clock divided by frequency, minus one for zero-based index
    TimerLoadSet(TIMER0_BASE, TIMER_A, timerLoad);

    // Enable the specific Timer0A interrupt
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Register the interrupt handler for Timer0A
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AInterruptHandler);

    // Start Timer0A
    TimerEnable(TIMER0_BASE, TIMER_A);
}

/**
 * Initializes semaphores used for synchronizing tasks and interrupts.
 */
void SemaphoresConfig(void) {
    // Create binary semaphores for task synchronization and error check
    semaphore1 = xSemaphoreCreateBinary();
    if (semaphore1 == NULL) { UARTprintf("Error: Failed to create Semaphore 1\n"); }

    semaphore2 = xSemaphoreCreateBinary();
    if (semaphore2 == NULL) { UARTprintf("Error: Failed to create Semaphore 2\n"); }

    semaphore3 = xSemaphoreCreateBinary();
    if (semaphore3 == NULL) { UARTprintf("Error: Failed to create Semaphore 3\n"); }

    semaphore4 = xSemaphoreCreateBinary();
    if (semaphore4 == NULL) { UARTprintf("Error: Failed to create Semaphore 4\n"); }

    semaphoreUART = xSemaphoreCreateBinary();
    if (semaphoreUART == NULL) { UARTprintf("Error: Failed to create UART Semaphore\n"); }
}

/**
 * Configures and creates tasks.
 */
void TaskConfig()
{
    BaseType_t status;

    // Create a task for handling UART communication with the camera module
    status = xTaskCreate(CameraUARTService1, "CameraUARTService1", 100, NULL, configMAX_PRIORITIES - 2, NULL);
    if (status != pdTRUE) { UARTprintf("Error: Failed to create Camera UART Service 1\n"); }

    // Create a task for controlling Motor 1
    status = xTaskCreate(Motor1Service2, "Motor1Service2", 128, NULL, configMAX_PRIORITIES - 1, NULL);
    if (status != pdTRUE) { UARTprintf("Error: Failed to create Motor 1 Service 2\n"); }

    // Create a task for controlling Motor 2
    status = xTaskCreate(Motor2Service3, "Motor2Service3", 128, NULL, configMAX_PRIORITIES - 1, NULL);
    if (status != pdTRUE) { UARTprintf("Error: Failed to create Motor 2 Service 3\n"); }

    // Create a task for managing diagnostic LEDs
    status = xTaskCreate(DiagnosticsLEDService4, "DiagnosticsLEDService4", 128, NULL, configMAX_PRIORITIES - 3, NULL);
    if (status != pdTRUE) { UARTprintf("Error: Failed to create Diagnostics LED Service 4\n"); }
}
/**
 * Configures UART peripherals for communication with Jetson (or other UART devices).
 * This function sets up UART1 for receiving data and UART2 for transmitting data.
 */
void ConfigureUARTJetson(void) {
    // Enable UART1 and UART2 peripherals to ensure they are powered and ready for configuration.
    SysCtlPeripheralEnable(UART1_RX_PERIPH);
    SysCtlPeripheralEnable(UART1_RX_PORT_PERIPH);
    SysCtlPeripheralEnable(UART2_TX_PERIPH);
    SysCtlPeripheralEnable(UART2_TX_PORT_PERIPH);

    // Configure the pin associated with UART1_RX for UART functionality.
    GPIOPinConfigure(UART1_RX_PIN_CONF);
    GPIOPinTypeUART(UART1_RX_PORT_BASE, UART1_RX_PIN);

    // Configure the pin associated with UART2_TX for UART functionality.
    GPIOPinConfigure(UART2_TX_PIN_CONF);
    GPIOPinTypeUART(UART2_TX_PORT_BASE, UART2_TX_PIN);

    // Set up UART1 for reception at 115200 baud, 8 data bits, 1 stop bit, and no parity.
    UARTConfigSetExpClk(UART1_RX_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntEnable(UART1_RX_BASE, UART_INT_RX);
    UARTEnable(UART1_RX_BASE);

    // Set up UART2 for transmission at 115200 baud, 8 data bits, 1 stop bit, and no parity.
    UARTConfigSetExpClk(UART2_TX_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTEnable(UART2_TX_BASE);

    // Register and enable interrupt handler for UART1_RX to handle incoming data.
    UARTIntRegister(UART1_RX_BASE, UART1IntHandler);
    IntEnable(INT_UART1);

    // Register and enable interrupt handler for UART2_TX (optional here, not utilized in this function).
    UARTIntRegister(UART2_TX_BASE, UART2IntHandler);
    IntEnable(INT_UART2);
}

/**
 * Interrupt handler for UART1. This function processes incoming data as soon as it is available.
 */
void UART1IntHandler(void) {
    // Get the current interrupt status and clear it.
    uint32_t ui32Status = UARTIntStatus(UART1_RX_BASE, true);
    UARTIntClear(UART1_RX_BASE, ui32Status);

    // Continue reading data while characters are available in the UART buffer.
    while (UARTCharsAvail(UART1_RX_BASE)) {
        // Read the incoming byte and store it globally.
        lastReceivedByte = UARTCharGet(UART1_RX_BASE);
        newDataAvailable = true;  // Flag to indicate new data is ready to be processed.
    }
}

/**
 * Placeholder for UART2 interrupt handler. Currently, it does nothing but is required for completeness.
 */
void UART2IntHandler(void) {
    // Currently empty, as no specific functionality is implemented for UART2 TX interrupts.
}

/**
 * Configures PWM (Pulse Width Modulation) for controlling motor speed.
 * This function sets up PWM generators for two motors.
 * @param pwmPeriod The period of the PWM signal, which determines the frequency.
 */
void ConfigurePWM(uint32_t pwmPeriod) {
    // Set the PWM clock divider to 1 for full speed.
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the PWM peripherals for motor control.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1); // PWM1 for Motor 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM0 for Motor 2

    // Enable the GPIO peripherals that the PWM pins are multiplexed on.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // GPIOF for Motor 1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // GPIOB for Motor 2

    // Set the specific pins to be used as PWM outputs.
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0); // PF0 for Motor 1
    ROM_GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4); // PB4 for Motor 2

    // Configure pin muxing for PWM output on these pins.
    ROM_GPIOPinConfigure(GPIO_PF0_M1PWM4); // PF0 as M1PWM4 for Motor 1
    ROM_GPIOPinConfigure(GPIO_PB4_M0PWM2); // PB4 as M0PWM2 for Motor 2

    // Configure and enable PWM generator for Motor 1.
    ROM_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, pwmPeriod);
    ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, pwmPeriod / 2);  // Set to 50% duty cycle
    ROM_PWMOutputState(PWM1_BASE, PWM_OUT_4_BIT, true);
    ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_2);

    // Configure and enable PWM generator for Motor 2.
    ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwmPeriod);
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwmPeriod / 2);  // Set to 50% duty cycle
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/**
 * Configures GPIOs for motor control, setting up pins as output for controlling motor direction.
 */
void ConfigureMotorGPIO(void) {
    // Enable the GPIO peripheral for Motor 1 and configure its pins as outputs.
    ROM_SysCtlPeripheralEnable(MOTOR1_GPIO_PERIPH);
    ROM_GPIOPinTypeGPIOOutput(MOTOR1_GPIO_BASE, MOTOR1_PIN_A1 | MOTOR1_PIN_A2);
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A1 | MOTOR1_PIN_A2, 0); // Initialize pins to low.

    // Enable the GPIO peripheral for Motor 2 and configure its pins as outputs.
    ROM_SysCtlPeripheralEnable(MOTOR2_GPIO_PERIPH);
    ROM_GPIOPinTypeGPIOOutput(MOTOR2_GPIO_BASE, MOTOR2_PIN_B1 | MOTOR2_PIN_B2);
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B1 | MOTOR2_PIN_B2, 0); // Initialize pins to low.
}


/**
 * Sets the direction of both motors to forward.
 * Motor 1 and Motor 2 are configured to rotate forward by setting appropriate GPIO outputs.
 */
void MotorForward(void) {
    // Set Motor 1 to move forward by activating one pin and deactivating the other.
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A1, MOTOR1_PIN_A1);  // Activate Motor 1 Pin A1
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A2, 0);             // Deactivate Motor 1 Pin A2

    // Set Motor 2 to move forward by activating one pin and deactivating the other.
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B1, MOTOR2_PIN_B1);  // Activate Motor 2 Pin B1
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B2, 0);              // Deactivate Motor 2 Pin B2
}

/**
 * Sets the direction of both motors to reverse.
 * Motor 1 and Motor 2 are configured to rotate in the reverse direction by setting appropriate GPIO outputs.
 */
void MotorReverse(void) {
    // Set Motor 1 to move in reverse by activating one pin and deactivating the other.
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A2, MOTOR1_PIN_A2);  // Activate Motor 1 Pin A2
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A1, 0);              // Deactivate Motor 1 Pin A1

    // Set Motor 2 to move in reverse by activating one pin and deactivating the other.
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B2, MOTOR2_PIN_B2);  // Activate Motor 2 Pin B2
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B1, 0);              // Deactivate Motor 2 Pin B1
}

/**
 * Stops both motors by setting both control pins of each motor to low.
 * This function ensures that both motors are in a stopped state with no voltage difference across the motor terminals.
 */
void MotorStop(void) {
    // Stop Motor 1 by setting both control pins low.
    ROM_GPIOPinWrite(MOTOR1_GPIO_BASE, MOTOR1_PIN_A1 | MOTOR1_PIN_A2, 0);

    // Stop Motor 2 by setting both control pins low.
    ROM_GPIOPinWrite(MOTOR2_GPIO_BASE, MOTOR2_PIN_B1 | MOTOR2_PIN_B2, 0);
}

/**
 * Starts or restarts the motors by first ensuring they are stopped and then setting them to move forward.
 * This function can be used to safely start the motors during system initialization or after an emergency stop.
 */
void MotorStart(void) {
    // Ensure that both motors are stopped before starting.
    MotorStop();

    // Optionally start the motors moving forward. Uncomment the next line to enable this behavior.
    // MotorForward();  // Uncomment to start motors in forward direction by default
}


//////////////////////////////////////////////////////////////////////////
////////////////    Task Function Definitions      ///////////////////////
//////////////////////////////////////////////////////////////////////////

void CameraUARTService1(void* pvParameters) {
    uint32_t estimatedMaxExecutions = 20; // Adjust based on expected maximum
    InitServiceData(&serviceData1, estimatedMaxExecutions);

    while (!abortS1) {
        if (xSemaphoreTake(semaphore1, portMAX_DELAY) == pdPASS) {
            serviceData1.startTime[serviceData1.serviceCount] = xTaskGetTickCount();

            if (newDataAvailable) {
                taskENTER_CRITICAL();
                uint8_t data = lastReceivedByte;  // Copy to local variable to minimize time in critical section
                newDataAvailable = false;
                taskEXIT_CRITICAL();

                TickType_t currentTime = xTaskGetTickCount();
                UARTprintf("[%u ms] [CameraUARTService1] Received Byte: 0x%02X\n", currentTime, data);

                switch(data) {
                    case 0xAA:
                        UARTprintf("[%u ms] [CameraUARTService1] Alert: STOP Sign Detected - Vehicle HALTED\n", currentTime);
                        break;
                    case 0x00:
                        UARTprintf("[%u ms] [CameraUARTService1] Info: Path Clear - Vehicle Continuing\n", currentTime);
                        break;
                    default:
                        UARTprintf("[%u ms] [CameraUARTService1] Warning: Unknown Command 0x%02X - No Action Taken\n", currentTime, data);
                        break;
                }
            }

            serviceData1.endTime[serviceData1.serviceCount] = xTaskGetTickCount();
            serviceData1.serviceCount++;

            TickType_t executionTime = serviceData1.endTime[serviceData1.serviceCount - 1] - serviceData1.startTime[serviceData1.serviceCount - 1];
            if (executionTime > serviceData1.wcet) {
                serviceData1.wcet = executionTime;
            }
        }
    }

    if (xSemaphoreTake(semaphoreUART, portMAX_DELAY) == pdPASS) {
#if TIMING_ANALYSIS==1
        int i = 0;
        for (i = 0; i < serviceData1.serviceCount; i++) {
            UARTprintf("[%u ms] [CameraUARTService1] Execution %u - Start: %u ms, End: %u ms, Execution Time: %u ms\n",
                        xTaskGetTickCount(), i + 1, serviceData1.startTime[i], serviceData1.endTime[i], serviceData1.endTime[i] - serviceData1.startTime[i]);
        }
#endif
        UARTprintf("[%u ms] [CameraUARTService1] Summary: Total Executions: %u, WCET: %u ms\n",
                    xTaskGetTickCount(), serviceData1.serviceCount, serviceData1.wcet);

        xSemaphoreGive(semaphoreUART);
    }

    DeinitServiceData(&serviceData1);
    vTaskDelete(NULL);
}

void Motor1Service2(void* pvParameters) {
    uint32_t estimatedMaxExecutions = 20; // Adjust based on expected maximum
    InitServiceData(&serviceData2, estimatedMaxExecutions);

    while (!abortS2) {
        if (xSemaphoreTake(semaphore2, portMAX_DELAY) == pdPASS) {
            serviceData2.startTime[serviceData2.serviceCount] = xTaskGetTickCount();

            if (newDataAvailable) {
                taskENTER_CRITICAL();
                uint8_t command = lastReceivedByte;
                newDataAvailable = false;
                taskEXIT_CRITICAL();

                TickType_t currentTime = xTaskGetTickCount();
                if (command == 0xAA) {  // If STOP sign detected
                    MotorStop();  // Stop the motor
                    UARTprintf("[%u ms] STOP Sign Detected - Motor Stopped.\n", currentTime);
                } else if (command == 0x00) {  // If STOP sign cleared
                    MotorForward();  // Resume the motor forward
                    UARTprintf("[%u ms] [Motor1Service2] Path Clear - Motor Resumed Forward.\n", currentTime);
                }
            }

            serviceData2.endTime[serviceData2.serviceCount] = xTaskGetTickCount();
            serviceData2.serviceCount++;

            TickType_t executionTime = serviceData2.endTime[serviceData2.serviceCount - 1] - serviceData2.startTime[serviceData2.serviceCount - 1];
            if (executionTime > serviceData2.wcet) {
                serviceData2.wcet = executionTime;
            }
        }
    }

    if (xSemaphoreTake(semaphoreUART, portMAX_DELAY) == pdPASS) {
#if TIMING_ANALYSIS==1
        int i = 0;
        for (i = 0; i < serviceData2.serviceCount; i++) {
            UARTprintf("[%u ms] [Motor1Service2] Execution %u - Start: %u ms, End: %u ms, Execution Time: %u ms\n",
                        xTaskGetTickCount(), i + 1, serviceData2.startTime[i], serviceData2.endTime[i], serviceData2.endTime[i] - serviceData2.startTime[i]);
        }
#endif
        UARTprintf("[%u ms] [Motor1Service2] Summary: Total Executions: %u, WCET: %u ms\n",
                    xTaskGetTickCount(), serviceData2.serviceCount, serviceData2.wcet);

        xSemaphoreGive(semaphoreUART);
    }

    DeinitServiceData(&serviceData2);
    vTaskDelete(NULL);
}



void Motor2Service3(void* pvParameters) {
    uint32_t estimatedMaxExecutions = 20; // Adjust based on expected maximum
    InitServiceData(&serviceData3, estimatedMaxExecutions);

    while (!abortS3) {
        if (xSemaphoreTake(semaphore3, portMAX_DELAY) == pdPASS) {
            serviceData3.startTime[serviceData3.serviceCount] = xTaskGetTickCount();

            if (newDataAvailable) {
                taskENTER_CRITICAL();
                uint8_t command = lastReceivedByte;
                newDataAvailable = false;
                taskEXIT_CRITICAL();

                TickType_t currentTime = xTaskGetTickCount();
                if (command == 0xAA) {  // If STOP sign detected
                    MotorStop();  // Stop the motor
                    UARTprintf("[%u ms] [Motor2Service3] STOP Sign Detected - Motor Stopped.\n", currentTime);
                } else if (command == 0x00) {  // If STOP sign cleared
                    MotorForward();  // Resume the motor forward
                    UARTprintf("[%u ms] [Motor2Service3] Path Clear - Motor Resumed Forward.\n", currentTime);
                }
            }
            // FIB_TEST(47, 2000); // Placeholder for the actual workload

            serviceData3.endTime[serviceData3.serviceCount] = xTaskGetTickCount();
            serviceData3.serviceCount++;

            TickType_t executionTime = serviceData3.endTime[serviceData3.serviceCount - 1] - serviceData3.startTime[serviceData3.serviceCount - 1];
            if (executionTime > serviceData3.wcet) {
                serviceData3.wcet = executionTime;
            }
        }
    }

    if (xSemaphoreTake(semaphoreUART, portMAX_DELAY) == pdPASS) {
#if TIMING_ANALYSIS==1
        int i = 0;
        for (i = 0; i < serviceData3.serviceCount; i++) {
            UARTprintf("[%u ms] [Motor2Service3] Execution %u - Start: %u ms, End: %u ms, Execution Time: %u ms\n",
                        xTaskGetTickCount(), i + 1, serviceData3.startTime[i], serviceData3.endTime[i], serviceData3.endTime[i] - serviceData3.startTime[i]);
        }
#endif
        UARTprintf("[%u ms] [Motor2Service3] Summary: Total Executions: %u, WCET: %u ms\n",
                    xTaskGetTickCount(), serviceData3.serviceCount, serviceData3.wcet);

        xSemaphoreGive(semaphoreUART);
    }

    DeinitServiceData(&serviceData3);
    vTaskDelete(NULL);
}

void DiagnosticsLEDService4(void* pvParameters) {
    uint32_t estimatedMaxExecutions = 20; // Adjust based on expected maximum
    InitServiceData(&serviceData4, estimatedMaxExecutions);

    // Initialize the blue LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    while (!abortS4) {
        if (xSemaphoreTake(semaphore4, portMAX_DELAY) == pdPASS) {
            serviceData4.startTime[serviceData4.serviceCount] = xTaskGetTickCount();
            // FIB_TEST(47, 2000); // Placeholder for the actual workload

            if (newDataAvailable) {
                taskENTER_CRITICAL();
                uint8_t command = lastReceivedByte;  // Copy to local variable to minimize time in critical section
                newDataAvailable = false;
                taskEXIT_CRITICAL();

                if (command == 0xAA) {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  // Turn on the blue LED
                    UARTprintf("[%u ms] [DiagnosticsLEDService4] Received Command AA: Blue LED ON\n", xTaskGetTickCount());
                } else if (command == 0x00) {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);  // Turn off the blue LED
                    UARTprintf("[%u ms] [DiagnosticsLEDService4] Received Command 00: Blue LED OFF\n", xTaskGetTickCount());
                }
            }

            serviceData4.endTime[serviceData4.serviceCount] = xTaskGetTickCount();
            serviceData4.serviceCount++;

            // Calculate WCET dynamically
            TickType_t executionTime = serviceData4.endTime[serviceData4.serviceCount - 1] - serviceData4.startTime[serviceData4.serviceCount - 1];
            if (executionTime > serviceData4.wcet) {
                serviceData4.wcet = executionTime;
            }
        }
    }

    if (xSemaphoreTake(semaphoreUART, portMAX_DELAY) == pdPASS) {
#if TIMING_ANALYSIS==1
        int i = 0;
        for (i = 0; i < serviceData4.serviceCount; i++) {
            UARTprintf("[%u ms] [DiagnosticsLEDService4] Execution %u - Start: %u ms, End: %u ms, Execution Time: %u ms\n",
                        xTaskGetTickCount(), i + 1, serviceData4.startTime[i], serviceData4.endTime[i], serviceData4.endTime[i] - serviceData4.startTime[i]);
        }
#endif
        UARTprintf("[%u ms] [DiagnosticsLEDService4] Summary: Total Executions: %u, WCET: %u ms\n",
                    xTaskGetTickCount(), serviceData4.serviceCount, serviceData4.wcet);

        xSemaphoreGive(semaphoreUART);
    }

    // Clean up
    DeinitServiceData(&serviceData4);
    vTaskDelete(NULL);
}
