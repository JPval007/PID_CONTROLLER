//*****************************************************************************
/*
 * Universidad del Valle de Guatemala
 * Control Systems 1
 * Seccion: 11
 * Autor: Juan Pablo Valenzuela
 * Description: This is a digital PID controller designed to improve a
 * servo motor's performance.
 */
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"
//*****************************************************************************
//*****************************************************************************
// SPI configuration definitions
//*****************************************************************************
#define NUM_SPI_DATA    1  // Number of bytes send
#define SPI_FREC  4000000  // SPI clock freq
#define SPI_ANCHO      16  // Number of bits send each time, between 14 and 16
//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the UART.
//
//*****************************************************************************
uint32_t g_ui32Flags; //example flag
uint32_t ui32Period = 0; //Timer period
uint8_t cont0 = 0, cont1 = 0; //counters
volatile uint32_t ui32Loop;
unsigned char toggle = 0; //toggle control
unsigned char XOR_TEST = 0; //toggle with XOR boolean operation
uint32_t adc_value[2];


//TIMER
uint32_t freq_muestreo = 1000;    // In Hz


//UART - ADC
uint32_t UART_TEST = 0; //Test variable for UARTPRINTF();
int ADC_CONTROL = 0; //controls which ADC channel is used
//Final values for analog reading
float AN0;
float AN1;
//SPI VARIABLES
uint32_t pui32residual[NUM_SPI_DATA];
uint32_t pui32DataTx[NUM_SPI_DATA]; // uint32_t data for buffer
uint8_t ui32Index;
uint16_t dato = 0b0111000000000000;  // DAC opcode for writing
uint16_t uk_entero;
//Data processing
float ek_1 = 0;
float Ek_1 = 0;
float kp = 0.01;//;4.7801;
float ki = 0;//287.2819;
float kd = 0;//0.018349;
float ref = 0;
float yk = 0;
float ek = 0;
float uk = 0;
float ed = 0;
float Ek = 0;
//Float to integrer conversion
float unfold=0;

//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
void Startup(void);
void UART_CONFIG(void);
void TIMER0_CONFIG(void);
void Toggle(void);
void ADC_CONFIG(void);
void SendNum(uint32_t num);
//HANDLER FOR ADC INTERRPUT
void ADC0Handler(void);
void SPI_CONFIG(void);
//DATA PROCESSING FUNCTION
void procesar(void);
//*****************************************************************************
//
// The interrupt handler timer interrupt.
//
//*****************************************************************************
void Timer0IntHandler(void);
//*****************************************************************************
//
// SETUP
//
//*****************************************************************************
void Setup(void)
{
    // External 16MHz clock
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //Reloj configurado a 80MHz
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); //GREEN
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2); //BLUE
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1); //RED

    //UART MODULE
    UART_CONFIG();

    //TIMER0
    TIMER0_CONFIG();

    //ADC0 CONFIGURATION
    ADC_CONFIG();

    //SPI CONFIGURATION
    SPI_CONFIG();
}
//*****************************************************************************
//
//                          MAIN LOOP (ARDUINO ANALOGY)
//
//*****************************************************************************
int main(void)
{
    //Configure the peripherals
    Setup();

    //
    // Loop forever.
    //
    while(1)
    {
    // it is empty because readings are taken in the interruption.

    }
}


void Startup(void){
    //Sucessful start message
    UARTprintf("Hola Mundo\n\n\n");
    return;
}

void UART_CONFIG(void){
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 80000000);

    //STARTUP MESSAGE
    Startup();
    return;
}


void TIMER0_CONFIG(void){
    ///*************************************************************************************************************************
    //              TIMER0 CONFIGURATION    
    //*************************************************************************************************************************s

    // Enable TIMER0 clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure as periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Calculation of timer period (1 seg) 
    ui32Period = (SysCtlClockGet()/freq_muestreo);
    // Timer period setup
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    //Register TIMER0 interrupt handler
    IntRegister(INT_TIMER0A, Timer0IntHandler);

    // ENABLE TIMER0A INTERRUPT
    IntEnable(INT_TIMER0A);
    // Activate Timeout Interrupt type
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // ENABLE GLOBAL INTERRUPT
    IntMasterEnable();
    // ENABLE TIMER
    TimerEnable(TIMER0_BASE, TIMER_A);

    // CLEAR THE TIMER INTERRUPT (JUST IN CASE)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    return;
}









//**************************************************************************************************************
//                  TIMER 0 HANDLER
//**************************************************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Interruption code
    //Toggle(); //LED
    //XOR_TEST = XOR_TEST ^ 0X1; //LED TOGGLE USING XOR
    ADCProcessorTrigger(ADC0_BASE, 2); //START ADC READING IN CHANNEL 2

    //*********************************************************
    //DAC CODE
    //*********************************************************
    //SEND THE ANALOG VOLTAGE DATA
    //REMEMBER TO SEND AN INTEGRER VALUE

    dato = uk;
    // PUT THE DATA INTO THE pui32DataTx BUFFER
    pui32DataTx[0] = (uint32_t)(dato);

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    while(SSIBusy(SSI0_BASE))
    {
    }
    //DAC RAMP EXAMPLE
    /*
    dato = dato +1;
    if (dato == 0b0111111111111111){
        dato = 0b0111000000000000;
    }
     */

}














void Toggle(void){
    if (toggle==1){
        // Turn on the LED.
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
        toggle = 0;
    } else {
        // Turn off the LED.
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x0);
        toggle = 1;
    }
}

void ADC_CONFIG(void){
    //Habilitamos el módulo del ADC0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //HABILITAMOS EL PUERTO E QUE ES DONDE ESTÁ EL ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //HABILITAMOS LOS PINES PAG 801 DATASHEET
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    // ESPERAMOS A QUE ESTÉ LISTO
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    //CONFIGURAMOS EL ADC PARA QUE SE ACTIVE CUANDO QUERRAMOS
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    //ADC_CTL_CH0 ES EL CANAL 0 Y ADC_CTL_CH1 ES EL CANAL 1 ADC_CTL_IE ES PARA QUE CAUSE UNA INTERRUPCIÓN AL TERMINAR
    //ADC_CTL_END ES PARA QUE SOLO REALICE 1 MUESTRA
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
    //AHORA QUE SE CONFIGURÓ, SE ACTIVA ESA SECUENCIA
    ADCSequenceEnable(ADC0_BASE, 2);
    //NOS ASEGURAMOS QUE NUESTRAS INTERRUPCIONES ESTÉN DESACTIVADAS
    ADCIntClear(ADC0_BASE, 2);
    //ACTIVAMOS LAS INTERRUPCIONES DEL MÓDULO
    ADCIntEnable(ADC0_BASE, 2);
    //ASIGNAMOS LA FUNCIÓN PARA LAS INTERRUPCIONES
    ADCIntRegister(ADC0_BASE, 2, ADC0Handler);

    return;
}

void ADC0Handler(void){
    //Eliminamos las banderas de interrupción
    ADCIntClear(ADC0_BASE, 2);

    //LEEMOS EL VALOR
    //Se debe colocar un puntero porque la función solo requiere la dirección de memoria para almacenar el valor
    ADCSequenceDataGet(ADC0_BASE, 2, adc_value);

    //CONVERSIÓN A VALORES ANALÓGICOS
    AN0 = (float) adc_value[0]*3.3/4095.0; //Canal 0 (Step)
    AN1 = (float) adc_value[1]*3.3/4095.0; //Canal 1 (System response)
    ref = AN0;
    yk = AN1;
    //Se procesan los datos de una vez
    procesar();
    //MOSTRAMOSE EL VALOR POR UART
    //UARTprintf("AN0: %d\n -> %2f \nAN1: %d\n -> %2f \n", adc_value[0], AN0, adc_value[1], AN1);
}

void SPI_CONFIG(void){
    // The SSI0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    //CONFIGURAR LOS PINES QUE UTILIZARÁ EL SPI
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    //los pines los manejará el módulo SPI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
    // Enable the SSI0 module.
    SSIEnable(SSI0_BASE);
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32residual[0]))
    {
    }

}

void procesar(void){
    ek = ref - yk;
    ed = ek - ek_1;
    Ek = Ek + ek;
    uk =kp*ek + ki*Ek + kd*ed;
    ek_1 = ek;
    //Ek_1 = Ek;
    //descomprimimos la variable
    unfold = uk*4095.0/3.3; // convertimos a un valor entre 0 y 4095
    //SE DEBE CONVERTIR EL DATO DE FLOAT A ENTERO Y DALE FORMATO DE 16 BITS
    if (ref<0.1){
        unfold = 0.1;
    }
    uk_entero = (uint16_t) unfold;
/*
    if (uk_entero>=4095){
        uk_entero=uk_entero>>4; //si se pasa de 4095 entonces se utilizan los msb
        }
*/
    //Determinar si se pasa de 4095, si se pasa entonces se debe hacer un corrimiento >>4
    //                      0x0FFF
    uk_entero = uk_entero & 0b0000111111111111; //eliminamos los bits de configuracion
    //                      0xF000
    uk_entero = uk_entero | 0b0111000000000000; //aseguramos los bits de config
    dato = uk_entero;
}
