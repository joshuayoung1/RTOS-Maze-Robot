//ECE 4437
//Joshua Young, Ben Avner, Francisco Sorto
//RTOS Robot
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include <ti/sysbios/BIOS.h>
#define PWM_FREQUENCY 55
int PIDvalue;
float setpoint = 6.25;
int p_error;
float lasterror, error, integral, derivative;
short int pong[20],ping[20], ping_count = 0, pong_count = 0, data_flag = 0, ptrack = 0;
void btinit(void);
float getvalue(void);
void pwminit(void);
void distinit(void);
void uartcomint(void);
void pwm(char m, char s, char s1);
float getfrontdist(void);
void pidcontrol(void);
void delayms(int ms);
void delaymicro(int ms);
void reflectinit(void);
bool isblackline(void);
void datacollect(void);
void pong_print(void);
void ping_print(void);
void ledinit(void);
int main(void){
//40 MHz Clock
SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
//Hardware Initialization
btinit();
pwminit();
distinit();
reflectinit();
ledinit();
//Collects one distance before starting to use in PID calculation
lasterror = getvalue() - setpoint;
BIOS_start();
}
//Starts Data collection if data flag is set to 1. This function is ran by BIOS every 100[ms]
void datacollect(void){
if(data_flag == 1){
p_error = (int)(error*100);
if(pong_count < 20){
if(ping_count < 20){
ping[ping_count] = p_error;
ping_count++;
}else if(ping_count == 20){
pong[pong_count] = p_error;
pong_count++;
ping_count++;
 GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);//Green
ping_print();
ptrack = 1;//Print pong
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
}else{
pong[pong_count] = p_error;
pong_count++;
}
}else{
ping_count = 0;
pong_count = 0;
ping[ping_count] = p_error;
ping_count++;
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
pong_print();
ptrack = 0;//print ping
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
}
}
}
//Function to print ping buffer
void ping_print(void){
UARTprintf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ",
ping[0], ping[1], ping[2], ping[3], ping[4], ping[5], ping[6], ping[7], ping[8], ping[9], ping[10],
ping[11], ping[12], ping[13],
ping[14], ping[15], ping[16], ping[17], ping[18], ping[19]);
}
//Function to print pong buffer
void pong_print(void){
UARTprintf("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ",
pong[0], pong[1], pong[2], pong[3], pong[4], pong[5], pong[6], pong[7], pong[8], pong[9],
pong[10], pong[11], pong[12], pong[13],
pong[14], pong[15], pong[16], pong[17], pong[18], pong[19]);
}
//PID function, which is ran every 50 [ms] by BIOS
void pidcontrol(void){
//Gets front and side distances
float sidedist = getvalue();
float frontdist = getfrontdist();
error = sidedist - setpoint;
derivative = error-lasterror;
//Intersection - Turn right
if (sidedist >15){
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7000);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 7000);
delayms(400);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
 //UARTprintf("Right!\n");
delayms(400);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
delayms(700);
}
//Dead End - Uturn
if (frontdist < 6){
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7000);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 7000);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
//UARTprintf("U-Turn\n");
delayms(850);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
lasterror = getvalue()-setpoint;
}
//Sets new motor speed with PID value
PIDvalue = (error*1200 + derivative*600);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7000 + PIDvalue);
lasterror = error;
}
//Function to determine black line. Called in BIOS every 10[ms]
bool isblackline(void){
uint32_t cycles=0;
//Writes pin high
GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
delaymicro(15);
GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
//Waits for pin to fall low, while counting cycles
while((GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0) && (cycles <800)){
cycles++;
}
//If cycles greater than 400 it is black line
if(cycles > 400){
//Recheck for thick black line
delayms(160);
GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0);
GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_PIN_0);
delaymicro(15);
GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
cycles = 0;
while((GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0) && (cycles <800)){
cycles++;
}
 //If it is black again, it is thick black line. Stop motors, stop BIOS
if(cycles > 400){
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
BIOS_exit();
return true;
//If it is not black again, it is THIN black line.
}else{
//IF first thin black line, start data collection
if(data_flag == 0){
data_flag = 1;
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
//If second thin black line, stop data collection and print out remaining ping/pong buffers.
}else{
data_flag = 0;
GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
if(ptrack == 0){
ping_print();
}else{
pong_print();
}
}
return true;
}
//Cycles less than 400 - not black
}else{
return false;
 }
}
//Function to return side distance in centimeters
float getvalue(void){
uint32_t ui32ADC0Value[4]; //ADC value
volatile float ui32DistAvg; //Average
ADCIntClear(ADC0_BASE, 1);
ADCProcessorTrigger(ADC0_BASE, 1);
//waits for ADC to be ready
while(!ADCIntStatus(ADC0_BASE, 1, false)){
//nothing
}
//gets data, stores in ui32ADC0Value
ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
ui32DistAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3])/4;
ui32DistAvg = 22700*pow(ui32DistAvg, -1.08476) + 0.8; // Units in cm - Add or subtract to calibrate
return ui32DistAvg;
}
//Function to return Front Distance in centimeters
float getfrontdist(void){
 uint32_t ui32ADC0Value; //ADC int value
volatile float frontdist;
ADCIntClear(ADC0_BASE, 3);
ADCProcessorTrigger(ADC0_BASE, 3);
//waits for ADC to be ready
while(!ADCIntStatus(ADC0_BASE, 3, false)){
//nothing
}
//gets data, stores in ui32ADC0Value
ADCSequenceDataGet(ADC0_BASE, 3, &ui32ADC0Value);
frontdist = ui32ADC0Value;
frontdist = 22700*pow(frontdist, -1.08476) + 0.8; // Units in cm - Add or subtract to calibrate
return frontdist;
}
//ON-Board LED initialization
void ledinit(void){
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}
//Bluetooth Initialization
void btinit(void){
//Enable Port B, and UART 1 (PB0, PB1)
SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
GPIOPinConfigure(GPIO_PB0_U1RX);
GPIOPinConfigure(GPIO_PB1_U1TX);
GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 |
UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
//UARTprintf function
UARTStdioConfig(1,115200,SysCtlClockGet());
}
//Distance Sensor Initialization
void distinit(void){
//Setup ADC channel 0, and GPIO E
SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //Setup PE3 Side Sensor
GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //Setup PE2 Front Sensor
ADCHardwareOversampleConfigure(ADC0_BASE, 8); //8 Samples per sequence
//Configure ADC PE3 Side Sensor
ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
ADCSequenceEnable(ADC0_BASE, 1);
//Configure ADC PE2 Front Sensor
ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
ADCSequenceStepConfigure(ADC0_BASE,3,0,ADC_CTL_CH1|ADC_CTL_IE|ADC_CTL_END);
ADCSequenceEnable(ADC0_BASE, 3);
}
//Reflectance Sensor Initialization
void reflectinit(void){
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOE);
}
//PWM initialization
void pwminit(void){
volatile uint32_t ui32Load;
 volatile uint32_t ui32PWMClock;
//PWM Clock 40MHz/64 = 625KHz
SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
//Enable PWM1 and GPIOD modules
SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
SysCtlPeripheralReset(SYSCTL_PERIPH_PWM1);
SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOD);
//PD2 - APhase // PD3 - BPhase
GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_3);
//Write A/B Phase Initialize for forward
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
//Set pin PD0 - AEnable
GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
GPIOPinConfigure(GPIO_PD0_M1PWM0);
//Set pin PD1 - BEnable
GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
GPIOPinConfigure(GPIO_PD1_M1PWM1);
 //The PWM clock is SYSCLK/64 (set in step 12 above).
//Divide the PWM clock by the desired frequency (55Hz)
//Then subtract 1 since the counter down-counts to zero.
ui32PWMClock = SysCtlClockGet() / 64;
//ui32Load = 11632
ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
//Configure module 1 PWM generator 0/1 as a down-counter and load the count value.
PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
//PWM A Enable
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7000);
PWMGenEnable(PWM1_BASE, PWM_GEN_0);
//PWM B Enable
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 7000);
PWMGenEnable(PWM1_BASE, PWM_GEN_1);
//Turn on Motors
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
}
//Used for PWM testing only
void pwm(char m, char s, char s1){
int x = s - '0';
int y = s1 - '0';
int z = 10*x + y;
UARTprintf("\n\n%d\n\n", z);
switch(m){
case 'R':
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, z*121+1);
break;
case 'L':
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, z*121+1);
break;
case 'B':
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, z*121+1);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, z*121+1);
break;
}
}
//Used for testing and demo-ing labs during semester.
//Bluetooth program that responds to user commands over the terminal
void uartcomint(void){
char command[2];
while (1){
 //UARTprintf("\033[2J\033[H");
UARTprintf("Enter Command: ");
UARTgets(command,3);
if (strcmp(command,"ON")==0){
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, GPIO_PIN_3);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 7987);
PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 7987);
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
UARTprintf("MOTORS ON!\n");
}
else if(strcmp(command,"OF")==0){
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
UARTprintf("MOTORS OFF!\n");
}
else if(strcmp(command,"SD")==0){
char strdist[6];
float distvalue = getvalue();
sprintf(strdist,"%.3f", distvalue);
UARTprintf("Distance in cm: %s\n\n",strdist);
 }
else if(strcmp(command,"FD")==0){
char strdist[6];
float distvalue = getfrontdist();
sprintf(strdist,"%.3f", distvalue);
UARTprintf("Distance in cm: %s\n\n",strdist);
}else if(strcmp(command,"LI")==0){
if (isblackline() == true){
UARTprintf("Black\n");
}else{
UARTprintf("White\n");
}
}else if(strcmp(command,"RT")==0){
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
UARTprintf("Right!\n");
delayms(400);
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
}
else if(strcmp(command,"UT")==0){ //Right backwards, left forwards
 PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0x00);
GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_3, 0x00);
UARTprintf("U-Turn\n");
delayms(800);
PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false);
PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);
}
else if(strcmp(command,"SP")==0){
char m,s,s1;
UARTprintf("Enter R, L or B to change motor speed: ");
m=UARTgetc();
UARTprintf("%c\nEnter speed for %c motor (0-99): ", m, m);
s=UARTgetc();
s1=UARTgetc();
UARTprintf("%c%c\n",s,s1);
pwm(m,s,s1);
}
else{
UARTprintf("Not a valid command.\n\n");
}
}//End of While
}
//Delay milliseconds
void delayms(int ms){
SysCtlDelay(ms*(SysCtlClockGet()/3000));
}
//Delay microseconds
void delaymicro(int ms){
SysCtlDelay(ms*(SysCtlClockGet()/3000000));
}
