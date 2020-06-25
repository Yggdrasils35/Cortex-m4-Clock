
#include <stdint.h>
#include <stdbool.h>
#include "hw_memmap.h"
#include "hw_ints.h"
#include "debug.h"
#include "gpio.h"
#include "hw_i2c.h"
#include "hw_types.h"
#include "i2c.h"
#include "pin_map.h"
#include "sysctl.h"
#include "systick.h"
#include "timer.h"
#include "interrupt.h"
#include "uart.h"
#include "adc.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
//I2C GPIO chip address and resigster define
//
//*****************************************************************************
#define TCA6424_I2CADDR 					0x22
#define PCA9557_I2CADDR						0x18

#define PCA9557_INPUT						0x00
#define	PCA9557_OUTPUT						0x01
#define PCA9557_POLINVERT					0x02
#define PCA9557_CONFIG						0x03

#define TCA6424_CONFIG_PORT0			0x0c
#define TCA6424_CONFIG_PORT1			0x0d
#define TCA6424_CONFIG_PORT2			0x0e

#define TCA6424_INPUT_PORT0				0x00
#define TCA6424_INPUT_PORT1				0x01
#define TCA6424_INPUT_PORT2				0x02

#define TCA6424_OUTPUT_PORT0			0x04
#define TCA6424_OUTPUT_PORT1			0x05
#define TCA6424_OUTPUT_PORT2			0x06

#define FASTDELAY 1000000
#define TARGET_IS_TM4C129_RA0 
#define SYSCTL_CFG_VCO_240      0xF1000000
void 	SysTick_Init(void);
void 	SysTick_Handler(void);
void 	PortJ_IntHandler(void);

void 	Delay(uint32_t value);
// 初始化函数
void 	S800_GPIO_Init(void);
void 	S800_I2C0_Init(void);
void 	Time_Init(void);
void 	Timer_Init(void);	// Timer初始化
void 	Timer_Disable(void);// Timer失能
void 	ADC_Init(void);		// ADC初始化

// 显示函数
void 	Display_Time(void);		// mode = 0
void 	Display_led(void);
void 	Display_Day(void);		// mode = 1;
void 	Display_ADC(void);
void 	Display_clock(uint8_t pclock);
void 	Display_TimeSet(void);	// 时间设置时的显示函数
void 	Display_DateSet(void);	// 日期设置时的显示函数
void 	Display(uint32_t num, int isFlag);	// 用于ADC显示

// 设置函数
uint8_t changeMode(void);
void Clock1_Set(void);			// mode = 2
void Clock2_Set(void);			// mode = 3
void Time_Set(void);			// mode = 4
void Date_Set(void);			// mode = 5

// 闹钟检测
void Test_Clock1(void);
void Test_Clock2(void);

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData);
uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr);

volatile uint8_t result;
uint32_t ui32SysClock,Tick=0, g_ui32SysClock;

uint16_t clk = 0;
uint8_t seg7[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c,0x58,0x5e,0x079,0x71,0x5c};	// 数字（包含小数点）

uint8_t reverse_bit(uint8_t value);
uint8_t tmode = 1;	// 12/24小时表示，1为24，0为12
uint8_t mode = 0; 	// 按键状态
uint8_t isClock1 = 0;
uint8_t isClock2 = 0;

// ADC
float  t;
uint32_t pui32ADC0Temp[1];
uint32_t pui32ADC0Volt[1];
uint32_t ui32VoltValue;
uint32_t ui32TempValueC;
uint32_t ui32TempValueF;

// 蜂鸣器
uint8_t idx[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};	// 数码管位数
uint8_t led[9] = {0xFF, 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0XDF, 0xBF, 0x7F};	// 单个led灯
uint8_t leds[9] = {0xFF,0x7F,0x3F,0x1F,0x0F,0x07,0x03,0x01,0x00};	// 多个led灯编码
int monthD[12] = {31,28,31,30,31,30,31,31,30,31,30,31};		// 月份对应天数

uint16_t tone[] = {247,262,294,330,349,392,440,494,523,587,659,698,784,1000};
uint16_t music[] = {5,5,6,8,7,6,5,6,13,13,//音调
	5,5,6,8,7,6,5,3,13,13,
	2,2,3,5,3,5,6,3,2,1,
	6,6,5,6,5,3,6,5,13,13,
	5,5,6,8,7,6,5,6,13,13,
	5,5,6,8,7,6,5,3,13,13,
	2,2,3,5,3,5,6,3,2,1,
	6,6,5,6,5,3,6,1,
	13,8,9,10,10,9,8,10,9,8,6,
	13,6,8,9,9,8,6,9,8,6,5,
	13,2,3,5,5,3,5,5,6,8,7,6,
	6,10,9,9,8,6,5,6,8
};
uint16_t time[] = {2,4,2,2,2,2,2,8,4, 4, //时间
	2,4,2,2,2,2,2,8,4, 4,
	2,4,2,4,2,2,4,2,2,8,
	2,4,2,2,2,2,2,8,4 ,4,
	2,4,2,2,2,2,2,8,4, 4,
	2,4,2,2,2,2,2,8,4, 4,
	2,4,2,4,2,2,4,2,2,8,
	2,4,2,2,2,2,2,8,
	4, 2,2,2, 4, 2,2,2, 2,2,8,
	4, 2,2,2,4,2,2,2,2,2,8,
	4, 2,2,2,4,2,2,5,2,6,2,4,
	2,2 ,2,4,2,4,2,2,12
};
uint16_t m = 0, n = 0;	// 蜂鸣器用寄存器

// 时间结构定义
struct Time
{
	int hour;	// 24hours
	int phour;	// 12hours
	int min;
	int sec;
	int psec;
	bool isPM;
};

struct Time clock;
struct Time clock1;
struct Time clock2;

struct Days
{
	int year;
	int month;
	int day;
}date;

void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	m = !m;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock / tone[music[n]]);
}

void Timer1IntHandler(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	if (music[n]!=13) {
		if (m) GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
		else GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
	}
	n = (n + 1) % 75;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock / time[n]);
}

int main(void)
{
	//use internal 16M oscillator, HSI
	// ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_16MHZ |SYSCTL_OSC_INT |SYSCTL_USE_OSC), 16000000);	// Systick时钟
	#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    //
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 20000000);
	#else
		SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
					   SYSCTL_XTAL_16MHZ);
	#endif
	SysTick_Init();
	S800_GPIO_Init();
	S800_I2C0_Init();
	Time_Init();
	// Timer_Init();
	clock.hour = 13;
	ADC_Init();
	
	while (1)
	{
		mode = changeMode();
		
		switch (mode) {
			case 0: Display_Time();break;
			case 1: Display_Day();break;
			case 2: Clock1_Set();break;
			case 3: Clock2_Set();break;
			case 4: Time_Set();break;
			case 5: Date_Set();break;
			default: Display_Time();
		}
		Test_Clock1();
		Test_Clock2();
		
	}
}

void Delay(uint32_t value)
{
	uint32_t ui32Loop;
	for(ui32Loop = 0; ui32Loop < value; ui32Loop++){};
}

void S800_GPIO_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);						//Enable PortF
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));			//Wait for the GPIO moduleF ready
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);			// Enable PortJ
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));
	
	GPIOIntRegister(GPIO_PORTJ_BASE, PortJ_IntHandler); 
	GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_5);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);			//Set PF0 as Output pin
	GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE,GPIO_PIN_0);			//Set the PJ0 as input pin
	GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
	GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0);
}


void S800_I2C0_Init(void)
{
	uint8_t result;
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	I2CMasterInitExpClk(I2C0_BASE,ui32SysClock, true);										//config I2C0 400k
	I2CMasterEnable(I2C0_BASE);	

	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT0,0x0ff);		//config port 0 as input
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT1,0x0);			//config port 1 as output
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_CONFIG_PORT2,0x0);			//config port 2 as output 

	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_CONFIG,0x00);					//config port as output
	result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,0x0ff);				//turn off the LED1-8
	
}

void Time_Init(void)
{
	date.year = 2020;
	date.month = 6;
	date.day = 22;
}


void ADC_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	// ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE |
							// ADC_CTL_END);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0,  ADC_CTL_TS | ADC_CTL_IE |
                             ADC_CTL_END);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0,  ADC_CTL_CH2 | ADC_CTL_IE |
                             ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntClear(ADC0_BASE, 3);
	ADCSequenceEnable(ADC0_BASE, 1);
	ADCIntClear(ADC0_BASE, 1);
}

uint8_t I2C0_WriteByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t WriteData)
{
	uint8_t rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
		
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE)){};
		
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	I2CMasterDataPut(I2C0_BASE, WriteData);
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_BASE)){};

	rop = (uint8_t)I2CMasterErr(I2C0_BASE);

	return rop;
}

uint8_t I2C0_ReadByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t value,rop;
	while(I2CMasterBusy(I2C0_BASE)){};
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, false);
	I2CMasterDataPut(I2C0_BASE, RegAddr);
	// I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusBusy(I2C0_BASE));
	rop = (uint8_t)I2CMasterErr(I2C0_BASE);
	Delay(100);
	//receive data
	I2CMasterSlaveAddrSet(I2C0_BASE, DevAddr, true);
	I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusBusy(I2C0_BASE));
	value=I2CMasterDataGet(I2C0_BASE);
	Delay(100);
	return value;
}

void PortJ_IntHandler(void)
{
	uint32_t ulStatus;
	ulStatus=GPIOIntStatus(GPIO_PORTJ_BASE,true);
	
	GPIOIntClear(GPIO_PORTJ_BASE,ulStatus);
	if (ulStatus & GPIO_PIN_0) {
		tmode = !tmode;
	}
	
}


uint8_t changeMode(void)
{
	uint8_t tmp, pmode1;	// pmode1检测sw1
	int tsec = 0;
	tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	pmode1 = tmp & 0x01;
	if (tmp & 0x01 && tmp & 0x02) {
		Delay(FASTDELAY);
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x01 && tmp & 0x02) {
			Display_ADC();
			return 0;
		}
	}
	if (pmode1) {
		Delay(FASTDELAY);
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		pmode1 = tmp & 0x01;
		if (pmode1) {
			mode = (mode + 1) % 3;
		}
	}
	return mode;
}


// initialize the systick timer
void  SysTick_Init(void)
{
	SysTickEnable();
	SysTickPeriodSet(ui32SysClock/100);
	SysTickIntEnable();
}


// the interrupt serverce subrotine of systick
void SysTick_Handler(void)
{
	clock.psec++;
	clk++;
	if (clock.hour >= 12) clock.isPM = 1; 
	
	if (clock.psec >= 100) {
		clock.psec = 0;
		clock.sec++;	// 秒数增加
		if (clock.sec >= 60) {
            clock.sec = 0;
            clock.min++;	// 分钟增加
			if (clock.min >= 60) {
				clock.min = 0;
                clock.hour++;		// 小时增加
				if (clock.hour == 24) {
					clock.hour = 0;
					date.day++;		// 天数增加
					if (date.day == monthD[date.month]) {
						date.day = 1;
						date.month = (date.month + 1) % 13;		// 月份增加
					}
				}
			}
		}
	}
	clock.phour = clock.hour % 12;
}

void Display_Time()		// mode = 0
{
	// display the percent seconds
	int x = clock.psec % 10;
	int y = clock.psec / 10;
	if (isClock1 && isClock2) {
		if (clock.psec / 50 == 0 && clock.sec % 2 == 0) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, 0xFC);
		else {
			if (clock.psec / 50 == 0 && clock.sec % 2 == 1) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[1]);
			else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
		}
	}
	else {
		if (isClock1) {
			if (clock.psec / 50 == 0) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[1]);
			else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
		}
		if (isClock2) {
			if (clock.sec % 2 == 0) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[2]);
			else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
		}
	}
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[0]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[1]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));

	// display the seconds
	x = clock.sec % 10;
	y = clock.sec / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[2]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[3]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	// display the minutes
	x = clock.min % 10;
	y = clock.min / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	// display the hours
	if (!tmode) {
		x = clock.phour % 10;
		y = clock.phour / 10;
		if (clock.isPM) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[5]);
	}
	else {
		x = clock.hour % 10;
		y = clock.hour / 10;
		result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
	}
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
}

void Display_Day(void)	// mode = 1
{
	
	// display the day
	int x = date.day % 10;
	int y = date.day / 10;
	int pyear, z, m;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[0]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[1]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));

	// display the month
	x = date.month % 10;
	y = date.month / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[2]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[3]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	// display the year
	pyear = date.year;
	x = pyear % 10;
	pyear /= 10;
	y = pyear % 10;
	pyear /= 10;
	z = pyear % 10;
	m = pyear / 10;
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[z]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[m]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
}


void Display_ADC(void)
{
	// sequence1用作温度检测
	int loop = 0;
	ADCProcessorTrigger(ADC0_BASE, 1);
	while(!ADCIntStatus(ADC0_BASE, 1, false))
	{
	}
	ADCIntClear(ADC0_BASE, 1);
	ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Temp);
	
	// sequence3用做电压检测
	ADCProcessorTrigger(ADC0_BASE, 3);
	while(!ADCIntStatus(ADC0_BASE, 3, false))
	{
	}
	ADCIntClear(ADC0_BASE, 3);
	
	ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Volt);
	
	t = 1475-75*33* pui32ADC0Temp[0]/4096;
	ui32TempValueC=(uint32_t)t * 10;		// 偏于输出四位有效数字
	t = pui32ADC0Volt[0] / 4.096 * 3.3;		// 计算四位电压数值
	ui32VoltValue = (uint32_t)t;
	
	for (loop = 0; loop < 30; ++loop) {
		// 温度
		Display(ui32TempValueC, 0);
		
		// 电压
		Display(ui32VoltValue, 1);
	}
}


void Display_clock(uint8_t pclock)
{
	int x,y;
	if (pclock == 1) {
		// 闪烁显示clock1
		// To be continued
		x = clock1.min % 10;
		y = clock1.min / 10;
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		// display the hours
		x = clock1.hour % 10;
		y = clock1.hour / 10;
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	}
	else {
		// 闪烁显示clock2
		// To be continued
		x = clock2.min % 10;
		y = clock2.min / 10;
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		// display the hours
		x = clock2.hour % 10;
		y = clock2.hour / 10;
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	}
}


void Display_TimeSet(void)
{
	int x, y;
	
	if (clock.psec / 50 == 0) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[3]);
	else  result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
	
	x = clock.min % 10;
	y = clock.min / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	// display the hours
	x = clock.hour % 10;
	y = clock.hour / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
}


void Display_DateSet(void)
{
	int x = date.day % 10;
	int y = date.day / 10;
	
	if (clock.psec / 50 == 0) result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[4]);
	else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[4]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[5]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));

	// display the month
	x = date.month % 10;
	y = date.month / 10;
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[6]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
	
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[y]);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[7]));
	Delay(10000);
	result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
}


void Clock1_Set(void)	// mode = 2
{
	uint8_t tmp;
	
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[1]);
	while (true) {
		Display_clock(1);
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x20) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x20) break; // 按下SW6，闹钟1设置使能
		}
		if (tmp & 0x01) {	   // 按下SW1，进入clock2设置
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x01) {
				mode = 3;
				result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
				return;
			}
		}
	}
	
	tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	if (!isClock1) {
		clock1.min = clock.min;
		clock1.hour = clock.hour;
	}
	while (true) {
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x26) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x02) clock1.min = (clock1.min + 1) % 60;
			if (tmp & 0x04) clock1.hour = (clock1.hour + 1) % 24;
			if (tmp & 0x20) {
				mode = 0;
				isClock1 = 1;
				break;
			}
		}
		Display_clock(1);
	}
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
}

void Clock2_Set(void)	// mode = 3
{
	uint8_t tmp;
	
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[2]);
	
	while (true) {
		Display_clock(2);
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x40) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x40) break; // 按下SW7，闹钟2设置使能
		}
		
		if (tmp & 0x01) {	   // 按下SW1，进入时间设置
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x01) {
				mode = 4;
				result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
				return;
			}
		}
	}
	
	tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
	if (!isClock2) {
		clock2.min = clock.min;
		clock2.hour = clock.hour;
	}
	while (true) {
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x46) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x02) clock2.min = (clock2.min + 1) % 60;
			if (tmp & 0x04) clock2.hour = (clock2.hour + 1) % 24;
			if (tmp & 0x40) {
				mode = 0;
				isClock2 = 1;
				break;
			}
		}
		Display_clock(2);
	}
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
}

void Time_Set(void)		// mode = 4
{
	uint8_t tmp;
	
	while (true) {
		
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x07) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x02) clock.min = (clock.min + 1) % 60;
			if (tmp & 0x04) clock.hour = (clock.hour + 1) % 24;
			if (tmp & 0x01) {
				mode = 5;
				break;
			}
		}
		Display_TimeSet();
	}
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
}


void Date_Set(void)		// mode = 5
{
	uint8_t tmp;
	
	while (true) {
		tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
		if (tmp & 0x19) {
			Delay(FASTDELAY);
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (tmp & 0x08) date.month = (date.month + 1) % 12;
			if (tmp & 0x10) date.day = (date.day + 1) % 31;
			if (tmp & 0x01) {
				mode = -1;
				break;
				}
		}
		Display_DateSet();
	}
	result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
}


void Test_Clock1(void)
{
	if (isClock1 && clock.hour == clock1.hour && clock.min == clock1.min) {
		// led1闪烁
		uint8_t tmp;
		int beginTime = clock.sec;
		
		// Timer_Init();
		
		while(true) {
			GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, GPIO_PIN_5);
			
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (clock.psec / 50 == 0)	{
				result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[1]);
				Display_clock(1);
			}
			else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
			if (clock.sec - beginTime > 10) {	// 最多持续10s
				isClock1 = 0;
				GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
				result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
				break;
			}
			if (tmp != 0) {		// 任意按键结束闹钟
				Delay(FASTDELAY);
				tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
				if (tmp != 0) {
					isClock1 = 0;
					GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_5, 0);
					result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
					break;
				}
			}
		}
	}
}

void Test_Clock2(void)
{
	if (isClock2 && clock.hour == clock2.hour && clock.min == clock2.min) {
		// led2闪烁
		uint8_t tmp;
		int beginTime = clock.sec;
		while(true) {
			tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
			if (clock.sec % 2 == 0)	{
				result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[2]);
				Display_clock(2);
			}
			else result = I2C0_WriteByte(PCA9557_I2CADDR, PCA9557_OUTPUT, led[0]);
			if (clock.sec - beginTime > 10) {	// 最多持续10s
				isClock2 = 0;
				break;
			}
			if (tmp != 0) {		// 任意按键结束闹钟
				Delay(FASTDELAY);
				tmp = ~I2C0_ReadByte(TCA6424_I2CADDR, TCA6424_INPUT_PORT0);
				if (tmp != 0) {
					isClock2 = 0;
					break;
				}
			}
		}
	}
}


void Display_led(void)
{
	uint16_t idx = clk / 100;
	if (idx <= 8) result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,led[idx]);
	else {
		result = I2C0_WriteByte(PCA9557_I2CADDR,PCA9557_OUTPUT,~led[idx-8]);	
	}
	if (idx >= 16) clk = 0;
}

void Display(uint32_t num, int isFlag)
{
	int i = 0;
	int j = 0;
	while(num >= 1) {
		int x = num % 10;
		if (i == 2+isFlag) {
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]+0x80);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[i+isFlag*4]));
		}
		else {
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[x]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[i+isFlag*4]));
		}
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		num /= 10;
		i++;
	}
	// 当电压小于0时，表示小数
	if (i < 4 && isFlag) {
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]+0x80);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[3+isFlag*4]));
		Delay(10000);
		result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		for (j = 2; j >= i; --j) {
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT1,seg7[0]);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(idx[j+isFlag*4]));
			Delay(10000);
			result = I2C0_WriteByte(TCA6424_I2CADDR,TCA6424_OUTPUT_PORT2,(uint8_t)(0));
		}
	}		
}
uint8_t reverse_bit(uint8_t value)
{
    uint8_t num = 0;
    int i = 0;
    for(i=7;i>=0;i--)
    {
			num |= (( value % 2 ) << i );
			value >>= 1;
    }
    return num;
}
