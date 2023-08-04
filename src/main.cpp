#include "Energia.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "../lib/arduinoFFT/src/arduinoFFT.h"
//#include "driverlib/gpio.h"
//#include <stdint.h>

//#define InterruptPinA 39
//#define InterruptPinB 40

#define LED_RED   30
#define LED_BLUE  40
#define LED_GREEN 39

#define SYSCTL_PERIPH_TIMERX  SYSCTL_PERIPH_TIMER2      //TIMERx 时钟
#define TIMERX_BASE           TIMER2_BASE               //TIMERx 地址
#define TIMER_AorB            TIMER_A
#define INT_TIMERXAorB        INT_TIMER2A               //TIMER  中断源
//TIMERx 中断响应函数
#define TimerXIntHandler      Timer2IntHandler
#define TIMER_CFG_X           TIMER_CFG_PERIODIC        //TIMER  配置方式
#define TIMER_TIMAorB_INTx    TIMER_TIMA_TIMEOUT        //TIMER  中断方式
#define TIMER_INTERVAL_NUM    1024000                   //TIMER  中断分频系数
#define TimerLoadSetNUM       (((SysCtlClockGet() / TIMER_INTERVAL_NUM)) -1)


// A、B 波形结构体
typedef struct {
    int frequency;
    bool sin;
} WAVE;

// 波形参数
WAVE wave[2];
uint16_t waveNum = 0;

// FFT 参数
arduinoFFT FFT;
const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1024000;
/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];


// 定时器参数
uint8_t num;
uint16_t admem[1024];
uint16_t admemNum = 0;
bool state = false;   // 是否采集完成

// 按键控制开关参数
//volatile bool state = 0;

// 外部中断
//void myISR(){
//    state = !state;
//}

uint16_t numX = 0;

// 显示/波赋值
void PrintVector(double *vData, uint16_t bufferSize) {
//    for (uint16_t i = 0; i < bufferSize; i++) {
//        double abscissa = (i * 1.0 * samplingFrequency) / samples / 1000;
//        Serial.print(abscissa, 6);
//        Serial.print("kHz ");
//        auto tmp1 = (uint16_t) ceil(abscissa / 5) * 5;
//        Serial.print(tmp1);
//        Serial.print("kHz ");
//        Serial.print(vData[i], 4);
//        Serial.print("   ");
//        uint32_t tmp0 = ((uint32_t) vData[i]) >> 10;
//        Serial.println(tmp0);
//    }
//    Serial.println();

    uint32_t tmp[bufferSize];
    for (uint16_t i = 0; i < bufferSize; i++) {
        tmp[i] = ((uint32_t) vData[i]) >> 10;
    }

    uint16_t Aa, Ab;
    for (uint16_t i = 0; i < bufferSize; i++) {
        auto hz = ((i * 1.0 * samplingFrequency) / samples / 1000);
        auto hz_ture = (uint16_t) ceil(hz / 5) * 5;
        if (hz > 150 ){
            numX += (tmp[i] >= 9);   // ???????????
        }
        if (hz >= 15 && tmp[i] > 150 && (waveNum == 0 || hz_ture != wave[0].frequency)) {
            wave[waveNum].frequency = hz_ture;
//            wave[waveNum].sin = tmp[i]>180;
            if (waveNum == 0) {
                Aa = tmp[i] > tmp[i + 1] ? tmp[i] : tmp[i + 1];
            } else {
                Ab = tmp[i] > tmp[i + 1] ? tmp[i] : tmp[i + 1];
                if (Aa - Ab > 20) {
                    wave[0].sin = true, wave[1].sin = false;
                } else if (Ab - Aa > 20) {
                    wave[0].sin = false, wave[1].sin = true;
                } else if (numX >= 2) {
                    wave[0].sin = false, wave[1].sin = false;
                } else {
                    Serial.println(numX);
                    wave[0].sin = true, wave[1].sin = true;
                }
            }
            numX = 0;

//            for (int j = 3 * i - 2; j <= i * 3 + 2; ++j) {
////                if ((tmp[j - 2] < 10 || tmp[j + 2] < 10) && (tmp[j]/tmp[j-2] >= 3)) {
//                if (tmp[j]>=10 && tmp[j]/tmp[j-2] >= 3) {
//                    wave[waveNum].sin = false;
//                    break;
//                } else {
//                    wave[waveNum].sin = true;
//                }
//            }
            if (++waveNum >= 2) {
                waveNum = 0;
                break;
            }
        }
    }


    Serial.print(wave[0].frequency);
    Serial.print("kHz   sin ");
    Serial.println(wave[0].sin);
    Serial.print(wave[1].frequency);
    Serial.print("kHz   sin ");
    Serial.println(wave[1].sin);
    Serial.println("\n");
}


// 定时器中断工作内容
void TimerXIntHandler(void) {
    // Clear the timer interrupt
    //GPIO_PORTF_DATA_R = 0xff; // 置位PF3引脚,执行时间约25ns,上升到VCC电瓶约需要90ns
    //TimerIntClear(TIMERX_BASE, TIMER_TIMAorB_INTx); //执行时间约100ns
    //GPIO_PORTF_DATA_R = 0xff; // 置位PF3引脚,执行时间约25ns,上升到VCC电瓶约需要90ns
    TIMER2_ICR_R |= TIMER_TIMAorB_INTx;  // 清除定时器0的定时器A超时中断标志位，执行时间约60ns
    //GPIO_PORTF_DATA_R = 0x00; // 置位PF3引脚,执行时间约25ns,上升到VCC电瓶约需要90ns
    // Toggle the pin state
    //digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
    //digitalWrite(LED_GREEN, 1);   //执行时间约620ns
    //digitalWrite(LED_GREEN, 0);   //执行时间约620ns
    //GPIO_PORTF_DATA_R = num++; // 置位PF3引脚
    //GPIO_PORTF_DATA_R = 0xff; // 置位PF3引脚,执行时间约25ns,上升到VCC电瓶约需要90ns
    //GPIO_PORTF_DATA_R = 0x00; // 置位PF3引脚

    while ((ADC0_RIS_R & 0x08) == 0) {} // 等待转换完成
    admem[admemNum++] = ADC0_SSFIFO3_R & 0xFFF;
    ADC0_PSSI_R = 0x08;         // 开始转换，SS3

    if (admemNum >= 1024) {
        admemNum = 0;
//        digitalWrite(LED_RED, !digitalRead(LED_RED));
        state = true;
    }

    //digitalWrite(LED_RED, !digitalRead(LED_RED));
}


// 定时器初始化
void TimerInt_Init() {
//    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMERX);   //使能定时器0时钟
    TimerConfigure(TIMERX_BASE,
                   TIMER_CFG_X);   //定时器的高层配置，这里设为全字长周期定时
    //ui32Period = (SysCtlClockGet() / TIMER_INTERVAL_NUM) / 2;	  //SysCtlClockGet()返回系统时钟频率（也是除了PWM外其他模块的频率）
    TimerLoadSet(TIMERX_BASE, TIMER_AorB,
                 TimerLoadSetNUM);   //设置定时器重装载值（第二个参数指定的要调整的定时器，全字长周期只能写TIMER_A）
    TimerIntRegister(TIMERX_BASE, TIMER_AorB,
                     TimerXIntHandler);   //注册中断，将中断触发和中断服务函数联系起来（实质上是修改.s中的中断向量表，第二个参数是函数指针，也可以手动注册）
    TimerIntEnable(TIMERX_BASE,
                   TIMER_TIMAorB_INTx);   //使能单个的定时器中断源
    IntEnable(
            INT_TIMERXAorB);   //使能中断
    IntMasterEnable();   //这个函数允许处理器响应中断。不影响中断控制器的中断使能设置
    TimerEnable(TIMERX_BASE, TIMER_AorB);
}


void setup() {

//    pinMode(OPEN_PIN,INPUT);
//    attachInterrupt(2, myISR, CHANGE);

    wave[0] = {0, false};
    wave[1] = {0, false};

    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);

    digitalWrite(LED_RED, HIGH);

    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // 解锁Port F引脚
    GPIO_PORTF_CR_R = 0x1F;            // 允许更改PF4-PF0引脚

    GPIO_PORTF_AMSEL_R = 0x00;         // 禁用模拟功能
    GPIO_PORTF_PCTL_R = 0x00000000;    // 使用GPIO功能
    GPIO_PORTF_DIR_R |= 0xFF;          // 设置PF引脚为输出
    GPIO_PORTF_AFSEL_R = 0x00;         // 禁用替代功能
    GPIO_PORTF_DEN_R |= 0xFF;          // 使能数字功能
    // 配置PF3引脚为高速模式，8mA Slew Rate Control 模式
    GPIO_PORTF_DR2R_R |= 0xFF;       // 设置引脚驱动强度为8mA
    GPIO_PORTF_SLR_R |= 0xFF;        // 启用8mA Slew Rate Control

    SYSCTL_RCGCADC_R |= 0x01;   // 启用ADC0模块的时钟
    SYSCTL_RCGCGPIO_R |= 0x10;  // 启用Port E模块的时钟
    // 配置ADC通道对应的GPIO引脚为模拟输入
    GPIO_PORTE_DIR_R &= ~0x08;  // PE3作为输入
    GPIO_PORTE_AFSEL_R |= 0x08; // PE3允许使用替代功能
    GPIO_PORTE_DEN_R &= ~0x08;  // PE3不需要数字功能
    GPIO_PORTE_AMSEL_R |= 0x08; // PE3允许模拟功能

    ADC0_ACTSS_R &= ~0x0008;    // 关闭SS3
    ADC0_EMUX_R &= ~0xF000;     // 使用软件触发模式
    ADC0_SSMUX3_R = 0;          // 选择ADC通道0
    ADC0_SSCTL3_R = 0x0006;     // 使用温度传感器配置
    ADC0_ACTSS_R |= 0x0008;     // 打开SS3

    ADC0_PSSI_R = 0x08;         // 开始转换，SS3

    Serial.begin(1000000);
//    pinMode(CHANNEL_A,OUTPUT);
//    pinMode(CHANNEL_B,OUTPUT);
//    Serial.println(SysCtlClockGet());

//    pinMode(InterruptPinA, OUTPUT);
//    digitalWrite(InterruptPinA, 1);
//    pinMode(InterruptPinB, OUTPUT);
//    digitalWrite(InterruptPinB, 1);
//    for (int i = 11, j = 31; i <= 16; ++i, ++j) {
//        pinMode(i, OUTPUT);
//        digitalWrite(i, 0);
//        pinMode(j, OUTPUT);
//        digitalWrite(j, 0);
//    }

    TimerInt_Init();

}


void loop() {
    if (state) {
        state = false;
        for (uint16_t i = 0; i < samples; i++) {
            vReal[i] = admem[i];/* Build data with positive and negative values*/
            //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
            vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
        }
        FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
        /* Print the results of the simulated sampling according to time */
        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);    /* Weigh data */
        FFT.Compute(FFT_FORWARD); /* Compute FFT */
        FFT.ComplexToMagnitude(); /* Compute magnitudes */
        PrintVector(vReal, (samples >> 1));
    }

}

