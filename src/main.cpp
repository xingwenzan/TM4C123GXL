// #include "Energia.h"
#include "arduinoFFT.h"
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include <MD_AD9833.h>
#include <SPI.h>


// #include "../lib/arduinoFFT/src/arduinoFFT.h"


#define LED_RED   30
#define LED_BLUE  40
#define LED_GREEN 39
#define GPIO_LED_BASE GPIO_PORTF_BASE

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
// #define SCL_INDEX 0x00
// #define SCL_TIME 0x01
// #define SCL_FREQUENCY 0x02
// #define SCL_PLOT 0x03
#define PUSH1_PIN 31
#define PUSH2_PIN 17
#define OPEN_PIN 15
#define DATA1 13  ///< SPI Data pin number for AD1
#define CLK1 12   ///< SPI Clock pin number for AD1
#define FSYNC1 11 ///< FSYNC pin number for AD1
#define DATA2 34  ///< SPI Data pin number for AD2
#define CLK2 33    ///< SPI Clock pin number for AD2
#define FSYNC2 32  ///< FSYNC pin number for AD2


// 幅频结构体
typedef struct {
    uint16_t KHZ;
    uint32_t A;
} AF;

// A、B 波形结构体
typedef struct {
    int frequency;
    bool sin;
} WAVE;

// 排序、波形参数
AF q[17];
uint16_t afNum = 0;
WAVE A, B;
/*TIMER_B 未完成，勿用！！
#define SYSCTL_PERIPH_TIMERX  SYSCTL_PERIPH_TIMER0
#define TIMERX_BASE           TIMER0_BASE
#define TIMER_AorB            TIMER_B
#define INT_TIMERXAorB        INT_TIMER0B
#define TimerXIntHandler      Timer0IntHandler
#define TIMER_CFG_X           TIMER_CFG_PERIODIC
#define TIMER_TIMAorB_INTx    TIMER_TIMB_TIMEOUT
#define TIMER_INTERVAL_NUM    10
#define TimerLoadSetNUM       ((SysCtlClockGet() / TIMER_INTERVAL_NUM) / 2) -1
*/

// FFT 参数
arduinoFFT FFT;
MD_AD9833 AD1(DATA1, CLK1, FSYNC1);
MD_AD9833 AD2(DATA2, CLK2, FSYNC2);
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

// 排序
void sort(int l, int r) {
    if (l >= r) {
        return;
    }
    int i = l - 1, j = r + 1;
    uint32_t x = q[(i + j) >> 1].A;
    while (i < j) {
        while (q[++i].A > x) {}
        while (q[--j].A < x) {}
        if (i < j) {
            AF tmp = q[i];
            q[i] = q[j], q[j] = tmp;
        }
    }
    sort(l, j);
    sort(j + 1, r);
}

// 判断是否为正弦波
bool isSin(WAVE x) {
    return vReal[x.frequency * 3] < 5e3;
}

// 显示/波赋值
void PrintVector(double *vData, uint16_t bufferSize) {
    //  for (uint16_t i = 0; i < bufferSize; i++) {
    //      double abscissa;
    //      abscissa = ((i * 1.0 * samplingFrequency) / samples);
    //     //  if(abscissa >= 296000 && abscissa<=303000){
    //      Serial.print(abscissa, 6);
    //      Serial.print("kHz ");
    //      Serial.println(vData[i], 4);
    //     //  }

    //  }
    //   Serial.println();
    for (uint16_t i = 0; i < bufferSize; i++) {
        uint16_t hz = (uint16_t) ((i * 1.0 * samplingFrequency) / samples / 1000);
        if (hz >= 20 && hz % 5 == 0) {
            q[afNum++] = {hz, (uint32_t) vData[i]};
            if (hz >= 100 || afNum >= 17) {
                afNum = 0;
                break;
            }
        }
// //        Serial.println(q[i].A);
    }

    sort(0, 16);

    //  Serial.println(q[1].KHZ);
    //  Serial.println(q[2].KHZ);

    if (q[0].KHZ > q[1].KHZ) {
        B.frequency = q[0].KHZ, A.frequency = q[1].KHZ;
    } else {
        B.frequency = q[1].KHZ, A.frequency = q[0].KHZ;
    }

    A.sin = isSin(A);
    B.sin = isSin(B);

    Serial.print(A.frequency);
    Serial.print("kHz   sin ");
    Serial.println(A.sin);
    Serial.print(B.frequency);
    Serial.print("kHz   sin ");
    Serial.println(B.sin);
    Serial.println();
}

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
        digitalWrite(LED_RED, !digitalRead(LED_RED));
        for (uint16_t i = 0; i < samples; i++) {
            vReal[i] = admem[i];/* Build data with positive and negative values*/
            //vReal[i] = uint8_t((amplitude * (sin((i * (twoPi * cycles)) / samples) + 1.0)) / 2.0);/* Build data displaced on the Y axis to include only positive values*/
            vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
        }

        FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
        /* Print the results of the simulated sampling according to time */
//        Serial.println("Data:");
//        PrintVector(vReal, samples);
        FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);    /* Weigh data */
//        Serial.println("Weighed data:");
//        PrintVector(vReal, samples);
        FFT.Compute(FFT_FORWARD); /* Compute FFT */
//        Serial.println("Computed Real values:");
//        PrintVector(vReal, samples);
//        Serial.println("Computed Imaginary values:");
//        PrintVector(vImag, samples);
        FFT.ComplexToMagnitude(); /* Compute magnitudes */
//        Serial.println("Computed magnitudes:");
        PrintVector(vReal, (samples >> 1));
//        int freq1 = A.frequency*1000;
//        int freq2 = B.frequency*1000;
//        AD1.setFrequency(MD_AD9833::CHAN_0, freq1);
//        AD2.setFrequency(MD_AD9833::CHAN_0, freq2);
        // double x = FFT.MajorPeak();
        // Serial.println(x, 6);
        // 这行代码会停止 Timer0A。你可以更改参数以适应你的需要。第一个参数是你要控制的定时器的基地址（如 TIMER0_BASE，TIMER1_BASE 等），第二个参数是你要控制的定时器（如 TIMER_A，TIMER_B）
        TimerDisable(TIMER0_BASE, TIMER_A);

    }

    //digitalWrite(LED_RED, !digitalRead(LED_RED));
}

void TimerInt_Init(void) {
    uint32_t ui32Period;

    SysCtlPeripheralEnable(
            SYSCTL_PERIPH_TIMERX);                                                                    //使能定时器0时钟
    TimerConfigure(TIMERX_BASE,
                   TIMER_CFG_X);                                                                        //定时器的高层配置，这里设为全字长周期定时
    //ui32Period = (SysCtlClockGet() / TIMER_INTERVAL_NUM) / 2;											//SysCtlClockGet()返回系统时钟频率（也是除了PWM外其他模块的频率）
    TimerLoadSet(TIMERX_BASE, TIMER_AorB,
                 TimerLoadSetNUM);                                                    //设置定时器重装载值（第二个参数指定的要调整的定时器，全字长周期只能写TIMER_A）
    TimerIntRegister(TIMERX_BASE, TIMER_AorB,
                     TimerXIntHandler);                                        //注册中断，将中断触发和中断服务函数联系起来（实质上是修改.s中的中断向量表，第二个参数是函数指针，也可以手动注册）
    TimerIntEnable(TIMERX_BASE,
                   TIMER_TIMAorB_INTx);                                                            //使能单个的定时器中断源
    IntEnable(
            INT_TIMERXAorB);                                                                                                        //使能中断
    IntMasterEnable();                                                                                                                        //这个函数允许处理器响应中断。不影响中断控制器的中断使能设置
    TimerEnable(TIMERX_BASE, TIMER_AorB);
}

void setup() {
    A = {0, false};
    B = {0, false};

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

    Serial.begin(9600);

    Serial.println(SysCtlClockGet());

    TimerInt_Init();

    pinMode(PUSH1_PIN, INPUT_PULLUP);
    pinMode(PUSH2_PIN, INPUT_PULLUP);
    pinMode(OPEN_PIN, INPUT_PULLUP);
    AD1.begin();
    AD2.begin();
    AD1.setMode(MD_AD9833::MODE_SINE);
    AD2.setMode(MD_AD9833::MODE_SINE);
}

void loop() {
    int freq1 = A.frequency * 1000;
    int freq2 = B.frequency * 1000;
    AD1.setFrequency(MD_AD9833::CHAN_0, freq1);
    AD2.setFrequency(MD_AD9833::CHAN_0, freq2);
}


