#include "DSP28x_Project.h"
#include "math.h"
#include "oled.h"

//*****************************************//
//*******************ACAC******************//
//*****************************************//

//**********函数声明**********//
__interrupt void epwm1_isr(void);
__interrupt void adc_isr(void);

void InitADC();
void Init_KEY();
char KEY_Scan(char mode);
void KEY_Control(int key);
void OLED_output();

void EPWM1_Init(void);
void EPWM2_Init(void);
void EPWM3_Init(void);
void EPWM4_Init(void);
void EPWM5_Init(void);
void EPWM6_Init(void);
void EPWM7_Init(void);
void EPWM8_Init(void);

void PID1_Init();
float PID1_Cal(float u);
void PID2_Init();
float PID2_Cal(float u);

float PID_FirstOrderFilter1(float data);
float PID_FirstOrderFilter2(float data);
float PID_FirstOrderFilter3(float data);
float PID_FirstOrderFilter4(float data);

void PLL1(float UI);
float Integral_Cal1();
float Integral_Cal2();

//***********按键变量**********//
#define KEY_H1 (GpioDataRegs.GPBDAT.bit.GPIO56)
#define KEY_H2 (GpioDataRegs.GPBDAT.bit.GPIO54)
#define KEY_H3 (GpioDataRegs.GPBDAT.bit.GPIO39)
#define KEY_H4 (GpioDataRegs.GPADAT.bit.GPIO19)

#define KEY1_PRESS 1
#define KEY2_PRESS 2
#define KEY3_PRESS 3
#define KEY4_PRESS 4
#define KEY_UNPRESS 0
int key = 0;
int N_key = 0;

//**********Flash变量**********//
extern Uint16 RamfunstLoadStart;
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

//**********PID配置**********//
typedef struct {
    float Ref;         // 参考值
    float Xin;         // 输入值
    float Err;         // 误差值
    float Err_last;    // 上一次误差值
    float Kp, Ki, Kd;  // 比例 积分 微分系数
    float result;      // 计算结果
    float Integral;    // 积分值
    float frac;        // 返回值小数部分积分项
} pidsettings;

pidsettings pid1;
pidsettings pid2;

void PID_Init1(void);
void PID_Init2(void);
float PID_Cal1(float u);
float PID_Cal2(float u);

//**********基础变量**********//
#define EPWM_TIMER_TBPRD 2250  // EPWM周期(开关频率)  频率=90MHz/2/TBPRD
#define pi 3.1415926
int EPWM1_MAX_CMP = EPWM_TIMER_TBPRD * 0.95;
int EPWM1_MIN_CMP = EPWM_TIMER_TBPRD * 0.05;

float U_REF = 24;   // 目标电压有效值
float I_REF = 1.3;  // 目标电流有效值

int Sin_Resolution = 180;  // sin表精度
int sin_len = 180;
float sinx1[180];  // 采样用队列
float sinx2[180];  // 采样用队列

int for_count = 0;
int epwm_count = 0;
int tag = 0;
int t_n = 0;
int tag_last = 0;
int tag_list[4] = {0, 1, 0, 1};

float a = 0;
float sum1 = 0;
float sum2 = 0;
float U_in = 0;   // 输入电压
float I_in = 0;   // 输入电流
float U_out = 0;  // 输出电压
float I_out = 0;  // 输出电流
float I_L = 0;    // 电感电流
float U_RMS = 0;  // 电压有效值
float I_RMS = 0;  // 电流有效值

float U_near0 = 5;       // 零点死区电压范围
float Duty_Cycle = 0.4;  // 占空比

//*************锁相环参数*************//
float W0 = 314.159;  // 50HZ的角频率 100pi              //290.159
float W1 = 314.159;
float W1_pi = 0;
float Ts = 0.0002;
float k = 1;
float x = 0;
float Ua = 0;
float Ub = 0;
float Ud = 0;
float Uq = 0;
float theta = 0;  // 经过PLL后获得的相位信息
float Uout = 0;
float Iout_pwm = 0;
float t_err1 = 0;
float t_err2 = 0;

float kp = 250;
float ki = 0.1;
float err = 0;
float last_err = 0;
float result = 0;

float av = 0;
float su = 0;
int c = 0;

int g_j = 0;
int g_i = 0;
int g_k = 0;

//*******************主函数*******************//
void main(void) {
    InitSysCtrl();
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();

    // EALLOW;
    // PieVectTable.EPWM1_INT = &epwm1_isr;
    // EDIS;

    //    #ifdef FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    InitFlash();
    //    #endif

    // Flash配置
    // 开启:注释两行井号语句 文件启用F28069.cmd 关停28069_RAM_lnk.cmd
    // 关闭:启用两行井号语句 文件关停F28069.cmd 启停28069_RAM_lnk.cmd

    Init_KEY();  // 默认GPIO口已配置正常,可不使用
    EPWM1_Init();
    EPWM2_Init();
    EPWM3_Init();
    //       EPWM4_Init();      //暂时禁用
    //       EPWM5_Init();
    //       EPWM6_Init();
    //       EPWM7_Init();
    //       EPWM8_Init();
    InitADC();
    PID1_Init();
    PID2_Init();
    OLED_Init();

    // IER |= M_INT3;                      // 启用 CPU 级中断组3（ePWM1）
    // PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // 启用 PIE 组3中断1（ePWM1）

    for (for_count = 0; for_count < sin_len; for_count++)
        sinx1[for_count] = 0;
    for (for_count = 0; for_count < sin_len; for_count++)
        sinx2[for_count] = 0;

    OLED_ShowString(0, 0, "OFF Grid");
    OLED_ShowString(0, 1, "Uref");
    OLED_ShowString(0, 2, "D1");
    OLED_ShowString(0, 3, "TAG");

    EINT;
    ERTM;

    //*******************主循环*******************//
    for (;;) {
        OLED_output();
    }
}

//*******************ADC中断*******************//
__interrupt void adc_isr(void) {
    U_in = (AdcResult.ADCRESULT0 - 2038.2) / 33.447;  // A0
    U_out = (AdcResult.ADCRESULT1 - 2044.6) / 33.75;  // A1
    I_L = (AdcResult.ADCRESULT6 - 2068.7) / 321.5;    // B0
    I_L = PID_FirstOrderFilter3(I_L);

    tag_last = tag;
    if (tag == 2)  // 并网工作模式
    {
        PLL1(U_in);                                                       // 采集当前输入电压相位theta
        pid2.Ref = fabs(I_REF * 1.4142 * sin(theta) / (1 - Duty_Cycle));  // 计算电感电流目标值
        Duty_Cycle += PID2_Cal(fabs(I_L));                                // 闭环PID 输入当前电流
        Duty_Cycle = PID_FirstOrderFilter4(Duty_Cycle);                   // 滤波
    } else if (tag == 1)                                                  // 离网工作模式
    {
        for (for_count = 0; for_count < sin_len - 1; for_count++)  // 队列，用于记录一周期长度内的离散正弦电压采样值
            sinx1[for_count] = sinx1[for_count + 1];
        sinx1[sin_len - 1] = U_out;
        U_RMS = Integral_Cal1();               // 离散积分计算 得有效值
        U_RMS = PID_FirstOrderFilter1(U_RMS);  // 滤波
        Duty_Cycle += PID1_Cal(U_RMS);         // 闭环PID
    } else if (tag == 0)                       // 全关断模式
    {
        Duty_Cycle = 0.4;
        U_REF = 24;
        I_REF = 1.3;
    }

    if ((tag == 1) || (tag == 2))  // 工作模式输出EPWM波
    {
        if (Duty_Cycle <= 0.03)
            Duty_Cycle = 0.03;  // 大小限制
        if (Duty_Cycle >= 0.65)
            Duty_Cycle = 0.65;
        if (U_in > U_near0)  // 正半周期
        {
            EPwm1Regs.DBCTL.bit.POLSEL = 2;                            // 1A 1B互补
            EPwm1Regs.CMPA.half.CMPA = Duty_Cycle * EPWM_TIMER_TBPRD;  // 1A 1B高频
            EPwm2Regs.DBCTL.bit.POLSEL = 0;                            // 2A 2B相同
            EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD;               // 2A 2B常通
        } else if (U_in < -U_near0)                                    // 负半周期
        {
            EPwm2Regs.DBCTL.bit.POLSEL = 2;                            // 2A 2B互补
            EPwm2Regs.CMPA.half.CMPA = Duty_Cycle * EPWM_TIMER_TBPRD;  // 2A 2B高频
            EPwm1Regs.DBCTL.bit.POLSEL = 0;                            // 1A 1B相同
            EPwm1Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD;               // 1A 1B常通
        } else                                                         // 零点±区间
        {
            EPwm1Regs.DBCTL.bit.POLSEL = 2;                            // 1A 1B互补
            EPwm2Regs.DBCTL.bit.POLSEL = 2;                            // 2A 2B互补
            EPwm1Regs.CMPA.half.CMPA = Duty_Cycle * EPWM_TIMER_TBPRD;  // 1A 1B高频
            EPwm2Regs.CMPA.half.CMPA = Duty_Cycle * EPWM_TIMER_TBPRD;  // 2A 2B高频
        }
    } else  // 非工作模式全关断
    {
        EPwm1Regs.DBCTL.bit.POLSEL = 0;  // 1A 1B相同
        EPwm1Regs.CMPA.half.CMPA = 0;    // 1A 1B常断
        EPwm2Regs.DBCTL.bit.POLSEL = 0;  // 2A 2B相同
        EPwm2Regs.CMPA.half.CMPA = 0;    // 2A 2B常断
    }
    pid1.Ref = U_REF;

    N_key++;
    if (N_key >= 8000)  // 按键分频检测
    {
        N_key = 0;
        key = KEY_Scan(0);
        KEY_Control(key);
    }

    //    su += AdcResult.ADCRESULT0;      //采样公式计算220
    //    c++;
    //    if (c==1000)
    //    {
    //         av = su/1000.0;
    //         su = 0;
    //         c = 0;
    //    }

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;    // Clear ADCINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // Acknowledge interrupt to PIE
}

//*******************锁相环函数*******************//
void PLL1(float UI) {  // 广义二阶积分SOGI
    x = UI - Ua;
    Ua = Ua + (x * k - Ub) * W0 * Ts;                          // α轴
    Ub = Ub + Ua * W0 * Ts;                                    // β轴
    Ud = cos(theta) * Ua + sin(theta) * Ub;                    // d轴
    Uq = cos(theta + t_err2) * Ub - sin(theta + t_err2) * Ua;  // q轴

    err = 0 - Uq;  // PI调节
    result = kp * err - ki * (err - last_err);
    last_err = err;
    if (fabs(result) < 0.05)
        result = 0;

    W1 -= result;
    if (W1 >= 115 * pi)
        W1 = 115 * pi;  // 361
    if (W1 <= 85 * pi)
        W1 = 85 * pi;  // 267
    W1_pi = W1 / 2 / pi;

    theta += W1 * Ts;
    if (theta >= 2 * pi)
        theta -= 2 * pi;
}

//*******************OLED函数*******************//
void OLED_output() {
    OLED_ShowFloat(5, 1, pid1.Ref, 3);
    OLED_ShowFloat(3, 2, Duty_Cycle, 3);
    OLED_ShowNum(4, 3, tag, 1);
}

//*******************积分函数*******************//
float Integral_Cal1() {
    sum1 = 0;
    for (for_count = 0; for_count < sin_len; for_count++)
        sum1 += sinx1[for_count] * sinx1[for_count];
    return sqrt(sum1 / sin_len);
}

float Integral_Cal2() {
    sum2 = 0;
    for (for_count = 0; for_count < sin_len; for_count++)
        sum2 += sinx2[for_count] * sinx2[for_count];
    return sqrt(sum2 / sin_len);
}

//*******************PID1初始化*******************//
void PID1_Init() {
    pid1.Xin = 0;
    pid1.Ref = U_REF;
    pid1.Err = 0;
    pid1.Err_last = 0;
    pid1.Kp = 0.00005;
    pid1.Ki = 0;
    pid1.Kd = 0;
    pid1.result = 0;
    pid1.Integral = 0;
    pid1.frac = 0;
}

//*******************PID1计算*******************//
float PID1_Cal(float u) {
    pid1.Xin = u;
    pid1.Err = pid1.Ref - pid1.Xin;
    //    pid1.Err = pid1.Xin - pid1.Ref;
    pid1.Integral += pid1.Err;
    if (pid1.Integral > 15) {
        pid1.Integral = 15;
    } else if (pid1.Integral < -15) {
        pid1.Integral = -15;
    }

    pid1.result = pid1.Kp * pid1.Err + pid1.Ki * pid1.Integral + pid1.Kd * (pid1.Err - pid1.Err_last);
    pid1.Err_last = pid1.Err;

    if (fabs(pid1.Err) > 0.03) {
        return pid1.result;
    } else {
        return 0;
    }
}

//*******************PID2初始化*******************//
void PID2_Init() {
    pid2.Xin = 0;
    pid2.Ref = I_REF;
    pid2.Err = 0;
    pid2.Err_last = 0;
    pid2.Kp = 0.00015;  // 0.0007;
    pid2.Ki = 0;
    pid2.Kd = 0;
    pid2.result = 0;
    pid2.Integral = 0;
    pid2.frac = 0;
}

//*******************PID2计算*******************//
float PID2_Cal(float u) {
    pid2.Xin = u;
    pid2.Err = pid2.Ref - pid2.Xin;
    //    pid2.Err = pid2.Xin - pid2.Ref;
    pid2.Integral += pid2.Err;
    if (pid2.Integral > 20) {
        pid2.Integral = 20;
    } else if (pid2.Integral < -20) {
        pid2.Integral = -20;
    }

    pid2.result = pid2.Kp * pid2.Err + pid2.Ki * pid2.Integral + pid2.Kd * (pid2.Err - pid2.Err_last);
    pid2.Err_last = pid2.Err;

    if (fabs(pid2.Err) > 0.03) {
        return pid2.result;
    } else {
        return 0;
    }
}

//*******************一阶滤波*******************//
float f1 = 0.4;
float f2 = 0.4;
float f3 = 0.19966;
float f4 = 0.98429;
float datalast1 = 0;
float datalast2 = 0;
float datalast3 = 0;
float datalast4 = 0;
float PID_FirstOrderFilter1(float data) {
    datalast1 = f1 * data + (1 - f1) * datalast1;
    return datalast1;
}

float PID_FirstOrderFilter2(float data) {
    datalast2 = f2 * data + (1 - f2) * datalast2;
    return datalast2;
}

float PID_FirstOrderFilter3(float data) {
    datalast3 = f3 * data + (1 - f3) * datalast3;
    return datalast3;
}

float PID_FirstOrderFilter4(float data) {
    datalast4 = f4 * data + (1 - f4) * datalast4;
    return datalast4;
}

//*******************按键控制函数*******************//
void KEY_Control(int key) {
    switch (key) {
        case KEY1_PRESS:
            if (tag == 1)
                U_REF += 0.5;
            if (tag == 2)
                I_REF += 0.01;
            break;

        case KEY2_PRESS:
            if (tag == 1)
                U_REF -= 0.5;
            if (tag == 2)
                I_REF -= 0.01;
            break;

        case KEY3_PRESS:
            t_n = (t_n + 1) % 4;
            tag = tag_list[t_n];
            break;

        case KEY4_PRESS:
            break;

        case KEY_UNPRESS:
            break;
    }
    if (U_REF > 32)
        U_REF = 32;
    if (U_REF < 1)
        U_REF = 1;

    if (I_REF > 2)
        I_REF = 2;
    if (I_REF < 0.3)
        I_REF = 0.3;
}

//*******************按键检测函数*******************//
char KEY_Scan(char key_mode)  // 四个按键检测
{
    if ((KEY_H1 == 0) || (KEY_H2 == 0) || (KEY_H3 == 0) || (KEY_H4 == 0)) {
        if (KEY_H1 == 0) {
            return KEY1_PRESS;
        }
        if (KEY_H2 == 0) {
            return KEY2_PRESS;
        }
        if (KEY_H3 == 0) {
            return KEY3_PRESS;
        }
        if (KEY_H4 == 0) {
            return KEY4_PRESS;
        }
    } else
        return KEY_UNPRESS;
    return 0;
}

//*******************按键初始化函数*******************//
void Init_KEY()  // 配置GPIO56 54 39 19为KEY1 2 3 4,上拉，输入
{
    EALLOW;

    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0;  // 配置为GPIO功能 (GPIO56为KEY1)
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;   // 配置为输入方向

    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 0;  // 配置为GPIO功能 (GPIO54为KEY2)
    GpioCtrlRegs.GPBDIR.bit.GPIO54 = 0;   // 配置为输入方向

    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;  // 配置为GPIO功能 (GPIO39为KEY3)
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0;   // 配置为输入方向

    //       GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;   // 配置为GPIO功能 (GPIO19为KEY4)
    //       GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;    // 配置为输入方向

    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;  // 上拉使能
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO39 = 0;
    //       GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;

    EDIS;
}

//*******************ADC初始化函数*******************//
void InitADC() {
    EALLOW;
    PieVectTable.ADCINT1 = &adc_isr;
    EDIS;
    InitAdc();
    AdcOffsetSelfCal();

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER |= M_INT1;
    //    EINT;
    //    ERTM;

    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;
    AdcRegs.INTSEL1N2.bit.INT1SEL = 8;  // 中断源选择,对应SOC结束后触发中断

    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0;    // A0
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC1CTL.bit.CHSEL = 1;    // A1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC2CTL.bit.CHSEL = 10;   // B2
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC3CTL.bit.CHSEL = 9;    // B1
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC4CTL.bit.CHSEL = 2;    // A2
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC5CTL.bit.CHSEL = 3;    // A3
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC6CTL.bit.CHSEL = 8;    // B0
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC7CTL.bit.CHSEL = 11;   // B3
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC7CTL.bit.ACQPS = 10;   // 采样窗大小为6

    AdcRegs.ADCSOC8CTL.bit.CHSEL = 4;    // A4
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL = 9;  // 触发源为ePWM3
    AdcRegs.ADCSOC8CTL.bit.ACQPS = 10;   // 采样窗大小为6

    EDIS;

    // ADC触发源为EPWM3,配置在对应函数
}

//*******************EPWM1中断*******************//
__interrupt void epwm1_isr(void) {
    EPwm1Regs.ETCLR.bit.INT = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//*******************EPWM设置*******************//
void EPWM1_Init() {
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // 失能时基模块时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1;  // 开启相应时钟
    EDIS;

    InitEPwm1Gpio();

    // 设置TB
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // 不使用时钟同步功能
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // 不使用相位同步功能
    EPwm1Regs.TBPHS.half.TBPHS = 0;                 // 相位清零
    EPwm1Regs.TBCTR = 0x0000;                       // 清除计数器的值
    EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;             // 设置周期值
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 向上计数
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // 不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 设置CC
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // 设置比较值
    EPwm1Regs.CMPA.half.CMPA = 0;

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;    // 计数器到 0 时输出高电平
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 递增到 CMPA 时清除
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;    // 递减到 CMPA 时重新置高

    // 若需互补输出，配置 AQCTLB（根据实际需求调整）
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;  // 计数器到 0 时输出低电平
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;    // 递增到 CMPB 时置高
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;  // 递减到 CMPB 时清除

    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;  // 选择计数器到零的时候的事件
    EPwm1Regs.ETSEL.bit.INTEN = 1;             // 使能中断
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;        // 每发生一次触发事件就输出PWM

    // 设置DB死区模块
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;   // epwm作为上升沿和下降沿的触发源
    EPwm1Regs.DBCTL.bit.POLSEL = 2;    // 2
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;  // 设置为高电平延时，根据表格设置为AHC模式
    EPwm1Regs.DBRED = 10;              // 设置上升沿延时，5时大约为0.05us      死区时间= DBRED * 1 / 90MHz
    EPwm1Regs.DBFED = 10;              // 设置下降沿延时，5时大约为0.05us      DBRED=10: 111.1ns

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // 使能时基模块时钟
    EDIS;
}

void EPWM2_Init() {
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // 失能时基模块时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1;  // 开启相应时钟
    EDIS;

    InitEPwm2Gpio();

    // 设置TB
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // 不使用时钟同步功能、
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;       // 使用相位同步功能
    //    EPwm2Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
    EPwm2Regs.TBPHS.half.TBPHS = 0;                 // 相位清零
    EPwm2Regs.TBCTR = 0x0000;                       // 清除计数器的值
    EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;             // 设置周期值
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 向上计数
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // 不分频
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 设置CC
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // 设置比较值
    EPwm2Regs.CMPA.half.CMPA = 0;

    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;    // 计数器到 0 时输出高电平
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 递增到 CMPA 时清除
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;    // 递减到 CMPA 时重新置高

    // 若需互补输出，配置 AQCTLB（根据实际需求调整）
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;  // 计数器到 0 时输出低电平
    EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;    // 递增到 CMPB 时置高
    EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;  // 递减到 CMPB 时清除

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;  // 选择计数器到零的时候的事件
    EPwm2Regs.ETSEL.bit.INTEN = 1;             // 使能中断
    EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;        // 每发生一次触发事件就输出PWM

    // 设置DB死区模块
    EPwm2Regs.DBCTL.bit.IN_MODE = 0;   // epwm作为上升沿和下降沿的触发源
    EPwm2Regs.DBCTL.bit.POLSEL = 2;    // 2
    EPwm2Regs.DBCTL.bit.OUT_MODE = 3;  // 设置为高电平延时，根据表格设置为AHC模式
    EPwm2Regs.DBRED = 10;              // 设置上升沿延时，5时大约为0.05us
    EPwm2Regs.DBFED = 10;              // 设置下降沿延时，5时大约为0.05us

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // 使能时基模块时钟
    EDIS;
}

void EPWM3_Init() {
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;   // 失能时基模块时钟
    SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1;  // 开启相应时钟
    EDIS;

    InitEPwm3Gpio();

    // 设置TB
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // 不使用时钟同步功能
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // 不使用相位同步功能
    EPwm3Regs.TBPHS.half.TBPHS = 0;                 // 相位清零
    EPwm3Regs.TBCTR = 0x0000;                       // 清除计数器的值
    EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;             // 设置周期值
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // 向上计数
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;        // 不分频
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 设置CC
    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    // 设置比较值
    EPwm3Regs.CMPA.half.CMPA = 0;

    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;    // 计数器到 0 时输出高电平
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // 递增到 CMPA 时清除
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;    // 递减到 CMPA 时重新置高

    // 若需互补输出，配置 AQCTLB（根据实际需求调整）
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;  // 计数器到 0 时输出低电平
    EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;    // 递增到 CMPB 时置高
    EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;  // 递减到 CMPB 时清除

    EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;  // 选择计数器到零的时候的事件
    EPwm3Regs.ETSEL.bit.INTEN = 1;             // 使能中断
    EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;        // 每发生一次触发事件就输出PWM

    // 设置DB死区模块
    EPwm3Regs.DBCTL.bit.IN_MODE = 0;   // epwm作为上升沿和下降沿的触发源
    EPwm3Regs.DBCTL.bit.POLSEL = 2;    // 2
    EPwm3Regs.DBCTL.bit.OUT_MODE = 3;  // 设置为高电平延时，根据表格设置为AHC模式
    EPwm3Regs.DBRED = 10;              // 设置上升沿延时，5时大约为0.05us
    EPwm3Regs.DBFED = 10;              // 设置下降沿延时，5时大约为0.05us

    // ADC相关
    EPwm3Regs.ETSEL.bit.SOCAEN = 1;   // 使能EPWM3SOCA脉冲
    EPwm3Regs.ETSEL.bit.SOCASEL = 2;  // Select SOC from CMPA on upcount，CTR=TBPRD事件且定时器增计数时，触发SOC
    EPwm3Regs.ETPS.bit.SOCAPRD = 1;   // Generate pulse on 1st event，当发生一个事件ET模块时，产生EPWM2SOCA脉冲

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;  // 使能时基模块时钟
    EDIS;
}
// void EPWM4_Init()
//{
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    //失能时基模块时钟
//     SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK=1;   //开启相应时钟
//     EDIS;
//
//     InitEPwm4Gpio();
//
//     //设置TB
//     EPwm4Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;       //不使用时钟同步功能
//     EPwm4Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
//     EPwm4Regs.TBPHS.half.TBPHS=0;                   //相位清零
//     EPwm4Regs.TBCTR=0x0000;                         //清除计数器的值
//     EPwm4Regs.TBPRD=EPWM_TIMER_TBPRD;               //设置周期值
//     EPwm4Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;    //向上计数
//     EPwm4Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;          //不分频
//     EPwm4Regs.TBCTL.bit.CLKDIV=TB_DIV1;
//
//     //设置CC
//     EPwm4Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;
//     EPwm4Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;
//     EPwm4Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
//     EPwm4Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;
//     //设置比较值
//     EPwm4Regs.CMPA.half.CMPA=0;
//
//     EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;     // 计数器到 0 时输出高电平
//     EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // 递增到 CMPA 时清除
//     EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;     // 递减到 CMPA 时重新置高
//
//     // 若需互补输出，配置 AQCTLB（根据实际需求调整）
//     EPwm4Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // 计数器到 0 时输出低电平
//     EPwm4Regs.AQCTLB.bit.CBU = AQ_SET;     // 递增到 CMPB 时置高
//     EPwm4Regs.AQCTLB.bit.CBD = AQ_CLEAR;   // 递减到 CMPB 时清除
//
//     EPwm4Regs.ETSEL.bit.INTSEL=ET_CTR_ZERO;         //选择计数器到零的时候的事件
//     EPwm4Regs.ETSEL.bit.INTEN=1;                    //使能中断
//     EPwm4Regs.ETPS.bit.INTPRD=ET_1ST;               //每发生一次触发事件就输出PWM
//
//     //设置DB死区模块
//     EPwm4Regs.DBCTL.bit.IN_MODE=0;                  //epwm作为上升沿和下降沿的触发源
//     EPwm4Regs.DBCTL.bit.POLSEL=2;                   //2
//     EPwm4Regs.DBCTL.bit.OUT_MODE=3;                 //设置为高电平延时，根据表格设置为AHC模式
//     EPwm4Regs.DBRED=10;                              //设置上升沿延时，5时大约为0.05us
//     EPwm4Regs.DBFED=10;                              //设置下降沿延时，5时大约为0.05us
//
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;            //使能时基模块时钟
//     EDIS;
// }

// void EPWM5_Init()
//{
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    //失能时基模块时钟
//     SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK=1;   //开启相应时钟
//     EDIS;
//
//     InitEPwm5Gpio();
//
//     //设置TB
//     EPwm5Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;       //不使用时钟同步功能
//     EPwm5Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
//     EPwm5Regs.TBPHS.half.TBPHS=0;                   //相位清零
//     EPwm5Regs.TBCTR=0x0000;                         //清除计数器的值
//     EPwm5Regs.TBPRD=EPWM_TIMER_TBPRD;               //设置周期值
//     EPwm5Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;    //向上计数
//     EPwm5Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;          //不分频
//     EPwm5Regs.TBCTL.bit.CLKDIV=TB_DIV1;
//
//     //设置CC
//     EPwm5Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;
//     EPwm5Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;
//     EPwm5Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
//     EPwm5Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;
//     //设置比较值
//     EPwm5Regs.CMPA.half.CMPA=0;
//
//     EPwm5Regs.AQCTLA.bit.ZRO = AQ_SET;     // 计数器到 0 时输出高电平
//     EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // 递增到 CMPA 时清除
//     EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;     // 递减到 CMPA 时重新置高
//
//     // 若需互补输出，配置 AQCTLB（根据实际需求调整）
//     EPwm5Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // 计数器到 0 时输出低电平
//     EPwm5Regs.AQCTLB.bit.CBU = AQ_SET;     // 递增到 CMPB 时置高
//     EPwm5Regs.AQCTLB.bit.CBD = AQ_CLEAR;   // 递减到 CMPB 时清除
//
//     EPwm5Regs.ETSEL.bit.INTSEL=ET_CTR_ZERO;         //选择计数器到零的时候的事件
//     EPwm5Regs.ETSEL.bit.INTEN=1;                    //使能中断
//     EPwm5Regs.ETPS.bit.INTPRD=ET_1ST;               //每发生一次触发事件就输出PWM
//
//     //设置DB死区模块
//     EPwm5Regs.DBCTL.bit.IN_MODE=0;                  //epwm作为上升沿和下降沿的触发源
//     EPwm5Regs.DBCTL.bit.POLSEL=2;                   //2
//     EPwm5Regs.DBCTL.bit.OUT_MODE=3;                 //设置为高电平延时，根据表格设置为AHC模式
//     EPwm5Regs.DBRED=10;                              //设置上升沿延时，5时大约为0.05us
//     EPwm5Regs.DBFED=10;                              //设置下降沿延时，5时大约为0.05us
//
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;            //使能时基模块时钟
//     EDIS;
// }
//
// void EPWM6_Init()
//{
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    //失能时基模块时钟
//     SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK=1;   //开启相应时钟
//     EDIS;
//
//     InitEPwm6Gpio();
//
//     //设置TB
//     EPwm6Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;       //不使用时钟同步功能
//     EPwm6Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
//     EPwm6Regs.TBPHS.half.TBPHS=0;                   //相位清零
//     EPwm6Regs.TBCTR=0x0000;                         //清除计数器的值
//     EPwm6Regs.TBPRD=EPWM_TIMER_TBPRD;               //设置周期值
//     EPwm6Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;    //向上计数
//     EPwm6Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;          //不分频
//     EPwm6Regs.TBCTL.bit.CLKDIV=TB_DIV1;
//
//     //设置CC
//     EPwm6Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;
//     EPwm6Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;
//     EPwm6Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
//     EPwm6Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;
//     //设置比较值
//     EPwm6Regs.CMPA.half.CMPA=0;
//
//     EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;     // 计数器到 0 时输出高电平
//     EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // 递增到 CMPA 时清除
//     EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;     // 递减到 CMPA 时重新置高
//
//     // 若需互补输出，配置 AQCTLB（根据实际需求调整）
//     EPwm6Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // 计数器到 0 时输出低电平
//     EPwm6Regs.AQCTLB.bit.CBU = AQ_SET;     // 递增到 CMPB 时置高
//     EPwm6Regs.AQCTLB.bit.CBD = AQ_CLEAR;   // 递减到 CMPB 时清除
//
//     EPwm6Regs.ETSEL.bit.INTSEL=ET_CTR_ZERO;         //选择计数器到零的时候的事件
//     EPwm6Regs.ETSEL.bit.INTEN=1;                    //使能中断
//     EPwm6Regs.ETPS.bit.INTPRD=ET_1ST;               //每发生一次触发事件就输出PWM
//
//     //设置DB死区模块
//     EPwm6Regs.DBCTL.bit.IN_MODE=0;                  //epwm作为上升沿和下降沿的触发源
//     EPwm6Regs.DBCTL.bit.POLSEL=2;                   //2
//     EPwm6Regs.DBCTL.bit.OUT_MODE=3;                 //设置为高电平延时，根据表格设置为AHC模式
//     EPwm6Regs.DBRED=10;                              //设置上升沿延时，5时大约为0.05us
//     EPwm6Regs.DBFED=10;                              //设置下降沿延时，5时大约为0.05us
//
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;            //使能时基模块时钟
//     EDIS;
// }
//
// void EPWM7_Init()
//{
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    //失能时基模块时钟
//     SysCtrlRegs.PCLKCR1.bit.EPWM7ENCLK=1;   //开启相应时钟
//     EDIS;
//
//     InitEPwm7Gpio();
//
//     //设置TB
//     EPwm7Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;       //不使用时钟同步功能
//     EPwm7Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
//     EPwm7Regs.TBPHS.half.TBPHS=0;                   //相位清零
//     EPwm7Regs.TBCTR=0x0000;                         //清除计数器的值
//     EPwm7Regs.TBPRD=EPWM_TIMER_TBPRD;               //设置周期值
//     EPwm7Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;    //向上计数
//     EPwm7Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;          //不分频
//     EPwm7Regs.TBCTL.bit.CLKDIV=TB_DIV1;
//
//     //设置CC
//     EPwm7Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;
//     EPwm7Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;
//     EPwm7Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
//     EPwm7Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;
//     //设置比较值
//     EPwm7Regs.CMPA.half.CMPA=0;
//
//     EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;     // 计数器到 0 时输出高电平
//     EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // 递增到 CMPA 时清除
//     EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;     // 递减到 CMPA 时重新置高
//
//     // 若需互补输出，配置 AQCTLB（根据实际需求调整）
//     EPwm7Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // 计数器到 0 时输出低电平
//     EPwm7Regs.AQCTLB.bit.CBU = AQ_SET;     // 递增到 CMPB 时置高
//     EPwm7Regs.AQCTLB.bit.CBD = AQ_CLEAR;   // 递减到 CMPB 时清除
//
//     EPwm7Regs.ETSEL.bit.INTSEL=ET_CTR_ZERO;         //选择计数器到零的时候的事件
//     EPwm7Regs.ETSEL.bit.INTEN=1;                    //使能中断
//     EPwm7Regs.ETPS.bit.INTPRD=ET_1ST;               //每发生一次触发事件就输出PWM
//
//     //设置DB死区模块
//     EPwm7Regs.DBCTL.bit.IN_MODE=0;                  //epwm作为上升沿和下降沿的触发源
//     EPwm7Regs.DBCTL.bit.POLSEL=2;                   //2
//     EPwm7Regs.DBCTL.bit.OUT_MODE=3;                 //设置为高电平延时，根据表格设置为AHC模式
//     EPwm7Regs.DBRED=10;                              //设置上升沿延时，5时大约为0.05us
//     EPwm7Regs.DBFED=10;                              //设置下降沿延时，5时大约为0.05us
//
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;            //使能时基模块时钟
//     EDIS;
// }
//
// void EPWM8_Init()
//{
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=0;    //失能时基模块时钟
//     SysCtrlRegs.PCLKCR1.bit.EPWM8ENCLK=1;   //开启相应时钟
//     EDIS;
//
//     InitEPwm8Gpio();
//
//     //设置TB
//     EPwm8Regs.TBCTL.bit.SYNCOSEL=TB_CTR_ZERO;       //不使用时钟同步功能
//     EPwm8Regs.TBCTL.bit.PHSEN=TB_DISABLE;           //不使用相位同步功能
//     EPwm8Regs.TBPHS.half.TBPHS=0;                   //相位清零
//     EPwm8Regs.TBCTR=0x0000;                         //清除计数器的值
//     EPwm8Regs.TBPRD=EPWM_TIMER_TBPRD;               //设置周期值
//     EPwm8Regs.TBCTL.bit.CTRMODE=TB_COUNT_UPDOWN;    //向上计数
//     EPwm8Regs.TBCTL.bit.HSPCLKDIV=TB_DIV1;          //不分频
//     EPwm8Regs.TBCTL.bit.CLKDIV=TB_DIV1;
//
//     //设置CC
//     EPwm8Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;
//     EPwm8Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;
//     EPwm8Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;
//     EPwm8Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;
//     //设置比较值
//     EPwm8Regs.CMPA.half.CMPA=0;
//
//     EPwm8Regs.AQCTLA.bit.ZRO = AQ_SET;     // 计数器到 0 时输出高电平
//     EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // 递增到 CMPA 时清除
//     EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;     // 递减到 CMPA 时重新置高
//
//     // 若需互补输出，配置 AQCTLB（根据实际需求调整）
//     EPwm8Regs.AQCTLB.bit.ZRO = AQ_CLEAR;   // 计数器到 0 时输出低电平
//     EPwm8Regs.AQCTLB.bit.CBU = AQ_SET;     // 递增到 CMPB 时置高
//     EPwm8Regs.AQCTLB.bit.CBD = AQ_CLEAR;   // 递减到 CMPB 时清除
//
//     EPwm8Regs.ETSEL.bit.INTSEL=ET_CTR_ZERO;         //选择计数器到零的时候的事件
//     EPwm8Regs.ETSEL.bit.INTEN=1;                    //使能中断
//     EPwm8Regs.ETPS.bit.INTPRD=ET_1ST;               //每发生一次触发事件就输出PWM
//
//     //设置DB死区模块
//     EPwm8Regs.DBCTL.bit.IN_MODE=0;                  //epwm作为上升沿和下降沿的触发源
//     EPwm8Regs.DBCTL.bit.POLSEL=2;                   //2
//     EPwm8Regs.DBCTL.bit.OUT_MODE=3;                 //设置为高电平延时，根据表格设置为AHC模式
//     EPwm8Regs.DBRED=10;                              //设置上升沿延时，5时大约为0.05us
//     EPwm8Regs.DBFED=10;                              //设置下降沿延时，5时大约为0.05us
//
//     EALLOW;
//     SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC=1;            //使能时基模块时钟
//     EDIS;
// }
