#include <Arduino.h>
#include <AccelStepper.h>
#include "ad5940.h"
#include "RampTest.h"
#include "Impedance.h"
#include <Adafruit_ADS1X15.h>

#define HAVE_VOLTAGE_CONTROL 

#ifdef HAVE_VOLTAGE_CONTROL
#define VOLTAGE_NONE    0
#define VOLTAGE_1       1
#define VOLTAGE_2       2

#define CMD_EXTRA_NONE  0
#define CMD_EXTRA_SUB   1
#define CMD_SUB         2
#define CMD_ADD         3
#define CMD_EXTRA_ADD   4

#define VALID_VOLTAGE_ERROR 1 /*V*/
#define MAX_ATTEMP 5
#endif

unsigned long timeStart = 0;
unsigned long timeNow = 0;

#define AppCVBuff_CV_SIZE 1024
uint32_t AppCVBuff[AppCVBuff_CV_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

#define APPBUFF_EIS_SIZE 512
uint32_t AppEISBuff[APPBUFF_EIS_SIZE];

float S_Vol, E_Vol;
int countRepeat, StepNumber = 0, RepeatTimes = 0;
BoolFlag logEn = bFALSE;
String inputString = "";
byte moc1, moc2, moc3, moc4, moc5;


#ifdef HAVE_VOLTAGE_CONTROL 
#define HALFSTEP 8
/* for voltage control */
float targetVoltage1; //gia tri Voltage tra ve cho stepper1
float curentVoltage1 = 0; //gia tri Voltage nhap vao cho stepper1
float targetVoltage2; //gia tri Voltage tra ve cho stepper2
float curentVoltage2 = 0;; //gia tri Voltage nhap vao cho stepper2

// ULN2003 Motor Driver Pins
#define motorPin1  13     // IN1 on the ULN2003 driver 1
#define motorPin2  12     // IN2 on the ULN2003 driver 1
#define motorPin3  14     // IN3 on the ULN2003 driver 1
#define motorPin4  27     // IN4 on the ULN2003 driver 1

#define motorPin5  26    // IN1 on the ULN2003 driver 2
#define motorPin6  25    // IN2 on the ULN2003 driver 2
#define motorPin7  33   // IN3 on the ULN2003 driver 2
#define motorPin8  32   // IN4 on the ULN2003 driver 2


// variables
const float stepsPerRevolution = 4096;  // change this to fit the number of steps per revolution
float StepsPerVol = 37.4064  ; // so buoc tren 1 Vol
int stepperSpeed = 100; //speed of the stepper (steps per second)

// initialize the ads library
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
// initialize the stepper library
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
AccelStepper stepper2(HALFSTEP, motorPin5, motorPin7, motorPin6, motorPin8);

hw_timer_t* timer = NULL; //khơi tạo timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; 

#endif

static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t index;
  /* Print data*/
  for(int i = 0; i < DataCount; i++)
  {
    printf("%d;%.6f\n",index++, pData[i]);
  }
  return 0;
}

int32_t ImpedanceShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpPol_Type *pImp = (fImpPol_Type*)pData;
  AppIMPCtrl(IMPCTRL_GETFREQ, &freq);

  if (S_Vol == E_Vol)
  {
    timeNow = millis() - timeStart;
    printf("%lu;", timeNow);
  }
  else
  {
    printf("%.2f;", freq);
  }
  /*Process data*/
  float phase;
  for(int i=0;i<DataCount;i++)
  {
    phase = pImp[i].Phase*180/MATH_PI;
    if(phase > 180) phase = phase - 360;
    else if (phase < -180) phase = phase + 360;
    //printf("%f;%f\n", pImp[i].Magnitude, pImp[i].Phase*180/MATH_PI);
    printf("%f;%f\n", pImp[i].Magnitude, phase);
  }
  return 0;
}

static int32_t AD5940CVPlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();    /* Call this right after AFE reset */
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  /* Configure FIFO and Sequencer */
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Configure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;  /* GPIO1 indicates AFE is in sleep state. GPIO2 indicates ADC is sampling. */
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurement accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

static int32_t AD5940EISPlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* Use hardware reset */
  AD5940_HWReset();
  AD5940_Initialize();
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB for FIFO, The reset 2kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4;//AppIMPCfg.FifoThresh;        /* DFT result. One pair for RCAL, another for Rz. One DFT result have real part and imaginary part */
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  /* Step4: Reconfigure GPIO */
  gpio_cfg.FuncSet = GP0_INT|GP1_SLEEP|GP2_SYNC;
  gpio_cfg.InputEnSet = 0;
  gpio_cfg.OutputEnSet = AGPIO_Pin0|AGPIO_Pin1|AGPIO_Pin2;
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;
  AD5940_AGPIOCfg(&gpio_cfg);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* Allow AFE to enter sleep mode. */
  return 0;
}

void AD5940RampStructInit(float S_Vol, float E_Vol, int StepNumber)
{
  AppRAMPCfg_Type *pRampCfg;

  AppRAMPGetCfg(&pRampCfg);
  /* Step1: configure general parmaters */
  pRampCfg->SeqStartAddr = 0x10;                /* leave 16 commands for LFOSC calibration.  */
  pRampCfg->MaxSeqLen = 1024-0x10;              /* 4kB/4 = 1024  */
  pRampCfg->RcalVal = 200.0;                  /* 10kOhm RCAL */
  pRampCfg->ADCRefVolt = 1820.0f;               /* The real ADC reference voltage. Measure it from capacitor C12 with DMM. */
  pRampCfg->FifoThresh = 10;                   /* Maximum value is 2kB/4-1 = 512-1. Set it to higher value to save power. */
  pRampCfg->SysClkFreq = 16000000.0f;           /* System clock is 16MHz by default */
  pRampCfg->LFOSCClkFreq = LFOSCFreq;           /* LFOSC frequency */
  /* Configure ramp signal parameters */
  pRampCfg->RampStartVolt =  S_Vol;           /* -1V */
  pRampCfg->RampPeakVolt = E_Vol;           /* +1V */
  pRampCfg->VzeroStart = 1300.0f;               /* 1.3V */
  pRampCfg->VzeroPeak = 1300.0f;                /* 1.3V */
  pRampCfg->StepNumber = StepNumber;                   /* Total steps. Equals to ADC sample number. Limited to 4095 */
  pRampCfg->RampDuration = 24*1000;            /* X * 1000, where x is total duration of ramp signal. Unit is ms. */
  pRampCfg->SampleDelay = 7.0f;                 /* 7ms. Time between update DAC and ADC sample. Unit is ms. */
  pRampCfg->LPTIARtiaSel = LPTIARTIA_4K;       /* Maximum current decides RTIA value */
  pRampCfg->LPTIARloadSel = LPTIARLOAD_SHORT;
  pRampCfg->AdcPgaGain = ADCPGA_1P5;
}

void AD5940ImpedanceStructInit(float S_Freq, float E_Freq, int numPoints, BoolFlag logEn)
{
  AppIMPCfg_Type *pImpedanceCfg;
  
  AppIMPGetCfg(&pImpedanceCfg);
  /* Step1: configure initialization sequence Info */
  pImpedanceCfg->SeqStartAddr = 0;
  pImpedanceCfg->MaxSeqLen = 512; /* @todo add checker in function */

  pImpedanceCfg->RcalVal = 200.0;
  pImpedanceCfg->SinFreq = 60000.0;
  pImpedanceCfg->FifoThresh = 4;
  
  /* Set switch matrix to onboard(EVAL-AD5940ELECZ) dummy sensor. */
  /* Note the RCAL0 resistor is 10kOhm. */
  pImpedanceCfg->DswitchSel = SWD_CE0;
  pImpedanceCfg->PswitchSel = SWP_RE0;
  pImpedanceCfg->NswitchSel = SWN_AIN0;
  pImpedanceCfg->TswitchSel = SWT_AIN0|SWT_TRTIA;
  /* The dummy sensor is as low as 5kOhm. We need to make sure RTIA is small enough that HSTIA won't be saturated. */
  pImpedanceCfg->HstiaRtiaSel = HSTIARTIA_200;  
  
  /* Configure the sweep function. */
  pImpedanceCfg->SweepCfg.SweepEn = bTRUE;
  pImpedanceCfg->SweepCfg.SweepStart = S_Freq;  /* Start from 1kHz */
  pImpedanceCfg->SweepCfg.SweepStop = E_Freq;   /* Stop at 100kHz */
  pImpedanceCfg->SweepCfg.SweepPoints = numPoints;    /* Points is 101 */
  pImpedanceCfg->SweepCfg.SweepLog = logEn;
  /* Configure Power Mode. Use HP mode if frequency is higher than 80kHz. */
  pImpedanceCfg->PwrMod = AFEPWR_HP;
  /* Configure filters if necessary */
  pImpedanceCfg->ADCSinc3Osr = ADCSINC3OSR_2;   /* Sample rate is 800kSPS/2 = 400kSPS */
  pImpedanceCfg->DftNum = DFTNUM_16384;
  pImpedanceCfg->DftSrc = DFTSRC_SINC3;
}

void AD5940_CV_Main()
{
  uint32_t temp;
  uint32_t count = 0;
  AppRAMPCfg_Type *pRampCfg;
  
  AD5940CVPlatformCfg();
  AD5940RampStructInit(S_Vol, E_Vol, StepNumber);

  AppRAMPInit(AppCVBuff, AppCVBuff_CV_SIZE);    /* Initialize RAMP application. Provide a buffer, which is used to store sequencer commands */
  AppRAMPCtrl(APPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  while (count < (StepNumber * RepeatTimes))
  {
    AppRAMPGetCfg(&pRampCfg);
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = AppCVBuff_CV_SIZE;
      AppRAMPISR(AppCVBuff, &temp);
      RampShowResult((float*)AppCVBuff, temp);
      
      count += temp;
      printf("%d", count);  //sonnh fix UI can not get the result
    }
    /* Repeat Measurement continuously*/
    if(pRampCfg->bTestFinished ==bTRUE)
    {
      AD5940_Delay10us(20000);
      pRampCfg->bTestFinished = bFALSE;
      AD5940_SEQCtrlS(bTRUE);   /* Enable sequencer, and wait for trigger */
      AppRAMPCtrl(APPCTRL_START, 0);
    }
  }
}

void AD5940_EIS_Main(void)
{
  uint32_t temp;  
  AD5940EISPlatformCfg();
  AD5940ImpedanceStructInit(S_Vol, E_Vol, StepNumber, logEn);
  
  AppIMPInit(AppEISBuff, APPBUFF_EIS_SIZE);    /* Initialize IMP application. Provide a buffer, which is used to store sequencer commands */
  AppIMPCtrl(IMPCTRL_START, 0);          /* Control IMP measurement to start. Second parameter has no meaning with this command. */

  if (S_Vol == E_Vol)
  {
    timeStart = millis();
    timeNow = 0;
    bool measure = true;
    while(measure)
    {
      if(AD5940_GetMCUIntFlag())
      {
        AD5940_ClrMCUIntFlag();
        temp = APPBUFF_EIS_SIZE;
        AppIMPISR(AppEISBuff, &temp);
        ImpedanceShowResult(AppEISBuff, temp);
      }
      if (Serial.available())
      {
        char inChar = (char)Serial.read();
        if (inChar == 's')
        {
          measure = false;
        }
      }
    }
  }
  else
  {
    uint32_t count = 0;
    while(count < (StepNumber * RepeatTimes))
    {
      if(AD5940_GetMCUIntFlag())
      {
        AD5940_ClrMCUIntFlag();
        temp = APPBUFF_EIS_SIZE;
        AppIMPISR(AppEISBuff, &temp);
        ImpedanceShowResult(AppEISBuff, temp);
        count += temp;
      }
    }
  }
}

#ifdef HAVE_VOLTAGE_CONTROL 
void readVoltage(){

  int16_t adc0, adc1;
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  curentVoltage1 = ads.computeVolts(adc0) * 18;  // Nhân với 1 hàm tuyến tính (chưa có công thúc), Tại đây không dùng map vì map trả về kiểu Int (Tự làm tròn)
  curentVoltage2 = ads.computeVolts(adc1) * 18;

#ifdef DEBUG
  Serial.print("AIN0: "); 
  Serial.print(adc0);
  Serial.print("\tVoltage1: ");
  Serial.println(curentVoltage1, 7); 
  Serial.println();

  Serial.print("AIN1: "); 
  Serial.print(adc1);
  Serial.print("\tVoltage2: ");
  Serial.println(curentVoltage2, 7); 
  Serial.println();
#endif
}

long volToSteps(float vol1,float vol2){
  long a;
  a = (vol2 - vol1)*StepsPerVol;
  return a;
}

void VoltageCtrl_Main() {
  int attempCount = 0;
  long currPos = 0;
  float voltage1Error;
  float voltage2Error;
  int isSuccess = 0;

  const float targetVoltage1_l = targetVoltage1;
  const float targetVoltage2_l = targetVoltage2;
  readVoltage();
  voltage1Error = fabs(targetVoltage1_l - curentVoltage1);
  voltage2Error = fabs(targetVoltage2_l - curentVoltage2);
  
  do {
    if (voltage1Error > VALID_VOLTAGE_ERROR) {
     long relative_step = volToSteps(curentVoltage1, targetVoltage1_l);
      stepper1.move(relative_step);
#ifdef DEBUG
      currPos = stepper1.currentPosition();
      Serial.print("Stepper1:\trelative step: ");
      Serial.print(relative_step);
      Serial.print("\tpos before: ");
      Serial.print(currPos);
#endif
      stepper1.runToPosition();
#ifdef DEBUG
      currPos = stepper1.currentPosition();
      Serial.print("\tpos after: ");
      Serial.println(currPos);
#endif
    }

    if (voltage2Error > VALID_VOLTAGE_ERROR) {
      long relative_step = volToSteps(curentVoltage2, targetVoltage2_l);
      stepper2.move(relative_step);
#ifdef DEBUG
      currPos = stepper2.currentPosition();
      Serial.print("Stepper2:\trelative step: ");
      Serial.print(relative_step);
      Serial.print("\tpos before: ");
      Serial.print(currPos);
#endif
      stepper2.runToPosition();
#ifdef DEBUG
      currPos = stepper2.currentPosition();
      Serial.print("\tpos after: ");
      Serial.println(currPos);
#endif
    }
    attempCount++;
    delay(200);

    /*read voltage values after rotate steppers */
    readVoltage();
    voltage1Error = fabs(targetVoltage1_l - curentVoltage1);
    voltage2Error = fabs(targetVoltage2_l - curentVoltage2);

    if (voltage1Error < VALID_VOLTAGE_ERROR && voltage2Error < VALID_VOLTAGE_ERROR) 
      isSuccess = 1;
    /*send result to UI*/
    Serial.println("0;" + String(attempCount) + ";" + String(MAX_ATTEMP) + ";"  + String(isSuccess) + ";" + String(curentVoltage1) + ";" + String(curentVoltage2));
  }
  while((voltage1Error > VALID_VOLTAGE_ERROR || voltage2Error > VALID_VOLTAGE_ERROR) && attempCount < MAX_ATTEMP);
}
#endif

// hàm xử lý ngắt
void IRAM_ATTR onTimer() {   
  portENTER_CRITICAL_ISR(&timerMux); //vào chế độ tránh xung đột
  //sendVoltage();
  portEXIT_CRITICAL_ISR(&timerMux); // thoát 
}

/*******************************************************************************
 * Write code arduino in here
 ******************************************************************************/
void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  if (!ads.begin(0x48))
  {
    Serial.println("!ADS_Failed");    
    while (1);
  }

  uint32_t checkInitMCU = AD5940_MCUResourceInit(0);
  if(checkInitMCU == 0)
  {
  }

#ifdef HAVE_VOLTAGE_CONTROL
  // set the speed at 15 rpm
  stepper1.setMaxSpeed(1000.0); // toc do max cua dong co
  stepper1.setAcceleration(100.0);// gia toc
  stepper1.setSpeed(stepperSpeed);// toc do hien tai
  stepper1.setCurrentPosition(0);
  
  stepper2.setMaxSpeed(1000.0); // toc do max cua dong co
  stepper2.setAcceleration(100.0);// gia toc
  stepper2.setSpeed(stepperSpeed);// toc do hien tai
  stepper2.setCurrentPosition(0);
#if 0 /*not use timer anymore*/
  //khoi tạo timer với chu kì 1us vì thạch anh của ESP chạy 8MHz
  timer = timerBegin(0, 80, true);
  //khởi tạo hàm xử lý ngắt ngắt cho Timer
  timerAttachInterrupt(timer, &onTimer, true);
  //khởi tạo thời gian ngắt cho timer là 1s (1000000 us)
  timerAlarmWrite(timer, 1000000, true);
  //bắt đầu chạy timer
  timerAlarmEnable(timer);
#endif
#endif

}

void loop() {
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar != '!')
    {
      inputString += inChar;
    }
    else
    {
      for(int i = 0; i < inputString.length(); i++)
      {
        if(inputString[i] == '#') {moc1 = i;}
        if(inputString[i] == '?') {moc2 = i;}
        if(inputString[i] == '/') {moc3 = i;}
        if(inputString[i] == '|') {moc4 = i;}
        if(inputString[i] == '$') {moc5 = i;}
      }
      if(inputString[0] == '1' || inputString[0] == '2'){
        S_Vol = inputString.substring((moc1 + 1), moc2).toDouble() * 1.0;
        E_Vol = inputString.substring((moc2 + 1), moc3).toInt() * 1.0;
        StepNumber = inputString.substring((moc3 + 1), moc4).toInt();
        RepeatTimes = inputString.substring((moc4 + 1), moc5).toInt();
        logEn = (inputString.substring(moc5 + 1).toInt() == 1) ? bTRUE : bFALSE;
      }
#ifdef HAVE_VOLTAGE_CONTROL 
      else if(inputString[0] == '3') {
        targetVoltage1  = inputString.substring((moc1 + 1), moc2).toDouble() * 1.0;
        targetVoltage2 = inputString.substring((moc2 + 1), moc3).toDouble() * 1.0;
      }
#endif
      
      if (inputString[0] == '1')
      {
        AD5940_CV_Main();
        ESP.restart();
      }
      else if (inputString[0] = '2')
      {
        AD5940_EIS_Main();
        ESP.restart();
      }
#ifdef HAVE_VOLTAGE_CONTROL
      else if (inputString[0] = '3')
      {
        VoltageCtrl_Main();
      }
#endif
      inputString = "";
    }
  }
}
