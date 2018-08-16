/* Includes ------------------------------------------------------------------*/

/* mbed specific header files. */
#include "mbed.h"
#include "math.h"
/* Helper header files. */
#include "DevSPI.h"

/* Component specific header files. */
#include "powerstep01_class.h"
//#include "rtos.h"
#include "SPISlave.h"

#define SPI_SPEED   (10000000)

SPISlave SpiS(PB_15, PB_14, PB_13, PB_12); /*Comunnication between ESP32 and Nucleo*/

/* Global variables-----------------------------------------------------------------*/
long value = 0;
int16_t f[3]= {0};
uint32_t Tipo[7]= {0,0,0,0,0,0,0};
unsigned int step = 0;
int x, y, k=1,integral=0;
bool direction_c=0;
float spd_cur = 0.00004,kp = 1.736, ki = 2.257 ;
Serial pc(SERIAL_TX, SERIAL_RX);
InterruptIn mybutton(USER_BUTTON);
DigitalOut myled(PB_3);
DigitalOut my_b(PB_1,0);
DigitalOut freio(PB_7,0);
InterruptIn finalc(PB_0);
int begin,end,d_i,d_f,max=10000,min=-10000;


/* Initialization parameters of the motor connected to the expansion board. */
/* Current mode. */
powerstep01_Init_u_t initDeviceParameters = {
    /* common parameters */
    .cm.cp.cmVmSelection = POWERSTEP01_CM_VM_CURRENT, // enum powerstep01_CmVm_t
    30000, // Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2
    30000, // Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
    10000, // Maximum speed in step/s, range 15.25 to 15610 steps/s
    0, // Minimum speed in step/s, range 0 to 976.3 steps/s
    POWERSTEP01_LSPD_OPT_OFF, // Low speed optimization bit, enum powerstep01_LspdOpt_t
    10000.00, // Full step speed in step/s, range 7.63 to 15625 steps/s
    POWERSTEP01_BOOST_MODE_OFF, // Boost of the amplitude square wave, enum powerstep01_BoostMode_t
    281.25, // Overcurrent threshold settings via enum powerstep01_OcdTh_t
    STEP_MODE_1_4, // Step mode settings via enum motorStepMode_t Modificado********************
    POWERSTEP01_SYNC_SEL_DISABLED, // Synch. Mode settings via enum powerstep01_SyncSel_t
    (POWERSTEP01_ALARM_EN_OVERCURRENT|
    POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN|
    POWERSTEP01_ALARM_EN_THERMAL_WARNING|
    POWERSTEP01_ALARM_EN_UVLO|
    POWERSTEP01_ALARM_EN_STALL_DETECTION|
    POWERSTEP01_ALARM_EN_SW_TURN_ON|
    POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD), // Alarm settings via bitmap enum powerstep01_AlarmEn_t
    POWERSTEP01_IGATE_64mA, // Gate sink/source current via enum powerstep01_Igate_t
    POWERSTEP01_TBOOST_0ns, // Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t
    POWERSTEP01_TCC_500ns, // Controlled current time via enum powerstep01_Tcc_t
    POWERSTEP01_WD_EN_DISABLE, // External clock watchdog, enum powerstep01_WdEn_t
    POWERSTEP01_TBLANK_375ns, // Duration of the blanking time via enum powerstep01_TBlank_t
    POWERSTEP01_TDT_125ns, // Duration of the dead time via enum powerstep01_Tdt_t
    /* current mode parameters */
    300.0, // Hold torque in mV, range from 7.8mV to 1000 mV
    300.0, // Running torque in mV, range from 7.8mV to 1000 mV
    300.0, // Acceleration torque in mV, range from 7.8mV to 1000 mV
    300.0, // Deceleration torque in mV, range from 7.8mV to 1000 mV
    POWERSTEP01_TOFF_FAST_8us, //Maximum fast decay time , enum powerstep01_ToffFast_t
    POWERSTEP01_FAST_STEP_12us, //Maximum fall step time , enum powerstep01_FastStep_t
    3.0, // Minimum on-time in us, range 0.5us to 64us
    21.0, // Minimum off-time in us, range 0.5us to 64us
    POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ, // Clock setting , enum powerstep01_ConfigOscMgmt_t
    POWERSTEP01_CONFIG_SW_HARD_STOP, // External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t
    POWERSTEP01_CONFIG_TQ_REG_TVAL_USED, // External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t
    POWERSTEP01_CONFIG_VS_COMP_DISABLE, // Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
    POWERSTEP01_CONFIG_OC_SD_DISABLE, // Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t
    POWERSTEP01_CONFIG_UVLOVAL_LOW, // UVLO Threshold via powerstep01_ConfigUvLoVal_t
    POWERSTEP01_CONFIG_VCCVAL_15V, // VCC Val, enum powerstep01_ConfigVccVal_t
    POWERSTEP01_CONFIG_TSW_048us, // Switching period, enum powerstep01_ConfigTsw_t
    POWERSTEP01_CONFIG_PRED_DISABLE // Predictive current enabling , enum powerstep01_ConfigPredEn_t
};

/* Motor Control Component. */
POWERSTEP01 *motor;


/* Functions -----------------------------------------------------------------*/

/**
 * @brief  This is an example of user handler for the flag interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->AttachFlagIRQ(&myFlagIRQHandler);
 *           + motor->EnableFlagIRQ();
 *         To disable it:
 *           + motor->DisbleFlagIRQ();
 */
void myFlagIRQHandler(void)
{
    /* Set ISR flag. */
    motor->isrFlag = TRUE;

    /* Get the value of the status register. */
    unsigned int statusRegister = motor->GetStatus();

    //printf("    WARNING: \"FLAG\" interrupt triggered.\r\n");
    /* Check HIZ flag: if set, power brigdes are disabled */
    if ((statusRegister & POWERSTEP01_STATUS_HIZ)==POWERSTEP01_STATUS_HIZ) {
        // HIZ state
        //printf("    HiZ state.\r\n");
    }
    /* Check BUSY flag: if not set, a command is under execution */
    if ((statusRegister & POWERSTEP01_STATUS_BUSY)==0) {
        // BUSY
        //printf("    Busy.\r\n");
    }
    /* Check SW_F flag: if not set, the SW input is opened */
    if ((statusRegister & POWERSTEP01_STATUS_SW_F )!=0) {
        // SW closed (connected to ground)
        //printf("    SW closed (connected to ground).\r\n");
    }
    /* Check SW_EN bit */
    if ((statusRegister & POWERSTEP01_STATUS_SW_EVN)==
            POWERSTEP01_STATUS_SW_EVN) {
        // SW turn_on event
        //printf("    SW turn_on event.\r\n");
    }
    if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS)==
            POWERSTEP01_STATUS_MOT_STATUS_STOPPED) {
        // MOTOR STOPPED
        //printf("    Stopped.\r\n");
    } else {
        if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS)==
                POWERSTEP01_STATUS_MOT_STATUS_ACCELERATION) {
            // MOTOR ACCELERATION
            //printf("    Accelerating ");
        } else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS)==
                    POWERSTEP01_STATUS_MOT_STATUS_DECELERATION) {
            // MOTOR DECELERATION
            //printf("    Decelerating ");
        } else  if ((statusRegister & POWERSTEP01_STATUS_MOT_STATUS)==
                    POWERSTEP01_STATUS_MOT_STATUS_CONST_SPD) {
            // MOTOR RUNNING AT CONSTANT SPEED
            pc.printf("    Steady running ");
        }
        /* Check direction bit */
        if ((statusRegister & POWERSTEP01_STATUS_DIR)==0) {
            // StepperMotor::BWD
            //printf(" in backward direction.\r\n");
            direction_c = 1;
        } else {
            // StepperMotor::FWD
            //printf(" in forward direction.\r\n");
            direction_c = 0;
        }
    }
    /* Check Command Error flag: if set, the command received by SPI can't be */
    /* performed. This occurs for instance when a move command is sent to the */
    /* Powerstep01 while it is already running */
    if ((statusRegister & POWERSTEP01_STATUS_CMD_ERROR)==
            POWERSTEP01_STATUS_CMD_ERROR) {
        // Command Error
        //printf("    Non-performable command detected.\r\n");
    }
    /* Check Step mode clock flag: if set, the device is working in step clock mode */
    if ((statusRegister & POWERSTEP01_STATUS_STCK_MOD)==
            POWERSTEP01_STATUS_STCK_MOD) {
        //Step clock mode enabled
        //printf("    Step clock mode enabled.\r\n");
    }
    /* Check UVLO flag: if not set, there is an undervoltage lock-out */
    if ((statusRegister & POWERSTEP01_STATUS_UVLO)==0) {
        //Undervoltage lock-out
        //printf("    undervoltage lock-out.\r\n");
    }
    /* Check UVLO ADC flag: if not set, there is an ADC undervoltage lock-out */
    if ((statusRegister & POWERSTEP01_STATUS_UVLO_ADC)==0) {
        //ADC undervoltage lock-out
        //printf("    ADC undervoltage lock-out:\r\n");
        //printf("    Expected with default IHM03A1 HW configuration.\r\n");
    }
    /* Check thermal STATUS flags: if  set, the thermal status is not normal */
    if ((statusRegister & POWERSTEP01_STATUS_TH_STATUS)!=0) {
        //thermal status: 1: Warning, 2: Bridge shutdown, 3: Device shutdown
        if (((statusRegister & POWERSTEP01_STATUS_TH_STATUS)>>11)==1) {
            //printf("    Thermal status - Warning.\r\n");
        } else if (((statusRegister & POWERSTEP01_STATUS_TH_STATUS)>>11)==2) {
            //printf("    Thermal status - Bridge shutdown.\r\n");
        } else if (((statusRegister & POWERSTEP01_STATUS_TH_STATUS)>>11)==3) {
            //printf("    Thermal status - Device shutdown.\r\n");
        }
    }
    /* Check OCD  flag: if not set, there is an overcurrent detection */
    if ((statusRegister & POWERSTEP01_STATUS_OCD)==0) {
        //Overcurrent detection
        //printf("    Overcurrent detection.\r\n");
    }
    /* Check STALL_A flag: if not set, there is a Stall condition on bridge A */
    if ((statusRegister & POWERSTEP01_STATUS_STALL_A)==0) {
        //Bridge A stalled
        //printf("    Bridge A stalled.\r\n");
    }
    /* Check STALL_B flag: if not set, there is a Stall condition on bridge B */
    if ((statusRegister & POWERSTEP01_STATUS_STALL_B)==0) {
        //Bridge B stalled
        //printf("    Bridge B stalled.\r\n");
    }

    /* Reset ISR flag. */
    motor->isrFlag = FALSE;
}

/**
 * @brief  This is an example of user handler for the busy interrupt.
 * @param  None
 * @retval None
 * @note   If needed, implement it, and then attach and enable it:
 *           + motor->AttachBusyIRQ(&myBusyIRQHandler);
 *           + motor->EnableBusyIRQ();
 *         To disable it:
 *           + motor->DisbleBusyIRQ();
 */
void myBusyIRQHandler(void)
{
    /* Set ISR flag. */
    motor->isrFlag = TRUE;

    if (motor->CheckBusyHw()) {
        /* Busy pin is low, so at list one Powerstep01 chip is busy */
        /* To be customized (for example Switch on a LED) */
    } else {
        /* To be customized (for example Switch off a LED) */
    }

    /* Reset ISR flag. */
    motor->isrFlag = FALSE;
}

/**
 * @brief  This is an example of error handler.
 * @param[in] error Number of the error
 * @retval None
 * @note   If needed, implement it, and then attach it:
 *           + motor->AttachErrorHandler(&myErrorHandler);
 */
void myErrorHandler(uint16_t error)
{
    /* Printing to the console. */
    //printf("Error %d detected\r\n\n", error);

    /* Infinite loop */
    while(1) {
    }
}

/**
 * @brief  This function receive and transform the force value in loadcell.
 * @param[in] void
 * @retval Force in Newtons
 */
double recibe(void)
{
    my_b = 1;
    for (int co = 0; co <= 2; co++ ) {
        f[co] = SpiS.read(); /* Read the SPI from ESP32- three words of 16 bits each*/
    }
    my_b = 0;
    if (f[0] != 0) {/*If the user stop the task this condition reset the motor and send it to home*/
        Tipo[0]=0;
        f[1]=0.00;
        f[2]=0.00;
        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, 150);
        motor->Move(StepperMotor::FWD, 1600);
        motor->WaitWhileActive();
        motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 600);
        motor->WaitWhileActive();
        //motor->HardStop();
        //motor->SetHome();
    }
    value =( static_cast<unsigned long>(f[1]) << 16
             | static_cast<unsigned long>(f[2]) );/*Add both 16-bits words on one alone data*/
    return ((value/11746)*9.81);
}
/**
 * @brief  This function is used to move the motor in passive mode.
 * @param[in] position desired
 * @retval void
 */
void mover_motor(int pos_d)/*Used to move the motor in passive mode*/
{
    motor->GoTo(pos_d);
    motor->WaitWhileActive();
    /*This section is used to stop the motor when is detected a disturbance*/
    /*do {

        //pc.printf("$%d;", recibe());
        //wait_ms(10);
        if (((motor->ReadStatusRegister() & POWERSTEP01_STATUS_DIR)!=0) && recibe()<20) {
            motor->HardHiZ();
            break;

        } else if (((motor->ReadStatusRegister() & POWERSTEP01_STATUS_DIR)==0) && recibe()>20) {
            motor->HardHiZ();
            break;
        }
    } while((motor->ReadStatusRegister() & POWERSTEP01_STATUS_BUSY)==1);*/
}

/**
 * @brief  This function to calculate the steps to move the motor and to work in steps mode [Move] based
 * on the equation F=kx.
 * @param[in] Force
 * @retval Number of steps to perform the task
 */

int calcula_n_steps(double F)
{
    if (F< 0.0) {
        F = 0.0;
    }
    switch (Tipo[2]) {
        case 1:/* Yellow band*/
            k=44;//268;
            break;
        case 2:/* Red band*/
            k=53;//340;
            break;
        case 3:/* Green band*/
            k=73;//448;
            break;
        case 4:/* Azul band*/
            k=100;//624;
            break;
        case 5:/* Manual band*/
            k=Tipo[6];
            break;
        case 6:
            k=133;
            break;
        case 7:
            k=191;
            break;
    }
    x = F/k;//convertir a pasos
    y = 50000.00*x; // pulsos solicitados- Setpoint
    pc.printf("pulsos= %i \r\n",y);
    return(y);
}

/* Encoder--------------------------------------------------------------------*/

/**
 * @brief  This function use the encoder to measure position.
 * @param[in] none
 * @retval none
 */
void EncoderInitialiseP(void)
{
    // configure GPIO PA0 & PA1 as inputs for Encoder
    RCC->AHB1ENR |= 0x00000011;  // Enable clock for GPIOA

    GPIOA->MODER   |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;           //PA0 & PB3 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_0 | GPIO_OTYPER_OT_1 ;                 //PA0 & PB3 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR0 | GPIO_OSPEEDER_OSPEEDR1 ;     // Low speed                        /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR1_1 ;           // Pull Down                        /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x00000011 ;                                          //  AF01 for PA0 & PA1              /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //                                  /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */

    // configure TIM2 as Encoder input
    RCC->APB1ENR |= 0x00000001;  // Enable clock for TIM2

    TIM2->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM2->SMCR  = 0x0003;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM2->CCMR1 = 0xF1F1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1
    TIM2->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM2->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM2->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM2->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register

    TIM2->CNT = 0x0000;  //reset the counter before we use it
}
/**
 * @brief  This function use the encoder to measure speed.
 * @param[in] none
 * @retval none
 */
void EncoderInitialiseV(void)
{
    // PA0 -> Counter frequency input pin as Timer2 TI1
    GPIOA->AFR[0] &= 0xfffffff0;
    GPIOA->AFR[0] |= GPIO_AF1_TIM2;
    GPIOA->MODER &= ~(GPIO_MODER_MODER0);
    GPIOA->MODER |= 0x2;
    // Initialize Timer2(32bit) for an external up counter mode
    RCC->APB1ENR |= ((uint32_t)0x00000001);
    TIM2->CR1 &= (uint16_t)(~(TIM_CR1_DIR | TIM_CR1_CMS | TIM_CR1_CKD));  // count_up + div by 1
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->PSC = 0x0000;
    TIM2->CCMR1 &= (uint16_t)~TIM_CCMR1_IC1F;   // input filter
    TIM2->CCER = TIM_CCER_CC1P;     // positive edge
    TIM2->SMCR &= (uint16_t)~(TIM_SMCR_SMS | TIM_SMCR_TS | TIM_SMCR_ECE);
    TIM2->SMCR |= (uint16_t)(TIM_TS_TI1FP1 | TIM_SMCR_SMS); // external mode 1
    TIM2->CR1 |= TIM_CR1_CEN;   //Enable the TIM Counter
}

/**
 * @brief  This function use a timer to stop the motor when end-final sensor is triggered.
 * @param[in] none
 * @retval none
 */
void para_motor (void)
{
    motor->HardStop();
}
/* PID function ----------------------------------------------------------------------*/
int act=0;
/*int PID_m (int er){
    integral = integral + er;
    act = kp*er + ki*integral;
    return act;
    }*/
/* Main ----------------------------------------------------------------------*/

int main() //main
{
    /* Variables ----------------------------------------------------------------------*/
    int i=0,error, yf,n=7;
    int32_t equivalente= 300.00; /*This value represent the maximal current that allow the motor*/
    double value_f, value_n;
//----- Initialization

    finalc.mode(PullUp);/*End-curse sensor*/

    /* Initializing SPI bus. */
    DevSPI dev_spi(D11, D12, D13);/** mosi, miso, sclk, ssel**/

    /* Initializing Motor Control Component. */
    motor = new POWERSTEP01(D2, D4, D8, D9, D10, dev_spi);

    if (motor->Init(&initDeviceParameters) != COMPONENT_OK) exit(EXIT_FAILURE);

    /* Attaching and enabling interrupt handlers. */
    motor->AttachFlagIRQ(&myFlagIRQHandler);
    motor->EnableFlagIRQ();
    motor->AttachBusyIRQ(&myBusyIRQHandler);
    motor->EnableBusyIRQ();

    /* Attaching an error handler */
    motor->AttachErrorHandler(&myErrorHandler);

    // Setup SPISlave
    SpiS.format(16, 0);
    SpiS.frequency(SPI_SPEED);

    /** main**/

    freio=1; /*Activate the brake in the motor*/
    int gate_time = 100,freq, multi=10; /**These values are necessary to measure the frequency speed in the encoder**/
    d_i = 0;

    EncoderInitialiseP(); /*Initialise the encoder to measure position*/

    while(1) { // Infinite loop

        /**With this button is know if the motor is operating normally**/
        if (mybutton == 0) {/*Initial test to test the safe operation*/
            motor->Move(StepperMotor::FWD, 1600);
            motor->WaitWhileActive();
            motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 800);

            motor->WaitWhileActive();
            TIM2->CNT=0;
            wait(1);
            pc.printf("%i\r\n", TIM2->CNT );

            motor->SetDeceleration(1000.0);
            motor->SetAcceleration(1000.0);
            motor->SetMaxSpeed(3125);
            motor->GoTo(10000);

        }
        finalc.fall(&para_motor);/*Measure the pulse generate in the end-curse sensor*/

        if(SpiS.receive()) { /*Receive the data from SPI for each task*/

            Tipo[i] = SpiS.read();
            i++;

            if (i==n) {
                i=0;
                /**If necessary to see the elements received through SPI bus uncomment here**/
                /*for (int j = 0; j <= 6; j++ ) {
                    pc.printf("Element[%d] = %i\r\n", j+1, Tipo[j] );// this element is used to visualizate
                                                                    //                  the elements sended
                }*/

                switch(Tipo[0]) { // Swicth externo
                        /* Passive*/

                    case 1:

                        freio=1; /* Disable the motor brake*/
                        wait(0.4);
                        Tipo[2]=Tipo[2]*100; /* Convert values in position for the actuator*/
                        Tipo[6]=Tipo[6]*100;
                        motor->SetMaxSpeed(Tipo[3]*50); /* Set maximal speed*/
                        /* Set motor torque*/
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_ACC, 15*Tipo[4]);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_DEC, 15*Tipo[4]);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_HOLD,15*Tipo[4]);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, 15*Tipo[4]);
                        /*Encoder Initialise*/
                        EncoderInitialiseP();
                        switch(Tipo[1]) { // Swicth interno

                            case 1:
                                /*Function go to home*/
                                motor->Move(StepperMotor::FWD, 1600);
                                motor->WaitWhileActive();
                                motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 1000);
                                finalc.fall(&para_motor);
                                motor->SetHome();
                                break;
                            case 2:
                                /* Function move between two points*/
                                if (Tipo[5]==0) {
                                    mover_motor(Tipo[2]);
                                    finalc.fall(&para_motor);
                                }
                                /* Move n times -cycles-*/
                                else {
                                    for (int k = 0; k < Tipo[5]; k++ ) {
                                        mover_motor(Tipo[2]);
                                        mover_motor(Tipo[6]);
                                        finalc.fall(&para_motor);
                                    }
                                }
                                break;

                        } // Swicth interno
                        break;
                        /* Isometric*/
                    case 2:
                        freio=0;
                        break;
                        /* Isotonic*/
                    case 3://
                        /* Set fixed weight*/
                        freio=1;
                        wait(0.3);
                        motor->SetMaxSpeed(30*100);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_ACC, equivalente);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_DEC, equivalente);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_HOLD, equivalente);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, equivalente);

                        motor->Move(StepperMotor::FWD, 1600);
                        motor->WaitWhileActive();
                        motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 800);
                        motor->WaitWhileActive();

                        EncoderInitialiseP();
                        wait(0.5);

                        int weight_ref = Tipo[3]*g;
                        TIM2->CNT=0;
                        while(Tipo[0]==3) {
                            value_f = recibe();
                            if (Tipo[0]==0) {
                                break;
                            }
                            finalc.fall(&para_motor);

                            error_t = weight_ref - value_f;
                            finalc.fall(&para_motor);
                            if (error_t < 0.0) {/*clockwise*/
                                finalc.fall(&para_motor);
                                /*--------Calculating speed value----------*/
                                /*It is used 0.7 as the length more long possible to achieve by the actuator,
                                The value ideal to multiply Vy must be 50000 but the motor isn't response*/

                                Vy = sqrt(Vo*Vo +(2*(value_f-Tipo[3]*g)/Tipo[3])*(0.7) );
                                Vy = 500*Vy;

                                /*-----------------------------------------*/
                                motor->Run(StepperMotor::FWD,Vy);
                                pc.printf("$%d %f;\r\n",  TIM2->CNT, value_f);/*Printing the position and force*/

                            } else if(error_t > 0.0 && (motor->ReadStatusRegister() & POWERSTEP01_STATUS_SW_F )==0) { /*counter-clock wise*/
                                /*--------Calculating speed value----------*/
                                Vy = sqrt(Vo*Vo +abs((2*(-value_f-Tipo[3]*g)/Tipo[3])*(-0.7)) );
                                Vy = 500*Vy;

                                /*-----------------------------------------*/
                                motor->Run(StepperMotor::BWD,Vy);
                                pc.printf("$%d %f;\r\n",  TIM2->CNT, value_f);
                            } else if(error_t == 0.0) {
                                motor->HardStop(); /*Stall the motor*/
                                pc.printf("$%d %f;\r\n",  TIM2->CNT, value_f);
                            }
                            wait(0.3);
                        }
                        break;
                        /* Elastic band*/
                    case 4:
                        /* Spring case */
                        /* f=kx */
                        motor->Move(StepperMotor::FWD, 1600);
                        motor->WaitWhileActive();
                        motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 800);
                        motor->WaitWhileActive();
                        EncoderInitialiseP();
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_ACC, equivalente);// Aqui se debe aumentar el factor de seguridad al menos en 2
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_DEC, equivalente);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_HOLD, equivalente);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, equivalente);
                        freio=1;
                        wait(0.3);
                        TIM2->CNT=0;
                        while(Tipo[0]==4) {
                            value_f = recibe();
                            if (Tipo[0]==0) {
                                break;
                            }
                            finalc.fall(&para_motor);
                            yf = calcula_n_steps(value_f);
                            /* Calculatin the error*/
                            error = yf-TIM2->CNT;
                            if (error  > 0.00) {/*clockwise*/
                                motor->Move(StepperMotor::FWD,abs(error));
                                /* This condition detect if have a change in the force performed*/
                                do {
                                    value_n = recibe();
                                    if (value_n < value_f) {
                                        motor->SoftStop();
                                    }
                                    if (Tipo[0]==0) {
                                        break;
                                    }
                                } while((motor->ReadStatusRegister() & POWERSTEP01_STATUS_BUSY)==1);

                            } else {/*counter-clock wise*/
                                motor->Move(StepperMotor::BWD,abs(error));
                                do {
                                    finalc.fall(&para_motor);
                                    value_n = recibe();
                                    if (value_n > value_f) {
                                        motor->SoftStop();
                                    }
                                    if (Tipo[0]==0) {
                                        break;
                                    }
                                    if ((motor->ReadStatusRegister() & POWERSTEP01_STATUS_SW_F )!=0) {
                                        motor->HardStop();
                                    }
                                } while((motor->ReadStatusRegister() & POWERSTEP01_STATUS_BUSY)==1);
                            }
                        }
                        //
                        break;
                    case 5:
                        /* Isokinetic */
                        motor->Move(StepperMotor::FWD, 1600);
                        motor->WaitWhileActive();
                        motor->GoUntil (ACTION_RESET, StepperMotor::BWD, 1000);
                        int C_initial= 200.00;/*Initial torque*/
                        /*Torque applied in acceleration and deceleration is bigger than run torque*/
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_ACC, C_initial*1.5);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_DEC, C_initial*1.5);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_HOLD, C_initial);
                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, C_initial);
                        freio=1;
                        wait(0.3);
                        int f_d = Tipo[3]*9.81;
                        value_f = recibe();
                        int data1 = 0;
                        int data2 = 0;
                        float porcent = 0.0;
                        EncoderInitialiseV();
                        while(Tipo[0]==5) {
                            /*Read the speed*/
                            TIM2->CNT = 0;
                            wait_ms(gate_time);                // Gate time for count
                            freq = TIM2->CNT*multi;            // read counter
                            while(value_f < f_d && value_f > -f_d) {
                                TIM2->CNT = 0;
                                wait_ms(gate_time);                // Gate time for count
                                freq = TIM2->CNT*multi;
                                value_f = recibe();
                                pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), freq, value_f, C_initial);
                                if (Tipo[0]==0) {
                                    break;
                                }
                            }
                            if (Tipo[0]==0) {
                                break;
                            }
                            pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), freq, value_f, C_initial);
                            //FWD
                            if (value_f > f_d) {/*clockwise*/
                                motor->Run(StepperMotor::FWD,Tipo[1]*50);// Constant speed
                                finalc.fall(&para_motor);
                                do {
                                    TIM2->CNT = 0;
                                    wait_ms(gate_time);                // Gate time for count
                                    freq = TIM2->CNT*multi;
                                    value_f = recibe();
                                    porcent = (abs(value_f-f_d)/f_d)*0.1; // Calculating the % to increase or decrease the torque
                                    finalc.fall(&para_motor);
                                    if (value_f > f_d) {
                                        data1 = C_initial*(1+porcent);
                                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, data1); // Change the torque calculate for the new force
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, data1);
                                    } else if (value_f >= 0.5*f_d && value_f < f_d) {
                                        data2 = C_initial*(1-porcent);
                                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, data2);// Change the torque calculate for the new force
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, data2);
                                    } else if (value_f < 0.5*f_d) {
                                        motor->SoftStop(); //stop the motor
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, C_initial);;
                                    }

                                } while(value_f >= 0.5*f_d);
                            }
                            //BWD
                            else if (value_f < -f_d) {/*counter-clock wise*/
                                motor->Run(StepperMotor::BWD,Tipo[1]*50); //constant speed
                                finalc.fall(&para_motor);
                                do {
                                    TIM2->CNT = 0;
                                    wait_ms(gate_time);                // Gate time for count
                                    freq = TIM2->CNT*multi;
                                    value_f = recibe();
                                    porcent = (abs(value_f-f_d)/f_d)*0.1;
                                    finalc.fall(&para_motor);
                                    if (value_f < -f_d) {
                                        data1 = C_initial*(1+porcent);
                                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, data1);// Change the torque calculate for the new force
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, data1);
                                    } else if (value_f <= -0.5*f_d && value_f > -f_d) {
                                        data2 = C_initial*(1-porcent);
                                        motor->SetAnalogValue(+ POWERSTEP01_TVAL_RUN, data2);// Change the torque calculate for the new force
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, data2);
                                    } else if (value_f > -0.5*f_d) {
                                        motor->SoftStop();//stop the motor
                                        pc.printf("$%d %d %d %d;\r\n",  motor->GetSpeed(), -freq, value_f, C_initial);
                                    }

                                } while(value_f <= -0.5*f_d && (motor->ReadStatusRegister() & POWERSTEP01_STATUS_SW_F )==0);
                            }



                        }


                        break;
                }// Swicth externo
            }// if

        }// if

    }// while
}// main