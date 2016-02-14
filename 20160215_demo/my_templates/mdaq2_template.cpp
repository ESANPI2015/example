/**
 * @file stm32/mdaq2/src/main.cpp
 *
 * @brief this is the main-sourcefile for the mdaq2-board.
 *
 * German Research Center for Artificial Intelligence\n
 * Project: iStruct
 *
 * @date some
 *
 * @author many
 */

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_it.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"

#include "adc.h"
#include "blackboard.h"
#include "config.h"
#include "dac.h"
#include "ita.h"
#include "jobs_angle.h"
#include "jobs_driftcomp.h"
#include "jobs_forcetorque.h"
#include "jobs_lis3lv02dq.h"
#include "jobs_phasedetect.h"
#include "jobs_scope.h"
#include "jobs_sensorarray.h"
#include "jobs_si1120.h"
#include "jobs_supportpolygon.h"
#include "jobs_temperature.h"
#include "jobs_transmit.h"
#include "jobs_vsprings.h"
#include "main.h"
#include "mux.h"
#include "spi.h"
#include "toggling.h"

#include "RobotConfig/Devices.h"
#include "representations/id.h"
#include "stm32common/comm.h"
#include "stm32common/usart.h"
#include "stm32common/clock.h"
#include "stm32common/distance_si1120.h"
#include "stm32common/isp.h"
#include "stm32common/message.h"
#include "stm32common/ping.h"
#include "stm32common/register.h"
#include "stm32common/jobs_heartbeat.h"
#include "stm32common/jobs_handle_incoming_packets.h"
#include "stm32common/led.h"
#include "stm32common/lis3lv02dq.h"
#include "stm32common/rtsched.h"
#include "stm32common/timing.h"
#include "stm32common/ichaus.h"
#include "<name>_comm.h"

#include <stdio.h>

/**
 * @addtogroup STM32
 * @{
 * @addtogroup STM32_Pcb
 * @{
 * @addtogroup STM32_Pcb_mdaq2
 * @{
 */

/**
 *  This is how variables are stored into flash:
 *  @code
 *      __attribute__((section(".dataflash"))) const boardSpecific_t boardConfig_inFlash;*
 *  @endcode
 *
 */

void mainEnableMCO(const char c)
{
    /* output clock on MCO pin */
    GPIO_InitTypeDef pa8;
    pa8.GPIO_Pin = GPIO_Pin_8;
    pa8.GPIO_Mode = c ? GPIO_Mode_AF_PP : GPIO_Mode_IN_FLOATING;
    pa8.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &pa8);

    switch(c)
    {
        case 0:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_NOCLOCK);
            break;
        case 1:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_SYSCLK);
            break;
        case 2:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_HSI);
            break;
        case 3:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_HSE);
            break;
        case 4:
            MODIFY_REG(RCC->CFGR, RCC_CFGR_MCO, RCC_CFGR_MCO_PLL);
            break;
    }
}

/* set prescalers for clocks */
void mainSetupSystemClock(void)
{
    /* Select HSI as clock source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    /* Enable external crystal oscillator*/
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(! (RCC->CR & RCC_CR_HSERDY));

    /* Enable Prefetch Buffer */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    /* Flash 2 wait state */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);

    /* set divider of HSE input if used as PLL source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE, RCC_CFGR_PLLXTPRE_HSE_Div2);
    /* set AHB prescaler to generate HCLK from SYSCLK */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
    /* APB2 prescaler: PCLK2 = HCLK */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
    /* APB1 prescaler: PCLK1 may not exceed 36 MHz! */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);

    /* PLL disable before configuration */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    /* PLL source: external clock (HSE) */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, RCC_CFGR_PLLSRC_HSE);
    /* PLL-Output := (HSE_VALUE / 2) * 12 */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL12);

    /* enable PLL and wait for stable output */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(! (RCC->CR & RCC_CR_PLLRDY));
    /* select pll output as system clock source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    /* store frequency of system clock in global variable */
    SystemCoreClock = (HSE_VALUE / 2) * 12;
}

static RTSNextCall job;
static <name>_comm_t comm;
static void jobsBehaviorGraph(void)
{
    <name>_process(&comm);

    job.time     = 1;/* us! */
    job.nextCall = jobsBehaviorGraph;
}
/**
 * The main program
 *
 * Peforms initialisation and other purposes
 *
 * @return  0 on exit
 */
int main(void)
{
    char buf[50];
    /* correct fucked up main clock */
    mainSetupSystemClock();

    /*Test if math support is linked in*/
    int32_t a;
    float b;
    a = -300;
    b = 1.0f / a;
    a = b * b;

    /* initialize memory used for registers and define dynamic register region*/
    registerInit(robotconfig::FIRST_DYNAMIC_REGISTER_ID_mdaq2);

    /* initialize communication for NDLCom */
    commInit(DEVICE_ID,LED_GREEN0);

    /* Read out actual board configuration */
    initBoardConfig();

    /*Intialize BlackBoard*/
    blackBoardInit();

    /* start real time clock. */
    clockRtcInit();
    /*Set RTC clock drift after init*/
    clockSetDrift(BOARD_CONFIG_RAM.RTC_DRIFT);

    /*
     * Enable clock for IO part of the chip:
     */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    /* io-pins used for GPIO (led for example, peripheral) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* uart1/3 */
    USARTx_Init(1,921600);
    USARTx_Init(3,921600);

    /* initialize debugging functions */
    messageInit();
    messageLevel = 3;
    messageDebug("*** iStruct MDAQ2 INIT ***");
    snprintf(buf, 50, "Version of %s at %s", __DATE__, __TIME__);
    messageDebug(buf);
    messageDebug("RTC, Registers, Communication initialized");

    /* set up leds */
    ledInit(sLeds);
    ledTest();
    ledSet(LED_GREEN);
    messageDebug("LEDs initialized");

    /* allows toggling a pin, for debugging */
    togglingInit();
    /* Timing, uses TIM5 */
    timingInit();
    messageDebug("TIM5 initialized");

    /* spi */
    SPI1_Configuration();
    messageDebug("SPI1 intitialized");
    /* spi */
    SPI2_Configuration();
    messageDebug("SPI2 intitialized");

    /* Initialize MUXs */
    MUX_Init();
    messageDebug("MUXs initialized");
    /* Initialize ADC */
    ADC_Init();
    messageDebug("ADC initialized");
    /* Initialize DAC */
    DAC_Init();
    messageDebug("DAC initialized");
    /* Initialize ITA */
    ITA_Init();
    messageDebug("IA initialized");

    /* prepare rtsched-datastructures */
    initRTSched(LED_YELLOW);
    messageDebug("Cooperative Real-Time Scheduler running");
    /* Initialize receiving job for all packets addressed to us. If handlers have been registered, a packet might be handled*/
    jobsHandleIncomingPacketsInit(LED_GREEN1);
    messageDebug("Receiving Job started");
    /*Ping Handler*/
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_RepresentationsPing, pingTryHandlePacket);
    /*Isp handler*/
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_IspCommand, ispTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_IspData, ispTryHandlePacket);
    /*Register Handler*/
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_RegisterValueWrite, registerTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_RegisterValueQuery, registerTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_RegisterDescriptionQuery, registerTryHandlePacket);
    /*BlackBoard Handler*/
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_Acceleration, blackBoardTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_FootAngles, blackBoardTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_DistanceSI1120, blackBoardTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_ForceTorque, blackBoardTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_SensorArrayMatrix, blackBoardTryHandlePacket);
    jobsHandleIncomingPacketsRegister(REPRESENTATIONS_REPRESENTATION_ID_Temperature, blackBoardTryHandlePacket);
    messageDebug("Core handlers registered");

    jobsHeartbeatInit(LED_GREEN);
    messageDebug("Heart-Beat running");
    jobsInitDriftComp();
    messageDebug("Drift Compensation enabled");

    messageDebug("Detected sensors:");
    uint8_t result;
    result = iCHausInit(ICHAUS_CLK_PORT, ICHAUS_CLK_PIN, ichausSensorList);
    if (result > 0)
    {
        snprintf(buf, 50, "> %d iCHaus sensors found", result);
        messageDebug(buf);
        jobsInitAngle();
    }
    result = LIS3LV02DQ_init();
    if (result == LIS3LV02DQ_OK) {
        snprintf(buf, 50, "> Accelerometer (LIS3LV02DQ) found");
        messageDebug(buf);
        /* Initialize Accelerometer */
        jobsLis3lv02dqInit();
    }
    result = distance_si1120Init(GPIOD, GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2);
    if (result) {
        snprintf(buf, 50, "> Distance sensor (SI1120) found");
        messageDebug(buf);
        /* initialize si1120 distance sensor and respective jobs*/
        jobsSi1120Init();
    }
    result = sensorArrayDetected();
    if (result) {
        snprintf(buf, 50, "> Sensor array detected");
        messageDebug(buf);
        jobsInitSensorArray();
    }
    result = forceTorqueSensorDetected();
    if (result) {
        snprintf(buf, 50, "> Force-Torque sensor detected");
        messageDebug(buf);
        jobsInitForceTorque();
    }
    messageDebug("*** Initializing and starting behavior graph ***");
    <name>_init(&comm, &commNDLComNode, 0, 0);
    job.time = 1;/* given in us */
    job.nextCall = jobsBehaviorGraph;
    rtsRegisterCall(&job);

    /* init remanining jobs */
    jobsInitPhaseDetector();
    jobsInitScope();
    jobsInitSupportPolygon();
    jobsInitTemperature();
    jobsInitTransmitter();
    jobsInitVirtualSprings();

    messageDebug("*** END OF INIT ***");

    /* enter event handler loop */
    rtsRunCalls();

    messageError("Real-Time Scheduler exited abnormally");

    /* Exit */
    return 0;
}

/**
 * @}
 * @}
 * @}
 */
