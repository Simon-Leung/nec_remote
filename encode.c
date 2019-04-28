#include "rtthread.h"
#include "board.h"
#include "stdint.h"
#include <rtdevice.h>

#define CARRIER_TIM TIM8 /* pwm creat 38Khz carrier, duty cycle is 1/3 */

#define logic_high() TIM_SetCompare4(CARRIER_TIM, 630) /* send logic 1 */
#define logic_low() TIM_SetCompare4(CARRIER_TIM, 0) /* send logic 0 */

#define PULSE_DELAY (560) // 560us
#define LOGIC_1_DELAY (1680) // 1680us
#define LOGIC_0_DELAY (560) // 560us
#define INVOKE_LOW_DELAY (9000) // 9ms, logic 1
#define INVOKE_HIGH_DELAY (4500) // 4.5ms, logic 0
#define STOP_DELAY (300) // 300us

static struct rt_completion trans_cmp;

static void delay_tim_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler =(72-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig( TIM6, TIM_IT_Update, ENABLE);
}

static void tim_delay(uint32_t us)
{

    TIM_SetAutoreload(TIM6, us - 1);
    TIM_Cmd(TIM6, ENABLE);
    TIM_ClearFlag(TIM6, TIM_IT_Update);
    rt_completion_wait(&trans_cmp, RT_WAITING_FOREVER);
    TIM_Cmd(TIM6, DISABLE);
}

void TIM6_IRQHandler(void)
{
    rt_completion_done(&trans_cmp);
    TIM_ClearFlag(TIM6, TIM_IT_Update);
}

static void pwm_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef  TIM_OCInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* PWM freq = 72000/(1894+1)=38Khz  */
    TIM_TimeBaseStructure.TIM_Period = 1894;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(CARRIER_TIM, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 630;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(CARRIER_TIM, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(CARRIER_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(CARRIER_TIM, ENABLE);
    TIM_Cmd(CARRIER_TIM, ENABLE);
    TIM_CtrlPWMOutputs(CARRIER_TIM, ENABLE);
}

static void create_bit(uint8_t bit)
{
    logic_high();
    tim_delay(PULSE_DELAY);
    logic_low();
    if(bit)
    {
        tim_delay(LOGIC_1_DELAY);
    }
    else
    {
        tim_delay(LOGIC_0_DELAY);
    }
}

static void create_invoke(void)
{
    logic_high();
    tim_delay(INVOKE_LOW_DELAY);
    logic_low();
    tim_delay(INVOKE_HIGH_DELAY);
}

static void create_stop(void)
{
    logic_high();
    tim_delay(STOP_DELAY);
    logic_low();
    tim_delay(STOP_DELAY);
}

void remote_encode(uint8_t value, uint8_t addr)
{
    int8_t i;
    uint32_t Send_code=(uint8_t)(~addr) + ((uint8_t)(addr)<<8) + ((uint8_t)(~value)<<16) + ((uint8_t)(value)<<24);

    create_invoke();
    for(i=31; i >= 0; i--)
    {
        create_bit((Send_code & (1<<i)) != 0);
    }
    create_stop();
}

int encode_init(void)
{
    rt_completion_init(&trans_cmp);
    delay_tim_init();
    rt_pin_mode(2, PIN_MODE_OUTPUT);
    rt_pin_write(2, PIN_HIGH);
    pwm_init();
    logic_low();
    return RT_EOK;
}

