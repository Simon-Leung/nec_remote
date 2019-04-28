#include "rtthread.h"
#include "board.h"
#include "stdint.h"
#include <rtdevice.h>

#define REMOTE_RX_MAX 256

struct remote_rx_fifo
{
    /* software fifo */
    uint8_t buffer[REMOTE_RX_MAX];

    uint16_t put_index, get_index;

    uint8_t is_full;
};

volatile uint32_t capt_val[32];
static struct rt_completion recv_cmp;
struct remote_rx_fifo rx_fifo;
rt_err_t (*rx_indicate)(rt_size_t size);

void remote_encode(uint8_t addr, uint8_t *buff, int sz);
int encode_init();

/*
 * Configure TIM channel 1 as input capture with falling
 * edge detection
 */
static void remote_tim_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_1);


    TIM_TimeBaseStructure.TIM_Period = 10000;
    TIM_TimeBaseStructure.TIM_Prescaler =(72-1);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x03;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);

    TIM_Cmd(TIM5,ENABLE );

    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig( TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);

}

/*
 * The interrupt is configured to trigger on every falling edge of signal
 */
void TIM5_IRQHandler(void)
{
    #define FREQ_80HZ 12500
    #define FREQ_70HZ 14285
    #define FREQ_1000HZ 1000
    #define FREQ_300HZ 3333
    #define FREQ_12_5HZ 80000
    #define FREQ_7HZ 150000
    volatile uint32_t capture = 0;
    static  int8_t cnt = -1;

    if(TIM_GetITStatus(TIM5,TIM_IT_CC2) != RESET)
    {
        capture = TIM_GetCapture2(TIM5);
        /* invoke bit, period > 13.5 ms */
        if(((capture > FREQ_80HZ) && (capture < FREQ_70HZ)) && (cnt == -1))
        {
            cnt ++; /* increase counter and skip first measure */
        }

        /* logical bit, period is equal to 1.25 ms or 2.25 ms */
        if((capture > FREQ_1000HZ && capture < FREQ_300HZ) &&(cnt >= 0))
        {
            capt_val[cnt] = capture; /* write to buffer */
            cnt ++;

            if(cnt == 32) /* buffer is full, mark that transmission is done */
            {
                rt_completion_done(&recv_cmp);
                cnt = -1;
            }
        }
        /* stop bit, period is equal to 110 ms */
        if((capture > FREQ_12_5HZ) && (capture < FREQ_7HZ))
        {
            rt_completion_done(&recv_cmp);
        }
        TIM_SetCounter(TIM5, 0);
    }
    TIM_ClearFlag(TIM5, TIM_IT_Update | TIM_IT_CC2);
}

static void remote_rx(uint8_t ch)
{
    rt_base_t level;

    /* disable interrupt */
    level = rt_hw_interrupt_disable();
    rx_fifo.buffer[rx_fifo.put_index] = ch;
    rx_fifo.put_index += 1;
    if (rx_fifo.put_index >= sizeof(rx_fifo.buffer))
        rx_fifo.put_index = 0;

    /* if the next position is read index, discard this 'read char' */
    if (rx_fifo.put_index == rx_fifo.get_index)
    {
        rx_fifo.get_index += 1;
        rx_fifo.is_full = RT_TRUE;
        if (rx_fifo.get_index >= sizeof(rx_fifo.buffer))
            rx_fifo.get_index = 0;
    }
    /* enable interrupt */
    rt_hw_interrupt_enable(level);

    /* invoke callback */
    if (rx_indicate != RT_NULL)
    {
        rt_size_t rx_length;

        /* get rx length */
        level = rt_hw_interrupt_disable();
        if(rx_fifo.put_index >= rx_fifo.get_index)
        {
            rx_length = rx_fifo.put_index - rx_fifo.get_index;
        }
        else
        {
            rx_length = sizeof(rx_fifo.buffer) - (rx_fifo.get_index - rx_fifo.put_index);
        }
        rt_hw_interrupt_enable(level);

        if (rx_length)
        {
            rx_indicate(rx_length);
        }
    }
}


/*
 * Decode signal and return as 32 bit quantity
 */
int remote_decode(int timeout)
{
    #define FREQ_500HZ 2000
    uint32_t dec = 0;
    int rc = -RT_ERROR;

    if(rt_completion_wait(&recv_cmp, timeout) == RT_EOK)
    {
        for(int i = 0; i < sizeof(capt_val); i++)
        {
            /* value is equal to 1 */
            if(capt_val[i] >= FREQ_500HZ)
            {
                dec |= (1 << (31 - i));

            }
            /* value is equal to 0 */
            else
            {
                dec |= (0 << (31 - i));
            }
        }
        {
            uint8_t t1, t2;
            t1 = dec >> 24;
            t2 = dec >> 16;
            /* address */
            if(t1 == (~t2))
            {
                t1 = dec >> 8;
                t2 = dec;
                if(t1 == (~t2))
                {
                    remote_rx(t1);
                    rc = RT_EOK;
                }
            }
        }
    }
    else
    {
        rc = -RT_ETIMEOUT;
    }
    return rc;
}

static void remote_thread(void *prama)
{
    uint8_t ch = 0xa2;

    while(1)
    {
        remote_encode(0x0, &ch, 1);
        remote_decode(RT_TICK_PER_SECOND);
    }
}

static void remote_rx_init(void)
{
    rt_memset(rx_fifo.buffer, 0, sizeof(rx_fifo.buffer));
    rx_fifo.put_index = 0;
    rx_fifo.get_index = 0;
    rx_fifo.is_full = RT_FALSE;
}

/*
 * Initialize NEC protocol transmission
 */
int remote_init(void)
{
    encode_init();
    rt_completion_init(&recv_cmp);
    remote_rx_init();
    remote_tim_init();
    {
        rt_err_t result;
        ALIGN(RT_ALIGN_SIZE)
        static uint8_t stack[1024];
        static struct rt_thread thread;

        /* init led thread */
        result = rt_thread_init(&thread,
                                "remote",
                                remote_thread,
                                RT_NULL,
                                (uint8_t*)&stack[0],
                                sizeof(stack),
                                21,
                                RT_TICK_PER_SECOND);
        if (result == RT_EOK)
        {
            rt_thread_startup(&thread);
        }
    }
    return RT_EOK;
}
INIT_APP_EXPORT(remote_init);


