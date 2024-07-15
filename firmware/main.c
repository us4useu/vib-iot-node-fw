/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup cli_example_main main.c
 * @{
 * @ingroup cli_example
 * @brief An example presenting OpenThread CLI.
 *
 */

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_default_backends.h"

#include "thread_utils.h"

#include <openthread/thread.h>

//custom includes
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_util_platform.h"
#include <openthread/message.h>
#include <openthread/udp.h>

#include "arm_const_structs.h"

#define SCHED_QUEUE_SIZE      32                              /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum app_scheduler event size. */

#define UDP_PORT 1212

//custom defines
#define BTN_PIN           8
#define INT1_PIN          4
#define INT2_PIN          30

#define FIFO_BURST_SIZE   16 

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool  spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static const nrf_drv_timer_t TIMER_1S = NRF_DRV_TIMER_INSTANCE(0);

//adc data
static nrf_saadc_value_t         m_adc_buf[2] = {0x00};  /**< TX buffer. */
//spi data
#define SPI_XFER_LEN    (6*FIFO_BURST_SIZE)+1
static uint8_t          m_tx_buf[97];            /**< TX buffer. */
static uint8_t          m_rx_buf[97];            /**< RX buffer. */
static uint16_t         m_length = 0;             /**< Transfer length. */

volatile static int16_t        temperature = 2500;
volatile static int16_t        battery;

#define ACQ_BUF_SZ      8192

union axis_data_t {
    struct {
        q15_t x_axis[ACQ_BUF_SZ];
        q15_t y_axis[ACQ_BUF_SZ];
        q15_t z_axis[ACQ_BUF_SZ];
    } samples;
    uint8_t acq_data_buf[ACQ_BUF_SZ*6]; //48 kB
} data;

//axis_data_t data;

//static uint8_t          acq_data_buf[ACQ_BUF_SZ*6];

#define FFT_IN_SZ       ACQ_BUF_SZ/2
#define FFT_OUT_SZ      ACQ_BUF_SZ

uint32_t nsamples = ACQ_BUF_SZ;

static q15_t fft_input[FFT_IN_SZ]; //4 kB
static q15_t fft_output[FFT_OUT_SZ]; //8 kB

//iis2 utils
static const uint8_t  mems_TEMP_L         = 0x0B;
static const uint8_t  mems_TEMP_H         = 0x0C;
static const uint8_t  mems_read           = 0x80;
static const uint8_t  mems_ID_addr        = 0x0F;
static const uint8_t  mems_CTRL1_addr     = 0x20;
static const uint8_t  mems_CTRL3_addr     = 0x22;
static const uint8_t  mems_CTRL4_addr     = 0x23;
static const uint8_t  mems_FIFOCTRL_addr  = 0x2E;
static const uint8_t  lis2hh_ACTTHS_addr  = 0x1E;

static uint8_t mems = 0; //0 - iis2dlpc/ais2ih, 1 - lis2hh, else - not defined
static uint8_t odr = 5; //400 Hz default
static float32_t odr_hz = 400.0f;

static uint8_t rng = 3; //+/-8G defualt range

static const uint8_t  iis2dlpc_conf[6]    = {0b01100101,   // 200 Hz ODR, high performance mode
                                             0b00001100,   // BDU enabled
                                             0b00000010,   // soft SLP mode
                                             0b00000010,   // FTH INT on INT1 pin
                                             0b00000000,   // DRDY on INT2 pin
                                             0b10110100};  // BW ODR/2, 16g scale, low noise

//lis2hh defines

static const uint8_t  lis2hh_conf[6]    =   {0b01001111,   // HighRes disabled, 200 Hz ODR, BDU enabled, XYZ enabled 
                                             0b01000000,   // Filters disabled
                                             0b10000010,   // FIFO enabled, FIFO_FTH_INT on INT1 pin
                                             0b10110100,   // BW 100 Hz, FS=8g, auto BW, burst en, i2c en, SPI 4-wire 
                                             0b00000001,   // no debug, no decimation, no self-test, INT active high, OD on INT
                                             0b00000000};  // no INT2

//cmd processing data
#define CMD_ACQ_START   0x01
#define CMD_ACQ_STOP    0x02
#define CMD_ACQ_DONE    0x04
#define CMD_ACQ_STREAM  0x07

#define CMD_ACQ_READ    0x80
#define CMD_ACQ_RNXT    0x81
#define CMD_ACQ_RREP    0x82 
#define CMD_GET_DPTR    0x19
#define CMD_SET_DPTR    0x20

#define CMD_SET_CONF    0x40
#define CMD_GET_CONF    0x50
#define CMD_GET_STAT    0x60

#define STATE_IDLE        0x00
#define STATE_ERROR       0xFF
#define STATE_ACQ         0x01
#define STATE_DATAREADY   0x02
#define STATE_FFTREADY    0x03
#define STATE_LOG         0x04
#define STATE_ACQSTOP     0x10

#define UDP_DATABURST_NSAMPLES 10
#define UDP_DATABURST_SZ   60    //send 10 x 3 axis x 2 byte samples = 60 byte buffer at once 

#define MAX_CMD_QSIZE   32
uint8_t cmd_queue[MAX_CMD_QSIZE];
otIp6Address cmd_respdst[MAX_CMD_QSIZE]; //0.5 kB
volatile uint8_t cmd_wrptr = 0;
volatile uint8_t cmd_procptr = 0;

bool read_temp = false;

bool acq_data = false;
uint32_t acq_data_count = 0;
uint32_t acq_data_ptr = 0;
uint8_t udp_data_buf[UDP_DATABURST_SZ];

uint8_t tout = 60;
bool sleep = false;
uint8_t pst = 1; //awake default
uint8_t role = 0;
uint8_t axis = 1; //0 = XY, 1 = XZ, 2 = YZ

void saadc_init(void);

/***************************************************************************************************
 * @section State
 **************************************************************************************************/

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    role = otThreadGetDeviceRole(p_context);
    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                 flags, role);
}

/***************************************************************************************************
 * @section Initialization
 **************************************************************************************************/


 void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            //NRF_LOG_INFO("Timer event compare");
            //read_temp = true;
            saadc_init();
            nrf_drv_saadc_sample();
            if(tout>0)
            {
              if(role > 1) {
                tout--;
                //if(tout<6)
                  NRF_LOG_INFO("%d", tout);
              }
            }
            break;

        default:
            //Do nothing.
            break;
    }
}

/**@brief Function for initializing the Application Timer Module.
 */
static void timer_init(void)
{
    //uint32_t time_ms = 10000; //Time(in miliseconds) between consecutive compare events.
    //uint32_t time_ticks;

    uint32_t err_code = app_timer_init();

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&TIMER_1S, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

   // time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_1S, time_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_1S, NRF_TIMER_CC_CHANNEL0, 1000000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the LEDs.
 */
static void leds_init(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_ON(LEDS_MASK);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Thread Stack.
 */
static void thread_instance_init(void)
{
    thread_configuration_t thread_configuration =
    {
        .radio_mode        = THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,// THREAD_RADIO_MODE_RX_ON_WHEN_IDLE,
        .autocommissioning = true,
        .autostart_disable = false,
    };

    thread_init(&thread_configuration);
    //thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);

    uint32_t err_code = bsp_thread_init(thread_ot_instance_get());
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for deinitializing the Thread Stack.
 *
 */
static void thread_instance_finalize(void)
{
    bsp_thread_deinit(thread_ot_instance_get());
    thread_soft_deinit();
}


/**@brief Function for initializing scheduler module.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;

    if(acq_data == true)
    {
        if(acq_data_count + FIFO_BURST_SIZE <= nsamples) {
            //
            for(size_t n = 0; n < FIFO_BURST_SIZE; n++) {
                data.samples.x_axis[acq_data_count+n] = (q15_t)((m_rx_buf[2+(6*n)]<<8)+m_rx_buf[1+(6*n)]);
                data.samples.y_axis[acq_data_count+n] = (q15_t)((m_rx_buf[4+(6*n)]<<8)+m_rx_buf[3+(6*n)]);
                data.samples.z_axis[acq_data_count+n] = (q15_t)((m_rx_buf[6+(6*n)]<<8)+m_rx_buf[5+(6*n)]);
            }
            acq_data_count += FIFO_BURST_SIZE;
        }
        else
        {
            NRF_LOG_INFO("Buffer full");
            acq_data = false;
        }
    }
    //NRF_LOG_INFO("X: %6d, Y: %6d, Z: %6d",(int16_t)((m_rx_buf[2]<<8)+m_rx_buf[1]),(int16_t)((m_rx_buf[4]<<8)+m_rx_buf[3]),(int16_t)((m_rx_buf[6]<<8)+m_rx_buf[5]));
}

void nrf_adc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        adc_result = p_event->data.done.p_buffer[0];
        nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);

        battery = (int16_t)(((int32_t)adc_result*7200)/1024);
        //NRF_LOG_INFO("Battery = %d mV", battery);

        nrf_drv_saadc_uninit();

    }
}

void saadc_init()
{
    nrf_saadc_config_t saadc_conf;
    saadc_conf.resolution = SAADC_RESOLUTION_VAL_8bit;
    saadc_conf.oversample = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass;
    saadc_conf.buffer = m_adc_buf;
    saadc_conf.buffer_size = 1;

    APP_ERROR_CHECK(nrf_drv_saadc_init(NULL, nrf_adc_event_handler));
    nrf_saadc_channel_config_t  adc_batt_ch_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    adc_batt_ch_config.reference = NRF_SAADC_REFERENCE_VDD4;
    adc_batt_ch_config.gain = NRF_SAADC_GAIN1_4;
    nrf_drv_saadc_channel_init(0, &adc_batt_ch_config);

    nrf_drv_saadc_buffer_convert(&m_adc_buf[0], 1);
    //nrf_drv_saadc_buffer_convert(&m_adc_buf[1], 1);

}

void spi_init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG; //4MHz, SPI mode 0
    spi_config.mode     = NRF_DRV_SPI_MODE_3; //spi mode 3
    spi_config.frequency = NRF_DRV_SPI_FREQ_4M; //8 Mbps
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    spi_config.irq_priority = 6;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
}

void mems_read_temp(void)
{
    m_tx_buf[0] = mems_read + mems_TEMP_L;
    m_tx_buf[1] = 0b00000000;
    m_tx_buf[2] = 0b00000000;
    spi_xfer_done = false;
    m_length = 3;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
      __WFE();
    } 

    temperature = (int16_t)m_rx_buf[2];
    temperature = ((temperature * 256) + (int16_t)m_rx_buf[1]);
    temperature = temperature>>5;
    temperature = (temperature*125/10) +2500;
    //NRF_LOG_INFO("Temperature = " NRF_LOG_FLOAT_MARKER , NRF_LOG_FLOAT((((float)temperature)/100.0f)));
}

void mems_init(void)
{
    NRF_LOG_INFO("Config accelerometer.");

    m_tx_buf[0] = mems_read + mems_ID_addr;
    m_tx_buf[1] = 0x00;
    spi_xfer_done = false;
    m_length = 2;

    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
      __WFE();
    } 

    NRF_LOG_INFO("Chip ID register = 0x%02x", m_rx_buf[1]);

    if(m_rx_buf[1] == 0x44) {
      mems = 0;
    }
    else if(m_rx_buf[1] == 0x41) {
      mems = 1;
    }
    else {
      return;
    }

    if(mems == 0)
    {
      NRF_LOG_INFO("MEMS is IIS2DLPC / AIS2IH", m_rx_buf[1]);

      m_tx_buf[0] = mems_CTRL1_addr + 1;
      m_tx_buf[1] = 0b01000000;
      spi_xfer_done = false;
      m_length = 2;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
      while (!spi_xfer_done)
      {
        __WFE();
      } 

      nrf_delay_ms(10);

      m_tx_buf[0] = mems_CTRL1_addr;
      memcpy(m_tx_buf+1, iis2dlpc_conf, 6); //copy config
      spi_xfer_done = false;
      m_length = 7;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
      while (!spi_xfer_done)
      {
        __WFE();
      }    
    }
    else if(mems == 1)
    {  
      NRF_LOG_INFO("MEMS is LIS2HH12", m_rx_buf[1]);

      m_tx_buf[0] = mems_CTRL1_addr + 4;
      m_tx_buf[1] = 0b01000000;
      spi_xfer_done = false;
      m_length = 2;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
      while (!spi_xfer_done)
      {
        __WFE();
      } 

      nrf_delay_ms(10);

      m_tx_buf[0] = mems_CTRL1_addr;
      memcpy(m_tx_buf+1, lis2hh_conf, 6); //copy config
      spi_xfer_done = false;
      m_length = 7;

      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
      while (!spi_xfer_done)
      {
        __WFE();
      }
          
      m_tx_buf[0] = mems_FIFOCTRL_addr;
      m_tx_buf[1] = 0b00000001;
      m_length = 2;
      spi_xfer_done = false;
      APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
      while (!spi_xfer_done)
      {
        __WFE();
      } 
    } 
    else
    {
      NRF_LOG_INFO("Error: unknown chip ID", m_rx_buf[1]);
    }

}

void mems_set_rng(uint8_t rng)
{
    //NRF_LOG_INFO("Config RNG.");

    m_tx_buf[0] = mems_CTRL4_addr;
    m_tx_buf[1] = (lis2hh_conf[3] & 0b11001111) | ((rng & 0b00000011) << 4);
    m_length = 2;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
     // __WFE();
    }  
}

void mems_set_odr(uint8_t odr)
{
    //NRF_LOG_INFO("Config ODR.");
    m_tx_buf[0] = mems_CTRL1_addr;
    m_tx_buf[1] = (lis2hh_conf[0] & 0b10001111) | ((odr & 0b00000111) << 4);
    m_length = 2;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
     // __WFE();
    }  
}

void mems_start_fifo()
{
    m_tx_buf[0] = mems_FIFOCTRL_addr;
    m_tx_buf[1] = 0b00100000 + (0b00011111&FIFO_BURST_SIZE);
    m_length = 2;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
     // __WFE();
    }   
}

void mems_stop_fifo()
{
    m_tx_buf[0] = mems_FIFOCTRL_addr;
    m_tx_buf[1] = 0b00000001;
    m_length = 2;
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
    while (!spi_xfer_done)
    {
     // __WFE();
    }   
}

void mems_powerdown()
{
  mems_set_odr(0);
}


void mems_burst_read(uint8_t burst_size)
{
    memset(m_tx_buf, 0, ((6*burst_size) +1));
    m_tx_buf[0] = 0x80 + 0x28; //start addr of acc data
    m_length = ((6*burst_size) +1); //read 6* burst data + 1 byte for cmd
    spi_xfer_done = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
}

void int1_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    tout = 60;
    if(acq_data_count < nsamples)
      mems_burst_read(FIFO_BURST_SIZE);
    else
      NRF_LOG_INFO("Data buffer full");

}

void btn_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    tout = 60;
    if(sleep)
    {
      sleep = false;
    }
}

void btn_gpiote_init(void)
{ 
    ret_code_t err_code;
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;
    
    err_code = nrf_drv_gpiote_in_init(BTN_PIN, &in_config, btn_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BTN_PIN, true);
}

void int1_gpiote_init(void)
{ 
    ret_code_t err_code;
    //err_code = nrf_drv_gpiote_init();
    //APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    
    err_code = nrf_drv_gpiote_in_init(INT1_PIN, &in_config, int1_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT1_PIN, true);
}

static otUdpSocket sUdpSocket;

bool queueCMD2proc(uint8_t cmd, otIp6Address sourceaddr)
{
    if(cmd_wrptr<MAX_CMD_QSIZE-1)
    {
        if(cmd_procptr != cmd_wrptr+1)
        {
            cmd_queue[cmd_wrptr] = cmd;
            cmd_respdst[cmd_wrptr] = sourceaddr;
            cmd_wrptr++;
            //NRF_LOG_INFO("cmd_wrptr: %d", cmd_wrptr);
         }
         else
            return true;
    }
    else
    {
        if(cmd_procptr != 0)
        {
            cmd_queue[cmd_wrptr] = cmd;
            cmd_respdst[cmd_wrptr] = sourceaddr;
            cmd_wrptr=0;
            //NRF_LOG_INFO("cmd_wrptr: %d", cmd_wrptr);
        }
        else
            return true;
    }

    return false;
}


int16_t conf[8] = {6, 2, 4096, 1, 0, 0, 0 ,0}; //requested config
size_t set_acq_data_ptr = 0;

void processCMD(uint8_t* buf, otIp6Address sourceaddr)
{
    int cmd_space   = '_';
    uint8_t* ptr;
    uint8_t* cmd_ptr;
    uint8_t* payload_ptr;

    if(strncmp(buf, "SET_CONF", 8) == 0) {
        sscanf(buf + 8, "=%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd",
               &conf[0], &conf[1], &conf[2], &conf[3], &conf[4],
               &conf[5], &conf[6], &conf[7]);
        queueCMD2proc(CMD_SET_CONF, sourceaddr);
    }
    else if(strncmp(buf, "GET_CONF", 8) == 0) {
        queueCMD2proc(CMD_GET_CONF, sourceaddr);
    }
    else if(strncmp(buf, "GET_STAT", 8) == 0) {
        queueCMD2proc(CMD_GET_STAT, sourceaddr);
    }
    else if(strncmp(buf, "ACQ_START", 9) == 0) {
        queueCMD2proc(CMD_ACQ_START, sourceaddr);
    }
    else if(strncmp(buf, "ACQ_STOP", 8) == 0) {
        queueCMD2proc(CMD_ACQ_STOP, sourceaddr);
    }
    else if(strncmp(buf, "ACQ_DONE", 8) == 0) {
        queueCMD2proc(CMD_ACQ_DONE, sourceaddr);
    }
    else if(strncmp(buf, "GET_DPTR", 8) == 0) {
        queueCMD2proc(CMD_GET_DPTR, sourceaddr);
    }
    else if(strncmp(buf, "GET_DATA", 8) == 0) {
        queueCMD2proc(CMD_ACQ_RNXT, sourceaddr);
    }
    else if(strncmp(buf, "GET_REPE", 8) == 0) {
        queueCMD2proc(CMD_ACQ_RREP, sourceaddr);
    }
    else if(strncmp(buf, "SET_DPTR", 8) == 0) {
        sscanf(buf + 8, "=%d", &set_acq_data_ptr);
        queueCMD2proc(CMD_SET_DPTR, sourceaddr);
    }
    else if(strncmp(buf, "GET_STRM", 8) == 0) {
        queueCMD2proc(CMD_ACQ_STREAM, sourceaddr);
    }
    else { }
}

void handleUdpCMD(uint8_t* buf, uint16_t len, otIp6Address sourceaddr)
{
    uint8_t* ptr;
    uint8_t* prev_ptr;
    int cmd_limiter = '&';
    uint8_t cmd_ctr = 1;
    prev_ptr = buf;

    //find first CMD
    while(1)
    {
        ptr = strchr(prev_ptr, '&');
        if(ptr!= NULL) ptr[0] = '\0';
        NRF_LOG_INFO("CMD[%d]: %s",cmd_ctr, prev_ptr);
        processCMD(prev_ptr, sourceaddr);
        if(ptr==NULL)
        {
          break;
        }
        cmd_ctr++;
        prev_ptr = ptr+1;
    }

}

otInstance *aInstance;

void handleUdpRx(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
      uint8_t buf[500];
      int length;

      length      = otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, sizeof(buf) - 1);
      buf[length] = '\0';

      //NRF_LOG_INFO("Received %d byte UDP message", length);
      handleUdpCMD(buf, length, aMessageInfo->mPeerAddr);
      //NRF_LOG_INFO("Received UDP message from IP: %s", otIp6Address);
}

const otMessageSettings messageSettings = {false, OT_MESSAGE_PRIORITY_NORMAL};
static otUdpSocket sUdpSocket;


otError UdpTx(uint8_t* buf, otIp6Address dstaddr)
{
    otError       error = OT_ERROR_NONE;
    otMessageInfo messageInfo;
    otMessage*    message = NULL;

    memset(&messageInfo, 0, sizeof(messageInfo));

    messageInfo.mPeerAddr = dstaddr;
    messageInfo.mPeerPort = (uint16_t)9292;
    
    message = otUdpNewMessage(aInstance, NULL);
    if(message == NULL)
    { 
        error = OT_ERROR_NO_BUFS;
        //NRF_LOG_INFO("Error otUdpNewMessage %d", error);
        return error;
    }

    //NRF_LOG_INFO("Massage append %s, len %d ", buf, (uint16_t)strlen(buf));

    //error = otMessageWrite(message, 0, buf, (uint16_t)strlen(buf));
    error = otMessageAppend(message, buf, (uint16_t)strlen(buf));
    error = otMessageAppend(message, "\n", 1);
    //
    if(error != OT_ERROR_NONE)
    {
        //NRF_LOG_INFO("Error otMessageAppend %d", error);
        if (message != NULL)
        {
            otMessageFree(message);
        }
        return error;
    }

    error = otUdpSend(aInstance, &sUdpSocket, message, &messageInfo);
    //NRF_LOG_INFO("Error otUdpSend %d", error);
    //otMessageFree(message);
    return error;
}

#define ACQ_RAW     0
#define ACQ_FFT     1
#define ACQ_LOG     2

uint16_t acq_type = 0;

void set_config(int16_t* config) 
{
    //verify/set ODR
    const uint16_t odrLUT[7] = {0, 10, 50, 100, 200, 400, 800};
    if(config[0] > 0 && config[0] < 7){
        odr = config[0];
        odr_hz = (float)odrLUT[odr];
        mems_set_odr(odr);
    }
    else { config[0] = odr; }
    
    //verify/set RANGE
    if(config[1] < 3){ rng = config[1]; }
    else { config[1] = rng; }
    
    //verify/set NSAMPLES
    if(config[2] == 512 || config[2] == 1024 || config[2] == 2048 || config[2] == 4096 || config[2] == 8192){
        nsamples = config[2];
    }
    else { config[2] = nsamples; }

    //verify/set PWR STATE
    if(config[3] < 3) { pst = config[3]; }
    else { config[3] = pst; }

    //verify/set ACQ TYPE
    if(config[4] < 3) { acq_type = config[4]; }
    else { config[4] = acq_type; }
}

int main(int argc, char *argv[])
{
    char strbuf[128]; //temp buffer for udp response

    log_init();
    scheduler_init();
    timer_init();
    leds_init();

    btn_gpiote_init();
    int1_gpiote_init();

    sleep = false;
     while(sleep)
    {
      __WFI();
    }

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("start");

    spi_init();
    mems_stop_fifo();
    mems_init();

    set_config(conf);

    bool mems_fifo_running = false;
    otError       err = OT_ERROR_NONE;

    uint16_t udp_sendsize = 0;

    otIp6Address udpSrvAddr;
    bool addrSet = false;

    while (true)
    {
        nrf_delay_ms(250);
        thread_instance_init();

        if(!mems_fifo_running)
        {
            aInstance = thread_ot_instance_get();

            otSockAddr  listenSockAddr;

            memset(&sUdpSocket, 0, sizeof(sUdpSocket));
            memset(&listenSockAddr, 0, sizeof(listenSockAddr));

            listenSockAddr.mPort    = UDP_PORT; //1212

            otUdpOpen(aInstance, &sUdpSocket, handleUdpRx, aInstance);
            otUdpBind(aInstance, &sUdpSocket, &listenSockAddr, OT_NETIF_UNSPECIFIED);

            nrf_drv_timer_enable(&TIMER_1S);
        }

        //mems_start_fifo();
        //mems_fifo_running = true;
        //acq_data = true;

        bool processingDone = false;
        size_t data_out_count = 0;
        bool streamData = false;

        uint8_t state = STATE_IDLE;// IDLE
        otError prev_err = OT_ERROR_NONE;

        while (!thread_soft_reset_was_requested())
        {
            thread_process();
            app_sched_execute();

            if(cmd_procptr<cmd_wrptr || (cmd_procptr==MAX_CMD_QSIZE-1 && cmd_wrptr == 0))
            {
               tout = 60;
               switch(cmd_queue[cmd_procptr])
               {
                  case CMD_SET_CONF:
                      set_config(conf);
                      sprintf(strbuf, "CONF=%d,%d,%d,%d,%d,%d,%d,%d",
                                      conf[0], conf[1], conf[2], conf[3], 
                                      conf[4],conf[5], conf[6], conf[7]);

                      udpSrvAddr = cmd_respdst[cmd_procptr];
                      addrSet = true;
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_GET_CONF:
                      sprintf(strbuf, "CONF=%d,%d,%d,%d,%d,%d,%d,%d",
                                      conf[0], conf[1], conf[2], conf[3], 
                                      conf[4],conf[5], conf[6], conf[7]);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_GET_STAT:
                      sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                      temperature, battery, 0, data_out_count,
                                      state,acq_data_ptr,0,0);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_SET_DPTR:
                      NRF_LOG_INFO("Setting pointer %d", set_acq_data_ptr);
                      //sprintf(strbuf, "DPTR=%d", acq_data_ptr);
                      if(set_acq_data_ptr < data_out_count){
                          acq_data_ptr = set_acq_data_ptr;
                          if(acq_data_ptr + UDP_DATABURST_NSAMPLES <= data_out_count) { 
                              udp_sendsize = UDP_DATABURST_NSAMPLES;
                          }
                          else {
                              udp_sendsize = (uint16_t)(data_out_count-acq_data_ptr);
                          }
                          sprintf(&strbuf[0], "P");
                          sprintf(&strbuf[1], "%04d=", acq_data_ptr);
                          for(int i = 0; i<udp_sendsize; i++) { 
                              sprintf(&strbuf[(12*i)+6], "%04X%04X%04X",  (uint16_t)data.samples.x_axis[acq_data_ptr], 
                                                                          (uint16_t)data.samples.y_axis[acq_data_ptr], 
                                                                          (uint16_t)data.samples.z_axis[acq_data_ptr] );
                              acq_data_ptr++;
                          }
                          err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                          if(err != OT_ERROR_NONE) {
                              NRF_LOG_INFO("%d", err);
                          }
                      }
                      break;
                  case CMD_GET_DPTR:
                      //sprintf(strbuf, "DPTR=%d", acq_data_ptr);
                      sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                      temperature, battery, 0, data_out_count,
                                      state,acq_data_ptr,0,0);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_ACQ_STREAM:
                      nrf_drv_timer_disable(&TIMER_1S);
                      streamData = true;
                      break;
                  case CMD_ACQ_START:
                      nrf_drv_timer_disable(&TIMER_1S);
                      spi_xfer_done = false;
                      mems_fifo_running = true;
                      acq_data_count = 0;
                      data_out_count = 0;
                      mems_start_fifo();
                      acq_data = true;
                      processingDone = false;
                      if(acq_type == ACQ_LOG) { state = STATE_LOG; }
                      else { state = STATE_ACQ; }
                      sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                      temperature, battery, 0, data_out_count,
                                      state,acq_data_ptr,0,0);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_ACQ_STOP:
                      acq_data = false;
                      mems_stop_fifo();
                      mems_fifo_running = false;
                      state = STATE_ACQSTOP;
                      acq_data_ptr = 0;
                      sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                      temperature, battery, 0, data_out_count,
                                      state,acq_data_ptr,0,0);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_ACQ_DONE:
                      state = STATE_IDLE;
                      sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                      temperature, battery, 0, data_out_count,
                                      state,acq_data_ptr,0,0);
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      nrf_drv_timer_enable(&TIMER_1S);
                      break;
                  case CMD_ACQ_RNXT:
                      nrf_drv_timer_disable(&TIMER_1S);

                      if(acq_data_ptr + UDP_DATABURST_NSAMPLES <= data_out_count) { 
                          udp_sendsize = UDP_DATABURST_NSAMPLES;
                      }
                      else {
                          udp_sendsize = (uint16_t)(data_out_count-acq_data_ptr);
                      }

                      for(int i = 0; i<udp_sendsize; i++) { 
                          sprintf(&strbuf[12*i], "%04X%04X%04X",  (uint16_t)data.samples.x_axis[acq_data_ptr], 
                                                                  (uint16_t)data.samples.y_axis[acq_data_ptr], 
                                                                  (uint16_t)data.samples.z_axis[acq_data_ptr] );
                          acq_data_ptr++;
                      }
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  case CMD_ACQ_RREP:
                      acq_data_ptr -= udp_sendsize;
                      NRF_LOG_INFO("CMD_ACQ_RREP @: %d", acq_data_ptr);                         
                      for(int i = 0; i<udp_sendsize; i++) { 
                          sprintf(&strbuf[12*i], "%04X%04X%04X",  (uint16_t)data.samples.x_axis[acq_data_ptr], 
                                                                  (uint16_t)data.samples.y_axis[acq_data_ptr], 
                                                                  (uint16_t)data.samples.z_axis[acq_data_ptr]);
                          acq_data_ptr++;
                      }
                      err = UdpTx(strbuf, cmd_respdst[cmd_procptr]);
                      if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                      }
                      break;
                  default:
                      break;
                 
               }
               cmd_procptr++;
               if(cmd_procptr>=MAX_CMD_QSIZE)
                  cmd_procptr=0;
            }

            if(acq_data_count == nsamples && !processingDone) {
                if(acq_type == ACQ_RAW) {
                    data_out_count = acq_data_count;
                    if(addrSet) {
                        state = STATE_DATAREADY;
                        sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                        temperature, battery, 0, data_out_count,
                                        state,acq_data_ptr,0,0);
                        err = UdpTx(strbuf, udpSrvAddr);
                        if(err != OT_ERROR_NONE) {
                          NRF_LOG_INFO("%d", err);
                        }
                    }
                    processingDone = true;
                    acq_data = false;
                    mems_stop_fifo();
                    mems_fifo_running = false;
                }
                else {
                    q15_t maxX, maxY, maxZ;
                    float32_t freqX, freqY, freqZ;
              
                    uint32_t maxIndex;
                    arm_rfft_instance_q15 S;
              
                    size_t fft_size = nsamples/2;
                    size_t fft_out_size = nsamples;

                    //Init FFT
                    arm_rfft_init_q15(&S, fft_size, 0, 1);

                    //Perform FFT on axis X
                    arm_rfft_q15(&S, data.samples.x_axis, fft_output); //arm_rfft_q15(&S, fft_input, fft_output);
                    arm_abs_q15(fft_output, fft_output, fft_size);

                    if(acq_type == ACQ_FFT) {
                        memcpy(data.samples.x_axis, fft_output, fft_size*sizeof(q15_t));
                    }

                    fft_output[0] = 0;
                    arm_max_q15(fft_output, fft_size, &maxX, &maxIndex);
                    freqX = (odr_hz * maxIndex) / (2*fft_size);

                   //Perform FFT on axis Y
                    arm_rfft_q15(&S, data.samples.y_axis, fft_output); //arm_rfft_q15(&S, fft_input, fft_output);
                    arm_abs_q15(fft_output, fft_output, fft_size);

                    if(acq_type == ACQ_FFT) {
                        memcpy(data.samples.y_axis, fft_output, fft_size*sizeof(q15_t));
                    }
                    
                    fft_output[0] = 0;
                    arm_max_q15(fft_output, fft_size, &maxY, &maxIndex);
                    freqY = (odr_hz * maxIndex) / (2*fft_size);
              
                    //Perform FFT on axis Z
                    arm_rfft_q15(&S, data.samples.z_axis, fft_output); //arm_rfft_q15(&S, fft_input, fft_output);
                    arm_abs_q15(fft_output, fft_output, fft_size);

                    if(acq_type == ACQ_FFT) {
                        memcpy(data.samples.z_axis, fft_output, fft_size*sizeof(q15_t));
                    }
                    
                    fft_output[0] = 0;
                    arm_max_q15(fft_output, fft_size, &maxZ, &maxIndex);
                    freqZ = (odr_hz * maxIndex) / (2*fft_size);

                    NRF_LOG_INFO("Fx = " NRF_LOG_FLOAT_MARKER ", Mx = %d", NRF_LOG_FLOAT(freqX), maxX);
                    NRF_LOG_INFO("Fy = " NRF_LOG_FLOAT_MARKER ", My = %d", NRF_LOG_FLOAT(freqY), maxY);
                    NRF_LOG_INFO("Fz = " NRF_LOG_FLOAT_MARKER ", Mz = %d", NRF_LOG_FLOAT(freqZ), maxZ);
                    
                    if(acq_type == ACQ_LOG) {
                        if(addrSet) {
                            sprintf(strbuf, "LOG=%d,%d,%d,%d,%d,%d",(int32_t)(freqX*100.0f),(int16_t)maxX,
                                                                    (int32_t)(freqY*100.0f),(int16_t)maxY,
                                                                    (int32_t)(freqZ*100.0f),(int16_t)maxZ);
                            err = UdpTx(strbuf, udpSrvAddr);
                            if(err != OT_ERROR_NONE) {
                              NRF_LOG_INFO("%d", err);
                            }
                        }
                        acq_data_count = 0;
                        data_out_count = 0;
                        processingDone = false;
                    }
                    else {
                        processingDone = true;
                        data_out_count = acq_data_count/2;
                        if(addrSet) {
                          state = STATE_FFTREADY;
                          sprintf(strbuf, "STAT=%d,%d,%d,%d,%d,%d,%d,%d", 
                                        temperature, battery, 0, data_out_count,
                                        state,acq_data_ptr,0,0);
                          err = UdpTx(strbuf, udpSrvAddr);
                          if(err != OT_ERROR_NONE) {
                              NRF_LOG_INFO("%d", err);
                          }
                        }
                        acq_data = false;
                        mems_stop_fifo();
                        mems_fifo_running = false;
                    }
                }
            }

            if(read_temp)
            {
                mems_read_temp();
                read_temp = false;
            }

            if (NRF_LOG_PROCESS() == false && streamData == false)
            {
                thread_sleep();
            }

            if((tout == 0 || pst == 2) && pst != 1)
            {
              if(!sleep) {
                NRF_LOG_INFO("sleep");
                mems_powerdown();
                NVIC_SystemReset();
              }
              __WFI();
            }

            if(streamData) {
                if(acq_data_ptr < data_out_count) {
                    uint32_t start_ptr = acq_data_ptr;
                    if(acq_data_ptr + UDP_DATABURST_NSAMPLES <= data_out_count) { 
                        udp_sendsize = UDP_DATABURST_NSAMPLES;
                    }
                    else {
                        udp_sendsize = (uint16_t)(data_out_count-acq_data_ptr);
                    }
                    sprintf(&strbuf[0], "P");
                    sprintf(&strbuf[1], "%04d=", acq_data_ptr);
                    for(int i = 0; i<udp_sendsize; i++) { 
                        /*sprintf(&strbuf[(12*i)+6], "%04X%04X%04X",  (uint16_t)data.samples.x_axis[acq_data_ptr], 
                                                                    (uint16_t)data.samples.y_axis[acq_data_ptr], 
                                                                    (uint16_t)data.samples.z_axis[acq_data_ptr]);*/
                        sprintf(&strbuf[(12*i)+6], "%04X%04X%04X",  (uint16_t)acq_data_ptr, 
                                                                    (uint16_t)acq_data_ptr, 
                                                                    (uint16_t)acq_data_ptr);
                        acq_data_ptr++;
                    }
                    err = UdpTx(strbuf, udpSrvAddr);;
                    if(err != OT_ERROR_NONE) {
                        if(prev_err != err)
                            prev_err = err;
                        prev_err = err;
                        //NRF_LOG_INFO("%d", err);
                        acq_data_ptr = start_ptr;
                        //nrf_delay_ms(10);
                    }
                    else {
                        prev_err = err;
                        //NRF_LOG_INFO("%d of %d", acq_data_ptr, data_out_count);
                    }
                }
                else {
                    NRF_LOG_INFO("stream done");
                    streamData = false;
                }
            }
        }

        thread_instance_finalize();
    }
}

/**
 *@}
 **/
