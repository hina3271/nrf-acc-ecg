/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_drv_timer.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_ecg.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "boards.h"
//#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "LPFilter.h"
#include "lib/ads/ads.h"
#include "lib/circbuf/circbuf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

const char version[] = COMMIT_HASH;

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "ARGOS_ECG"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                160 // 100ms                                          
#define APP_ADV_DURATION                0  // Infinite
#define APP_ADV_INTERVAL_SLOW           1600
#define APP_ADV_DURATION_SLOW           120000

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define NORMAL_ADC_SAMPLES 7000
#define ABNORMAL_ADC_SAMPLES 6260
extern const int16_t mock_normal_data[NORMAL_ADC_SAMPLES];
extern const int16_t mock_abnormal_data[ABNORMAL_ADC_SAMPLES];

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint32_t   ecg_control            = 0;
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};

bool filter_enable = true;
#define ADSBUFFER ads_buf
#define ADSBUFFERDEPTH 1024
CIRC_GBUF_DEF(int32_t, ADSBUFFER, ADSBUFFERDEPTH)

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

void get_mock_normal_adc(int32_t* data, int length) {
  static int offset = 0;
  for (int ii = 0; ii < length; ii++) {
    data[ii] = mock_normal_data[offset];
    offset++;
    offset %= NORMAL_ADC_SAMPLES;
  }
}

bool get_ads_data(int32_t* data, int length){
  int data_count;
  // Checks if we have enough data in the queue.
  data_count = ADSBUFFERDEPTH - CIRC_GBUF_FS(ADSBUFFER);
  if(data_count < length && data_count > 0)
    return false;

  for(int ii = 0; ii < length; ii++) {
    CIRC_GBUF_POP(ADSBUFFER, &data[ii]);
  }
  return true;
}

void get_mock_abnormal_adc(int32_t* data, int length) {
  static int offset = 0;
  for (int ii = 0; ii < length; ii++) {
    data[ii] = mock_abnormal_data[offset];
    offset++;
    offset %= ABNORMAL_ADC_SAMPLES;
  }
}

void get_ramp(int32_t* data, int length) {
  static int16_t value = 0;
  for(int ii = 0; ii < length; ii++) {
    data[ii] = value++;
  }
}

void get_constant(int32_t* data, int length, int16_t constant) {
  for(int ii = 0; ii < length; ii++) {
    data[ii] = constant;
  }
}

void update_led(uint8_t mask) {
  for (int ii = 0; ii < 4; ii++) {
    if ((mask >> ii) & 0x1) {
      bsp_board_led_on(ii);
    } else {
      bsp_board_led_off(ii);
    }
  }
}

/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
  
    // 1 uint32 packet count, 1 byte status, 30 int32 samples
    const int kNumSamples = 30;
    const uint16_t data_length = sizeof(uint32_t) + sizeof(int32_t)*kNumSamples + 1;
    static uint8_t index = 10;
    uint32_t           err_code;
    static uint32_t timer_count = 0;
    uint32_t status = 0;
    int timer_count_size = sizeof(timer_count);
    bool update;
    static int call_count = 0;


    static uint32_t packet_count =0;
    uint32_t * data_array_ptr;

    ble_arrhythmia_s ble_arrhythmia;
    int16_t ble_arrhythmia_size = sizeof(ble_arrhythmia);
    update = true;

    data_array_ptr = (int32_t*) (data_array + sizeof(uint32_t) + 1);
    *((uint32_t*)data_array) = packet_count;

    // Populates status bit
    data_array[4] = filter_enable ? 0 : 1;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE1:
            timer_count++;
            ble_ecg_uptime_set(&m_nus, m_conn_handle, timer_count);

            status = 255 - ((timer_count >> 5) % 256);
            ble_ecg_status_set(&m_nus, m_conn_handle, status);
            if (ecg_control & 0x1) {
              switch ((ecg_control >> 1) & 0x3) {
                case 0:
                  if (call_count == 0) {
                    get_ramp(data_array_ptr, kNumSamples);
                    update = true;
                  } else {
                    update = false;
                  }
                  break;
                case 1:
                  update = get_ads_data(data_array_ptr, kNumSamples);
                  break;
                case 2:
                  if (call_count == 0) {
                    get_mock_normal_adc(data_array_ptr, kNumSamples);
                    update = true;
                  } else {
                    update = false;
                  }
                  break;
                case 3:
                  if (call_count == 0) {
                    get_constant(data_array_ptr, kNumSamples, 1);
                    update = true;
                  } else {
                    update = false;
                  }
                  break;
                default:
                  break;
              }
              if (update){
                ble_nus_data_send(&m_nus, data_array, &data_length, m_conn_handle);
                ++packet_count;
              }
            }

            if (((timer_count % 50) == 0) && ((ecg_control >> 7) & 0x1)) {
              NRF_LOG_DEBUG("Arrhythmia!");
              ble_arrhythmia.timestamp = timer_count;
              ble_arrhythmia.arrythmia = 3;
              NRF_LOG_DEBUG("Size: %d", sizeof(ble_arrhythmia));
              ble_ecg_arrhythmia_send(&m_nus, &ble_arrhythmia, &ble_arrhythmia_size, m_conn_handle);
            }
            // TODO(andy) Double check this number.
            call_count++;
            call_count %= 6;
            break;
        default:
            //Do nothing.
            break;
    }

}



/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS.");
        NRF_LOG_DEBUG("Data Size: %d", p_evt->params.rx_data.length);
        if( p_evt->params.rx_data.length != 4)
          return;

        ecg_control = *((uint32_t*)p_evt->params.rx_data.p_data);
        if (ecg_control & (1<<8)) {
          NRF_LOG_DEBUG("Entering DFU Mode!");
          NRF_LOG_FLUSH();
          sd_power_gpregret_clr(0, 0xff);                                 
          sd_power_gpregret_set(0, 0xB1);
          NVIC_SystemReset();
        }
//        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        ble_ecg_control_set(&m_nus, m_conn_handle, ecg_control);

        
      
        // Updates LEDs
        update_led( (ecg_control >> 3) & 0xf);

        NRF_LOG_DEBUG("ecg_control: %d", ecg_control);

    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init, ecg_control, version, sizeof(version)-1);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    NRF_LOG_DEBUG("Connection Event");
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
 //   err_code = bsp_btn_ble_sleep_mode_prepare();
 //   APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
//    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.config.ble_adv_slow_enabled  = false;
    init.config.ble_adv_slow_interval = APP_ADV_INTERVAL_SLOW;
    init.config.ble_adv_slow_timeout  = APP_ADV_DURATION_SLOW;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    bool erase_bonds;
    bool status;
    int32_t adc_value;
    int32_t filter_out;
    int32_t ecg_data;
    uint32_t time_ms = 20; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    int data_count;
    int32_t data_buf[30];
    //uint8_t data_buf[80];
    uint16_t data_length = 120;
    LPFilter filter;
    ads_pins_t ads_pins;
    nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);  /**< SPI instance. */

    // HP IIR Filter Definition
    float hp_num[3] = {0.99835013, -1.99670027,  0.99835013};
    float hp_den[2] = {-1.99692777,  0.99693255};
    int num_d[2] = {0, 0};
    int den_d[2] = {0, 0};
    int hp_out;
    
    // Initialize.
    sd_power_gpregret_clr(0, 0xff); 
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

   //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
    err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
    NRF_LOG_INFO("Timer Configured!");

    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL1, time_ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_LED);

    // Start execution.
    NRF_LOG_INFO("Argos Proto Dev FW");
    NRF_LOG_FLUSH();

    LPFilter_init(&filter);

    // Sets up the ADS chip config
    ads_pins.miso = ADS_MISO_PIN;
    ads_pins.mosi = ADS_MOSI_PIN;
    ads_pins.sck = ADS_SCK_PIN;
    ads_pins.csn = ADS_CSN_PIN;
    ads_pins.rst = ADS_RST_PIN;
    ads_pins.drdy = ADS_DRDY_PIN;
    ads_pins.start = ADS_START_PIN;
    ads_pins.clksel = ADS_CLKSEL_PIN;

    status = AdsInit(spi, ads_pins);
    if( !status) {
      NRF_LOG_WARNING("Failed to init ADS!");
      NRF_LOG_FLUSH();
      while(1);
    }
  
    NRF_LOG_INFO("DEVICEID0: %08X", NRF_FICR->DEVICEID[0]);
    NRF_LOG_INFO("DEVICEID1: %08X", NRF_FICR->DEVICEID[1]);

    advertising_start();
    //while (nrf_gpio_pin_read(ADS_DRDY_PIN) == 0)
    //  ;
    nrf_drv_gpiote_in_event_enable(ADS_DRDY_PIN, true);
    // Enter main loop.
    data_count = 0;
    time_ticks = 0;
    for (;;) {
      NRF_LOG_FLUSH();
      while(1){
        __WFE();
        if (AdsNewData()) {
          break;
        }
      };

      /*
      for( data_count = 0; data_count < 30; data_count++) {
        data_buf[data_count] = AdsGetData();
      }
      ble_nus_data_send(&m_nus, (uint8_t*)data_buf, &data_length, m_conn_handle);
  */
      /*
      while (new_data == false);
      new_data = false;
*/
      adc_value = AdsGetData();

      // Passes through the HP filter
      hp_out = hp_num[0] * adc_value + 
               hp_num[1] * num_d[0] + 
               hp_num[2] * num_d[1] -
               hp_den[0] * den_d[0] -
               hp_den[1] * den_d[1];
      num_d[1] = num_d[0];
      num_d[0] = adc_value;
      den_d[1] = den_d[0];
      den_d[0] = hp_out;

      LPFilter_put(&filter, hp_out);
      ecg_data = LPFilter_get(&filter);

      if (ecg_control & 0x200) {
        filter_enable = false;
      } else {
        filter_enable = true;
      }
      
      if (ecg_control & 0x1) {
        if (filter_enable) {
          if(CIRC_GBUF_PUSH(ADSBUFFER,&ecg_data)){
           NRF_LOG_ERROR("ADS Buffer Overflow!");
           NRF_LOG_FLUSH();
        }
        } else {
          if(CIRC_GBUF_PUSH(ADSBUFFER,&adc_value)){
            NRF_LOG_ERROR("ADS Buffer Overflow!");
           NRF_LOG_FLUSH();
        }
        }
      } 
        
//    ads_overflow = true;
      /*
      data_buf[data_count] = ecg_data ;
      //data_buf[2*data_count+1] = (ecg_data & 0xff00) >> 8;
      data_count++;
      if(data_count == 30) {
        ble_nus_data_send(&m_nus, (uint8_t*)data_buf, &data_length, m_conn_handle);
        data_count = 0;
        //NRF_LOG_INFO("%d", ecg_data);
 //       NRF_LOG_INFO("%d", time_ticks++);
 //       NRF_LOG_FLUSH();
      }
      //NRF_LOG_FLUSH();
*/
      
    }
}
//ble_nus_data_send(&m_nus, data_array, &data_length, m_conn_handle);
/**
 * @}
 */
