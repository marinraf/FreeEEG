#include "tasks.h"

#include "esp_log.h"

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/semphr.h"

#include "config.h"

#include "ads1299.h"
#include "wifi_app.h"
#include "ws2812.h"
#include "icm42670.h"
#include "ics43434.h"
#include "sd_card.h"

#include "config_util.h"
#include "pin_config.h"

#include "driver/gpio.h"

#define TAG "tasks"

#define EVENT_BIT_OFFLINE_MEAS  (1<<0)
#define EVENT_BIT_ONLINE_MODE	(1<<1)
#define EVENT_BIT_ONLINE_MEAS	(1<<2)
#define EVENT_BIT_IDLE		(1<<3)
#define EVENT_BIT_AFE_READY	(1<<4)
#define EVENT_BIT_IMU_READY	(1<<5)

// Records are flushed to the card at least this often, so a flat battery
// loses at most a few seconds of data.
#define SD_SYNC_PERIOD_MS	5000

static EventGroupHandle_t event_group;
static RingbufHandle_t buff_afe;
static RingbufHandle_t buff_imu;
static struct timeval time_afe;
static struct timeval time_imu;

static TaskHandle_t imu_task_handle = NULL;
static void IRAM_ATTR imu_isr(void) {
    BaseType_t hp_task_woken = pdFALSE;

    if (imu_task_handle != NULL) {
      gettimeofday(&time_imu, NULL);
      vTaskNotifyGiveFromISR(imu_task_handle, &hp_task_woken);
    }

    if (hp_task_woken) {
        portYIELD_FROM_ISR();
    }
}

static TaskHandle_t afe_task_handle = NULL;
static void IRAM_ATTR afe_isr(void) {
    BaseType_t hp_task_woken = pdFALSE;

    if (afe_task_handle != NULL) {
      gettimeofday(&time_afe, NULL);
      vTaskNotifyGiveFromISR(afe_task_handle, &hp_task_woken);
    }

    if (hp_task_woken) {
        portYIELD_FROM_ISR();
    }
}

/*
 * Status LED (dimmed to save battery):
 *   red      booting, or SD card error in offline mode
 *   green    online, idle (waiting for "start")
 *   cyan     online, streaming, all electrodes in contact
 *   yellow   streaming/recording with an electrode off (lead-off enabled)
 *   blue     offline recording: 2 s when it starts, then a short flash every
 *            5 s (yellow instead if an electrode is off), dark otherwise
 */
static const ws2812_color_t LED_OFF    = {0x00, 0x00, 0x00};
static const ws2812_color_t LED_RED    = {0x40, 0x00, 0x00};
static const ws2812_color_t LED_GREEN  = {0x00, 0x40, 0x00};
static const ws2812_color_t LED_CYAN   = {0x00, 0x30, 0x30};
static const ws2812_color_t LED_YELLOW = {0x40, 0x30, 0x00};
static const ws2812_color_t LED_BLUE   = {0x00, 0x00, 0x40};

static SemaphoreHandle_t led_mutex;
static const ws2812_color_t *led_current = NULL;
static volatile TickType_t led_flash_until = 0;

static void led_set(const ws2812_color_t *color) {
  xSemaphoreTake(led_mutex, portMAX_DELAY);
  if (color != led_current) {
    ws2812_set(color);
    led_current = color;
  }
  xSemaphoreGive(led_mutex);
}

// True while some active electrode (or the reference) reports lead-off.
static volatile bool contact_bad = false;

/*
 * Online/offline mode handling.
 *
 * The switch ISR only wakes task_mode, which debounces the switch and does the
 * transition. mode_mutex serialises everything that starts/stops the AFE or
 * changes mode (task_mode and the UDP "start"/"stop" commands in task_recv).
 * sd_mutex protects sd_file, written by task_write and closed by task_mode.
 */
static TaskHandle_t mode_task_handle = NULL;
static SemaphoreHandle_t mode_mutex;
static SemaphoreHandle_t sd_mutex;
static FILE *sd_file = NULL;
static bool sd_error = false;
static bool offline_active = false;

static void IRAM_ATTR switch_isr(void *args) {
  BaseType_t hp_task_woken = pdFALSE;

  if (mode_task_handle != NULL) {
    vTaskNotifyGiveFromISR(mode_task_handle, &hp_task_woken);
  }

  if (hp_task_woken) {
    portYIELD_FROM_ISR();
  }
}

void tasks_init() {
  event_group = xEventGroupCreate();
  xEventGroupSetBits(event_group, EVENT_BIT_ONLINE_MODE | EVENT_BIT_IDLE);
  mode_mutex = xSemaphoreCreateMutex();
  sd_mutex = xSemaphoreCreateMutex();
  led_mutex = xSemaphoreCreateMutex();
  
  gpio_config_t switch_conf = {
    .pin_bit_mask = 1ULL << SWITCH_PIN,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    // Any edge wakes task_mode, which debounces and reads the final level.
    .intr_type = GPIO_INTR_ANYEDGE,
  };

  ESP_ERROR_CHECK(gpio_config(&switch_conf));

  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  ESP_ERROR_CHECK(gpio_isr_handler_add(
				       SWITCH_PIN,
				       switch_isr,
				       NULL
				       )
		  );

  wifi_ap_init();
  wifi_ap_start();
  wifi_http_start();
  wifi_udp_start();

  config_init();
  
  ws2812_init(WS2812_PIN);
  led_set(&LED_RED);

  ics43434_init(ICS43434_PIN_WS, ICS43434_PIN_CLK, ICS43434_PIN_DATA);

  buff_afe = xRingbufferCreateNoSplit(sizeof(ads1299_data_t), 500);
  buff_imu = xRingbufferCreateNoSplit(sizeof(icm42670_data_t), 500);
}


static int32_t offset_coeffs[NUM_ELECTRODES] = {0};
static bool calibration = false;

static void apply_config_to_afe(const config_t *config) {
  ads1299_config_t ads_config = config_to_ads1299(config);
  ads1299_send_config(&ads_config);
}

void task_afe(void *args) {
  afe_task_handle = xTaskGetCurrentTaskHandle();

  ads1299_data_t data;
  
  ads1299_init(ADS1299_PIN_MOSI, ADS1299_PIN_MISO, ADS1299_PIN_SCK, ADS1299_PIN_CS, ADS1299_PIN_DRDY, ADS1299_PIN_START, afe_isr);

  //////////////////////CALIBRATION///////////////////////////
  if (calibration) { 
    float acc[NUM_ELECTRODES] = {0};

    memset(acc, 0, sizeof(acc));
  
    ads1299_config_t cal_config = {
      .sample_rate = ADS1299_CONFIG_SAMPLERATE_1000SPS,
      .comm_ref = false
    };

    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      cal_config.electrodes[i] = ADS1299_CONFIG_MODE_SHORTED;
      cal_config.gains[i] = ADS1299_CONFIG_GAIN_1;
    }

    ads1299_send_config(&cal_config);

    ESP_LOGI(TAG, "Starting calibration...");

    ads1299_start();

    const uint16_t OFFSET_CAL_LEN = 2000;

    for (uint16_t i = 0; i < OFFSET_CAL_LEN; i++) {
      ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

      ads1299_read(&data);

      if ((data.status[0] & 0xF0) != 0xC0) {
	i--;
	continue;
      }
    
      for (uint8_t j = 0; j < NUM_ELECTRODES; j++) {
	acc[j] += (float) data.channel[j] / OFFSET_CAL_LEN;
      }
    }

    ads1299_stop();

    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      offset_coeffs[i] = (int32_t)(acc[i]);
    }

    ESP_LOGI(TAG, "Calibration offsets: %" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32,
      offset_coeffs[0], offset_coeffs[1], offset_coeffs[2], offset_coeffs[3], offset_coeffs[4], offset_coeffs[5], offset_coeffs[6], offset_coeffs[7]);
  }
  ////////////////////////////////////////////////////////////
  const config_t *config = get_config();
  apply_config_to_afe(config);

  // From now on, every configuration saved from the web page is pushed to the
  // ADS1299 immediately. The HTTP server is only running while the AFE is
  // stopped (see task_recv), so this never happens in the middle of a capture.
  config_set_listener(apply_config_to_afe);
  xEventGroupSetBits(event_group, EVENT_BIT_AFE_READY);

  while (1) {
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    ads1299_read(&data);

    // Status word starts with 0b1100; drop frames that are out of sync.
    if ((data.status[0] & 0xF0) != 0xC0) continue;
    
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
      data.channel[i] -= offset_coeffs[i];
    }
    
    data.timestamp = time_afe;

    xRingbufferSend(buff_afe, &data, sizeof(data), 0);
  }
}

void task_imu(void *args) {
  imu_task_handle = xTaskGetCurrentTaskHandle();

  icm42670_data_t data;
  icm42670_init(ICM42670_PIN_SCL, ICM42670_PIN_SDA, ICM42670_PIN_INT, imu_isr);
  xEventGroupSetBits(event_group, EVENT_BIT_IMU_READY);
  while(1) {
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

    icm42670_read_data(&data);
    data.timestamp = time_imu;
    xRingbufferSend(buff_imu, &data, sizeof(data), 0);
  }
}

void task_sound(void *args) {
  uint32_t buff[1024];
  uint8_t msg_buff[100];
  
  while (1) {
    if (wifi_udp_recv_packet(msg_buff)) {
      ESP_LOGI(TAG, "Message received: %s", msg_buff);
      if (strcmp((const char*)msg_buff, "start") == 0) {
	ESP_LOGI(TAG, "Starting recording");
	while (1) {
	  size_t len = ics43434_read(buff, sizeof buff);
	  wifi_udp_send((const uint8_t*)buff, len);
	} 
	vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
  ics43434_stop();
}

/*
 * Fills `packet` with whatever is waiting in the AFE and IMU ring buffers
 * (at most UDP_ELEC_BUFF_SIZE EEG samples and UDP_IMU_BUFF_SIZE IMU samples).
 * Only the channels that are not powered down are copied, packed to the left.
 */
static void build_packet(udp_packet_t *packet, const config_t *config) {
  memset(packet, 0, sizeof(*packet));

  uint8_t count;
  size_t size;
  uint8_t act_channels[NUM_ELECTRODES] = {0};
  uint8_t num_elec = 0;
  for (uint8_t i = 0 ; i < NUM_ELECTRODES; i++) if (config->channel_mode[i] != CONFIG_CHANNEL_PWRDWN) act_channels[i] = num_elec++;
  packet->num_elec = num_elec;

  // Read AFE data from buffer
  count = 0;
  ads1299_data_t* data_afe = NULL;
  data_afe = (ads1299_data_t *) xRingbufferReceive(buff_afe, &size, 0);
  if (data_afe != NULL) {
    packet->abs_time = data_afe->timestamp.tv_sec;
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) if (config->channel_mode[i] != CONFIG_CHANNEL_PWRDWN) packet->elec_data[count][act_channels[i]] = data_afe->channel[i];
    packet->loff_p |= data_afe->loff_p;
    packet->loff_n |= data_afe->loff_n;
    packet->elec_time[count++] = data_afe->timestamp.tv_usec;
    vRingbufferReturnItem(buff_afe, data_afe);
    while (count < UDP_ELEC_BUFF_SIZE) {
      data_afe = (ads1299_data_t *) xRingbufferReceive(buff_afe, &size, 0);
      if (data_afe == NULL) break;
      for (uint8_t i = 0; i < NUM_ELECTRODES; i++) if (config->channel_mode[i] != CONFIG_CHANNEL_PWRDWN) packet->elec_data[count][act_channels[i]] = data_afe->channel[i];
      packet->loff_p |= data_afe->loff_p;
      packet->loff_n |= data_afe->loff_n;
      packet->elec_time[count] = data_afe->timestamp.tv_usec;
      if (data_afe->timestamp.tv_sec > packet->abs_time)
        packet->elec_time[count] += 1000000;
      vRingbufferReturnItem(buff_afe, data_afe);
      count++;
    }
  }
  packet->num_elec_samp = count;

  // Read IMU data from buffer
  count = 0;
  icm42670_data_t *data_imu = NULL;
  while (count < UDP_IMU_BUFF_SIZE) {
    data_imu = (icm42670_data_t *) xRingbufferReceive(buff_imu, &size, 0);
    if (data_imu == NULL) break;
    packet->imu_time[count] = data_imu->timestamp.tv_usec;
    if (data_imu->timestamp.tv_sec > packet->abs_time)
      packet->imu_time[count] += 1000000;
    packet->accel_data[count][0] = data_imu->accel.x; packet->accel_data[count][1] = data_imu->accel.y; packet->accel_data[count][2] = data_imu->accel.z;
    packet->gyro_data[count][0] = data_imu->gyro.x; packet->gyro_data[count][1] = data_imu->gyro.y; packet->gyro_data[count][2] = data_imu->gyro.z;
    vRingbufferReturnItem(buff_imu, data_imu);
    count++;
  }
  packet->num_imu = count;

  // Only flags of the electrodes we asked the ADS1299 to check matter.
  if (config->lead_off) {
    uint8_t normal = 0;
    for (uint8_t i = 0; i < NUM_ELECTRODES; i++) if (config->channel_mode[i] == CONFIG_CHANNEL_NORMAL) normal |= (1 << i);
    packet->loff_p &= normal;
    packet->loff_n &= normal;
    if (packet->num_elec_samp > 0) {
      contact_bad = (packet->loff_p | packet->loff_n) != 0;
    }
  } else {
    packet->loff_p = 0;
    packet->loff_n = 0;
    contact_bad = false;
  }
}

/*
 * SD file layout (little-endian), see Client/FreeEEG/sdfile.py:
 *   header, 24 bytes:
 *     char    magic[4]      "FEEG"
 *     uint8   version       1
 *     uint16  sample_rate   samples per second
 *     uint8   comm_ref      1 if SRB1 common reference was enabled
 *     uint8   mode[8]       config_channel_mode_t per channel (0 = powered down)
 *     uint8   gain[8]       PGA gain per channel (1, 2, 4, 6, 8, 12 or 24)
 *   then records until end of file:
 *     uint16  length
 *     uint8   packet[length]   same bytes as one UDP packet
 */
typedef struct __attribute__((packed)) {
  char magic[4];
  uint8_t version;
  uint16_t sample_rate;
  uint8_t comm_ref;
  uint8_t mode[NUM_ELECTRODES];
  uint8_t gain[NUM_ELECTRODES];
} sd_header_t;

static bool write_sd_header(FILE *f, const config_t *config) {
  static const uint8_t gain_val[] = {1, 2, 4, 6, 8, 12, 24};
  sd_header_t hdr = {
    .magic = {'F', 'E', 'E', 'G'},
    .version = 1,
    .sample_rate = 250 << config->sample_rate,
    .comm_ref = config->common_reference ? 1 : 0,
  };
  for (uint8_t i = 0; i < NUM_ELECTRODES; i++) {
    hdr.mode[i] = config->channel_mode[i];
    hdr.gain[i] = gain_val[config->gain[i]];
  }
  return fwrite(&hdr, sizeof(hdr), 1, f) == 1;
}

/* Drains the ring buffers into sd_file. Called with sd_mutex held. */
static void write_sd_records(const config_t *config) {
  static udp_packet_t packet;
  static uint8_t buffer[UDP_PACKET_MAX_SIZE];

  if (sd_file == NULL || sd_error) {
    return;
  }

  // Keep going while full packets come out, so offline recording is not
  // limited by the 30 ms loop period like UDP streaming is.
  do {
    build_packet(&packet, config);
    if (packet.num_elec_samp == 0 && packet.num_imu == 0) break;

    uint16_t len = udp_packet_serialize(&packet, buffer);
    if (fwrite(&len, sizeof(len), 1, sd_file) != 1 ||
        fwrite(buffer, 1, len, sd_file) != len) {
      ESP_LOGE(TAG, "SD write failed (card removed or full?), recording stopped");
      sd_error = true;
      led_set(&LED_RED);
      return;
    }
  } while (packet.num_elec_samp == UDP_ELEC_BUFF_SIZE);
}

void task_write(void *args) {
  ESP_LOGI(TAG, "sizeof(udp_packet_t) = %u", (unsigned) sizeof(udp_packet_t));

  const config_t *config = get_config();
  TickType_t last_sync = xTaskGetTickCount();

  while(1) {
    EventBits_t bits = xEventGroupWaitBits(event_group, EVENT_BIT_OFFLINE_MEAS | EVENT_BIT_ONLINE_MEAS, pdFALSE, pdFALSE, portMAX_DELAY);
    if ((bits & EVENT_BIT_OFFLINE_MEAS) == EVENT_BIT_OFFLINE_MEAS) {
      xSemaphoreTake(sd_mutex, portMAX_DELAY);
      write_sd_records(config);
      bool synced = false;
      if (sd_file != NULL && !sd_error &&
          xTaskGetTickCount() - last_sync >= pdMS_TO_TICKS(SD_SYNC_PERIOD_MS)) {
        sdcard_sync(sd_file);
        last_sync = xTaskGetTickCount();
        synced = true;
      }
      bool recording = sd_file != NULL && !sd_error;
      xSemaphoreGive(sd_mutex);

      if (recording) {
        TickType_t now = xTaskGetTickCount();
        if (synced) {
          led_set(contact_bad ? &LED_YELLOW : &LED_BLUE);
          led_flash_until = now + pdMS_TO_TICKS(100);
        } else if ((int32_t)(now - led_flash_until) > 0) {
          led_set(&LED_OFF);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(30));
    } else {
      udp_packet_t packet;
      // Keep sending while full packets come out, so streaming does not fall
      // behind when the loop takes longer than 30 ms (it does at 1000 SPS).
      do {
        build_packet(&packet, config);
        wifi_udp_send_packet(&packet);
      } while (packet.num_elec_samp == UDP_ELEC_BUFF_SIZE);
      led_set(contact_bad ? &LED_YELLOW : &LED_CYAN);
      vTaskDelay(pdMS_TO_TICKS(30));
    }
  }
}

void task_recv(void *args) {
  uint8_t buff[257]; // wifi_udp_recv_packet() reads up to 256 bytes + NUL
  
  while (1) {
    xEventGroupWaitBits(event_group, EVENT_BIT_ONLINE_MODE, pdFALSE, pdFALSE, portMAX_DELAY);
    if (wifi_udp_recv_packet(buff)) {
      ESP_LOGI(TAG, "MSG received from udp: \"%s\"", buff);
      xSemaphoreTake(mode_mutex, portMAX_DELAY);
      // The switch may have moved to offline while we were waiting.
      if ((xEventGroupGetBits(event_group) & EVENT_BIT_ONLINE_MODE) == 0) {
	xSemaphoreGive(mode_mutex);
	continue;
      }
      if (strcmp((const char*) buff, "start") == 0) {
	xEventGroupClearBits(event_group, EVENT_BIT_IDLE);
	wifi_http_stop();
	// Clean all the data buffers before task_write starts reading them
	vRingbufferReset(buff_imu);
	vRingbufferReset(buff_afe);	
	xEventGroupSetBits(event_group, EVENT_BIT_ONLINE_MEAS);	
	ads1299_start();
	icm42670_start();
      } else if (strcmp((const char*) buff, "stop") == 0) {
	ads1299_stop();
	icm42670_stop();
	xEventGroupClearBits(event_group, EVENT_BIT_ONLINE_MEAS);
	wifi_http_start();
	xEventGroupSetBits(event_group, EVENT_BIT_IDLE);
	led_set(&LED_GREEN);
      } else if (strncmp((const char*) buff, "time ", 5) == 0) {
	// "time <unix seconds>[.fraction]" sent by the client before "start".
	// Ignored while measuring so timestamps never jump mid-recording.
	double t = atof((const char*) buff + 5);
	if ((xEventGroupGetBits(event_group) & EVENT_BIT_IDLE) && t > 1e9) {
	  struct timeval tv = {
	    .tv_sec = (time_t) t,
	    .tv_usec = (suseconds_t) ((t - (double)(time_t) t) * 1e6),
	  };
	  settimeofday(&tv, NULL);
	  ESP_LOGI(TAG, "Clock set from client: %.3f", t);
	}
      }
      xSemaphoreGive(mode_mutex);
    }
  }
}

/* Waits until the switch has read the same level for 50 ms. */
static int read_switch_debounced(void) {
  int level = gpio_get_level(SWITCH_PIN);
  int stable = 0;
  while (stable < 5) {
    vTaskDelay(pdMS_TO_TICKS(10));
    int now = gpio_get_level(SWITCH_PIN);
    if (now == level) {
      stable++;
    } else {
      level = now;
      stable = 0;
    }
  }
  ulTaskNotifyTake(pdTRUE, 0);  // forget the bounces seen meanwhile
  return level;
}

/* Called with mode_mutex held. */
static void enter_offline(void) {
  ESP_LOGI(TAG, "Switching to OFFLINE mode");

  EventBits_t bits = xEventGroupClearBits(event_group, EVENT_BIT_ONLINE_MODE | EVENT_BIT_ONLINE_MEAS | EVENT_BIT_IDLE);
  if (bits & EVENT_BIT_ONLINE_MEAS) {
    ads1299_stop();
    icm42670_stop();
  }
  // Let task_write finish the iteration it may be in before touching buffers.
  vTaskDelay(pdMS_TO_TICKS(100));

  wifi_http_stop();
  wifi_ap_stop();
  offline_active = true;

  if (!sdcard_mount(SDCARD_PIN_CLK, SDCARD_PIN_CMD, SDCARD_PIN_DAT0, SDCARD_PIN_DAT1, SDCARD_PIN_DAT2, SDCARD_PIN_DAT3, SDCARD_PIN_CD)) {
    ESP_LOGE(TAG, "No usable SD card: not recording");
    led_set(&LED_RED);
    return;
  }

  char path[32];
  FILE *f = sdcard_new_file(path, sizeof(path));
  if (f == NULL || !write_sd_header(f, get_config())) {
    ESP_LOGE(TAG, "Could not create the recording file: not recording");
    if (f != NULL) fclose(f);
    sdcard_unmount();
    led_set(&LED_RED);
    return;
  }

  xSemaphoreTake(sd_mutex, portMAX_DELAY);
  sd_file = f;
  sd_error = false;
  xSemaphoreGive(sd_mutex);

  vRingbufferReset(buff_imu);
  vRingbufferReset(buff_afe);
  xEventGroupSetBits(event_group, EVENT_BIT_OFFLINE_MEAS);
  ads1299_start();
  icm42670_start();

  led_flash_until = xTaskGetTickCount() + pdMS_TO_TICKS(2000);
  led_set(&LED_BLUE);
  ESP_LOGI(TAG, "Recording to %s", path);
}

/* Called with mode_mutex held. */
static void exit_offline(void) {
  ESP_LOGI(TAG, "Switching to ONLINE mode");

  EventBits_t bits = xEventGroupClearBits(event_group, EVENT_BIT_OFFLINE_MEAS);
  if (bits & EVENT_BIT_OFFLINE_MEAS) {
    ads1299_stop();
    icm42670_stop();
  }

  // Waits for task_write to finish its current write, then closes the file.
  xSemaphoreTake(sd_mutex, portMAX_DELAY);
  if (sd_file != NULL) {
    sdcard_sync(sd_file);
    fclose(sd_file);
    sd_file = NULL;
    ESP_LOGI(TAG, "Recording closed");
  }
  xSemaphoreGive(sd_mutex);
  sdcard_unmount();
  offline_active = false;

  wifi_ap_start();
  wifi_http_start();
  xEventGroupSetBits(event_group, EVENT_BIT_ONLINE_MODE | EVENT_BIT_IDLE);
  led_set(&LED_GREEN);
}

void task_mode(void *args) {
  mode_task_handle = xTaskGetCurrentTaskHandle();

  // The AFE and IMU tasks must have initialised their chips before we may
  // start them for an offline recording.
  xEventGroupWaitBits(event_group, EVENT_BIT_AFE_READY | EVENT_BIT_IMU_READY, pdFALSE, pdTRUE, portMAX_DELAY);
  led_set(&LED_GREEN);

  while (1) {
    // Also runs once at boot, so the switch position at power-up is honoured.
    bool want_offline = read_switch_debounced() == SWITCH_LEVEL_OFFLINE;

    if (want_offline != offline_active) {
      xSemaphoreTake(mode_mutex, portMAX_DELAY);
      if (want_offline) {
        enter_offline();
      } else {
        exit_offline();
      }
      xSemaphoreGive(mode_mutex);
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // next switch edge
  }
}
