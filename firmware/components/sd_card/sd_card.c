#include "sd_card.h"

#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"

#define TAG "sd_card"

static sdmmc_card_t *card = NULL;

bool sdcard_mount(uint8_t sd_pin_clk, uint8_t sd_pin_cmd, uint8_t sd_pin_d0, uint8_t sd_pin_d1, uint8_t sd_pin_d2, uint8_t sd_pin_d3, uint8_t sd_pin_cd) {
  if (card != NULL) {
    return true;
  }

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 2,
    .allocation_unit_size = 16 * 1024
  };

  gpio_config_t carddetect_conf = {
    .pin_bit_mask = 1ULL << sd_pin_cd,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };

  ESP_ERROR_CHECK(gpio_config(&carddetect_conf));

  // The card-detect switch pulls the pin low when a card is inserted. Only
  // warn: if the polarity is ever wrong, the mount below still decides.
  if (gpio_get_level(sd_pin_cd) == 1) {
    ESP_LOGW(TAG, "Card-detect says no SD card, trying to mount anyway");
  }

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  slot_config.width = 4;
  slot_config.clk = sd_pin_clk;
  slot_config.cmd = sd_pin_cmd;
  slot_config.d0 = sd_pin_d0;
  slot_config.d1 = sd_pin_d1;
  slot_config.d2 = sd_pin_d2;
  slot_config.d3 = sd_pin_d3;

  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  esp_err_t ret = esp_vfs_fat_sdmmc_mount(SDCARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
    card = NULL;
    return false;
  }

  sdmmc_card_print_info(stdout, card);
  return true;
}

FILE* sdcard_new_file(char *path, size_t path_len) {
  if (card == NULL) {
    return NULL;
  }

  char name[32];
  struct stat st;

  for (int i = 1; i <= 9999; i++) {
    snprintf(name, sizeof(name), SDCARD_MOUNT_POINT "/EEG_%04d.BIN", i);
    if (stat(name, &st) == 0) {
      continue;  // already exists
    }

    FILE *f = fopen(name, "wb");
    if (f == NULL) {
      ESP_LOGE(TAG, "Could not create %s", name);
      return NULL;
    }

    ESP_LOGI(TAG, "Recording to %s", name);
    if (path != NULL && path_len > 0) {
      snprintf(path, path_len, "%s", name);
    }
    return f;
  }

  ESP_LOGE(TAG, "No free file name left on the card");
  return NULL;
}

void sdcard_sync(FILE *f) {
  if (f == NULL) {
    return;
  }
  fflush(f);
  fsync(fileno(f));
}

void sdcard_unmount(void) {
  if (card == NULL) {
    return;
  }
  esp_vfs_fat_sdcard_unmount(SDCARD_MOUNT_POINT, card);
  card = NULL;
  ESP_LOGI(TAG, "SD card unmounted");
}
