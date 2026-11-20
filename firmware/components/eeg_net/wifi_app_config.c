#include "wifi_app.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_http_server.h"

#include "config.h"
#include "index.html.h"

#define TAG "wifi_http"

#define MIN(a,b) ((a < b) ? a : b)

static httpd_handle_t http_server = NULL;

static esp_err_t http_root_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *) webpage_html, webpage_html_len);
}

/* static esp_err_t http_status_handler(httpd_req_t *req) { */
/*   char content[] = ""; */
/*   return httpd_resp_send(req,  */
/* } */

config_samplerate_t parse_sample_rate(const char *value) {
    if (strcmp(value, "250") == 0) return CONFIG_SAMPLERATE_250;
    if (strcmp(value, "500") == 0) return CONFIG_SAMPLERATE_500;
    // Higher ADS1299 rates exist, but UDP streaming (30 samples per packet,
    // one packet per loop) and the 100 Hz FreeRTOS tick are not designed for
    // them, so they are clamped to 1000 SPS.
    if (atoi(value) >= 1000) return CONFIG_SAMPLERATE_1000;

    return CONFIG_SAMPLERATE_250;
}

config_gain_t parse_channel_gain(const char *value) {
    if (strcmp(value, "1") == 0) return CONFIG_GAIN_1;
    if (strcmp(value, "2") == 0) return CONFIG_GAIN_2;
    if (strcmp(value, "4") == 0) return CONFIG_GAIN_4;
    if (strcmp(value, "6") == 0) return CONFIG_GAIN_6;
    if (strcmp(value, "8") == 0) return CONFIG_GAIN_8;
    if (strcmp(value, "12") == 0) return CONFIG_GAIN_12;
    if (strcmp(value, "24") == 0) return CONFIG_GAIN_24;

    return CONFIG_GAIN_24;
}

config_channel_mode_t parse_channel_mode(const char *value) {
    if (strcmp(value, "-1") == 0) return CONFIG_CHANNEL_PWRDWN;
    if (strcmp(value, "0") == 0) return CONFIG_CHANNEL_NORMAL;
    if (strcmp(value, "1") == 0) return CONFIG_CHANNEL_SHORTED;
    if (strcmp(value, "2") == 0) return CONFIG_CHANNEL_BIASMEAS;
    if (strcmp(value, "3") == 0) return CONFIG_CHANNEL_MVDD;
    if (strcmp(value, "4") == 0) return CONFIG_CHANNEL_TEMP;
    if (strcmp(value, "5") == 0) return CONFIG_CHANNEL_TEST;
    if (strcmp(value, "6") == 0) return CONFIG_CHANNEL_BIASDRP;
    if (strcmp(value, "7") == 0) return CONFIG_CHANNEL_BIASDRN;

    return CONFIG_CHANNEL_PWRDWN;
}

/*
 * GET /config -> current configuration as JSON, using the same values the
 * web form uses, so the page can show what is actually stored on the device.
 */
static esp_err_t http_get_config_handler(httpd_req_t *req) {
    static const int gain_val[] = {1, 2, 4, 6, 8, 12, 24};
    const config_t *cfg = get_config();

    char json[512];
    int n = snprintf(json, sizeof(json),
                     "{\"sample_rate\":%d,\"comm_ref\":%s,\"lead_off\":%s,\"channels\":[",
                     250 << cfg->sample_rate,
                     cfg->common_reference ? "true" : "false",
                     cfg->lead_off ? "true" : "false");

    for (uint8_t i = 0; i < FREEEEG_NUM_CHANNELS; i++) {
        // Web form mode values are the config enum shifted by one (PWRDWN = -1).
        n += snprintf(json + n, sizeof(json) - n,
                      "%s{\"mode\":%d,\"gain\":%d}",
                      i ? "," : "",
                      (int) cfg->channel_mode[i] - 1,
                      gain_val[cfg->gain[i]]);
    }
    snprintf(json + n, sizeof(json) - n, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t http_update_handler(httpd_req_t *req) {
    char content[512];

    int recv_len = httpd_req_recv(
        req,
        content,
        MIN(req->content_len, sizeof(content) - 1)
    );

    if (recv_len <= 0) {
        return ESP_FAIL;
    }

    content[recv_len] = '\0';

    ESP_LOGI(TAG, "POST content: %s\n", content);

    const uint8_t BUFF_LEN = 16;
    char buff[BUFF_LEN];

    config_t config = {0};

    for (uint8_t i = 0; i < 8; ++i) {
      char mode[10];
      char gain[10];

      sprintf(mode, "ch%d_mode", i+1);
      if (httpd_query_key_value(content, mode, buff, BUFF_LEN) == ESP_OK) {
        config.channel_mode[i] = parse_channel_mode(buff);
      } else goto fail;
      
      sprintf(gain, "ch%d_gain", i+1);
      if (httpd_query_key_value(content, gain, buff, BUFF_LEN) == ESP_OK) {
        config.gain[i] = parse_channel_gain(buff);
      } else goto fail;
    }

    if (httpd_query_key_value(content, "sample_rate", buff, BUFF_LEN) == ESP_OK) {
      config.sample_rate = parse_sample_rate(buff);
    } else goto fail;

    
    if (httpd_query_key_value(content, "comm_ref", buff, BUFF_LEN) == ESP_OK)
      config.common_reference = true;
    else
      config.common_reference = false;

    config.lead_off = httpd_query_key_value(content, "lead_off", buff, BUFF_LEN) == ESP_OK;
    
    // set_config() also notifies the listener, which reprograms the ADS1299.
    set_config(&config);
    // Persist it so it survives a reboot.
    store_config();

    httpd_resp_sendstr(req, "Configuration saved and applied. <a href=\"/\">Back</a>");

    return ESP_OK;

 fail:
    ESP_LOGE(TAG, "Malfromed configuration request"); 
    httpd_resp_sendstr(req, "Error updating configuration");
    return ESP_FAIL;
}

/*
 * POST /time, body = milliseconds since the Unix epoch (the page sends
 * Date.now() when it loads). The board has no RTC, so this is how recordings
 * get real timestamps. The HTTP server only runs while not measuring, so the
 * clock never jumps in the middle of a recording.
 */
static esp_err_t http_time_handler(httpd_req_t *req) {
    char content[32];
    int recv_len = httpd_req_recv(req, content, MIN(req->content_len, sizeof(content) - 1));
    if (recv_len <= 0) {
        return ESP_FAIL;
    }
    content[recv_len] = '\0';

    long long ms = atoll(content);
    if (ms < 1000000000000LL) {  // before 2001: not a real wall-clock time
        httpd_resp_sendstr(req, "Bad time");
        return ESP_OK;
    }

    struct timeval tv = {
        .tv_sec = ms / 1000,
        .tv_usec = (ms % 1000) * 1000,
    };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "Clock set from web page: %lld ms", ms);

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

void wifi_http_start(void) {
  if (http_server != NULL) {
    return;
  }

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  ESP_ERROR_CHECK(httpd_start(&http_server, &config));

  httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = http_root_handler,
    .user_ctx = NULL,
  };

  httpd_uri_t update_uri = {
    .uri = "/update_config",
    .method = HTTP_POST,
    .handler = http_update_handler,
    .user_ctx = NULL,
  };
  
  httpd_uri_t get_config_uri = {
    .uri = "/config",
    .method = HTTP_GET,
    .handler = http_get_config_handler,
    .user_ctx = NULL,
  };

  ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &root_uri));
  ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &update_uri));
  ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &get_config_uri));

  httpd_uri_t time_uri = {
    .uri = "/time",
    .method = HTTP_POST,
    .handler = http_time_handler,
    .user_ctx = NULL,
  };
  ESP_ERROR_CHECK(httpd_register_uri_handler(http_server, &time_uri));

  ESP_LOGI(TAG, "HTTP server initialized");
}

void wifi_http_stop(void) {
  if (http_server != NULL) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(httpd_stop(http_server));
    http_server = NULL;
  }
}
