#include "mic_service.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "soc/i2s_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "MIC";

// ── Hardware pins (SPH0645LM4H-1-8, SEL=HIGH → right channel) ──
#define MIC_I2S_NUM     I2S_NUM_0
#define MIC_BCLK_PIN    1
#define MIC_WS_PIN      3
#define MIC_DATA_PIN    4

// ── Sampling parameters ──
#define MIC_SAMPLE_RATE     44100
#define MIC_DMA_BUF_COUNT   8
#define MIC_DMA_BUF_LEN     64      // frames per DMA buffer

// ── Analysis ──
#define ANALYSIS_SAMPLES    512     // ~11.6 ms at 44100 Hz
#define VOLUME_SMOOTH       0.3f
#define BEAT_RATIO          1.6f
#define BEAT_COOLDOWN_MS    200
#define LONG_AVG_ALPHA      0.002f

// ── Shared state (accessed from other tasks) ──
static volatile uint8_t  s_volume  = 0;
static volatile bool     s_beat    = false;

// ── Internal state ──
static float s_volume_smooth   = 0.0f;
static float s_energy_long_avg = 0.0f;
static bool  s_energy_primed   = false;
static uint32_t s_last_beat_ms = 0;

// ── BPM estimation ──
#define BPM_HISTORY  8          // number of beat intervals to average
static uint32_t s_beat_times[BPM_HISTORY];
static uint8_t  s_beat_idx   = 0;
static uint8_t  s_beat_count = 0;   // how many entries filled
static volatile uint16_t s_bpm = 0;

// ────────────────────────────────────────────────────────────
//  Audio processing helpers
// ────────────────────────────────────────────────────────────

// SPH0645: 18 useful bits inside a 32-bit I2S frame, MSB-aligned.
// Stereo capture (L,R interleaved); we extract R channel (SEL=HIGH).
static inline int32_t extract_sample(int32_t raw) {
    return raw >> 14;
}

static void process_samples(int32_t *buf, size_t count) {
    if (count == 0) return;

    // ── 1. Peak-to-peak amplitude (volume) ──
    int32_t vmin = INT32_MAX, vmax = INT32_MIN;
    float   energy = 0.0f;

    for (size_t i = 0; i < count; i++) {
        int32_t s = extract_sample(buf[i]);
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
        energy += (float)s * (float)s;
    }

    float pp = (float)(vmax - vmin);
    // Normalise peak-to-peak to 0-255.  18-bit signed range is ±131072 so
    // full-scale pp ≈ 262144.  In practice music rarely exceeds ~50 % FS.
    float norm = pp / 131072.0f;        // 0 .. ~2.0
    if (norm > 1.0f) norm = 1.0f;
    float vol = norm * 255.0f;

    // EMA smoothing
    s_volume_smooth = s_volume_smooth * (1.0f - VOLUME_SMOOTH) + vol * VOLUME_SMOOTH;
    s_volume = (uint8_t)(s_volume_smooth + 0.5f);

    // ── 2. Beat detection (energy ratio) ──
    energy /= (float)count;             // mean squared energy this window

    if (!s_energy_primed) {
        // Seed the long-term average with the first window
        s_energy_long_avg = energy;
        s_energy_primed   = true;
    } else {
        // Compare short-term to long-term
        uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        bool cooldown_ok = (now - s_last_beat_ms) >= BEAT_COOLDOWN_MS;

        if (cooldown_ok && s_energy_long_avg > 0.0f &&
            (energy / s_energy_long_avg) > BEAT_RATIO) {
            s_beat = true;
            s_last_beat_ms = now;

            // ── BPM: store timestamp and recompute ──
            s_beat_times[s_beat_idx] = now;
            s_beat_idx = (s_beat_idx + 1) % BPM_HISTORY;
            if (s_beat_count < BPM_HISTORY) s_beat_count++;

            if (s_beat_count >= 2) {
                // Average interval over stored beats
                uint8_t oldest = (s_beat_idx + BPM_HISTORY - s_beat_count) % BPM_HISTORY;
                uint8_t newest = (s_beat_idx + BPM_HISTORY - 1) % BPM_HISTORY;
                uint32_t span = s_beat_times[newest] - s_beat_times[oldest];
                uint16_t intervals = s_beat_count - 1;
                if (span > 0 && intervals > 0) {
                    uint32_t avg_ms = span / intervals;
                    s_bpm = (uint16_t)(60000 / avg_ms);
                }
            }
        }

        // Update long-term average (slow EMA)
        s_energy_long_avg = s_energy_long_avg * (1.0f - LONG_AVG_ALPHA) +
                            energy * LONG_AVG_ALPHA;
    }
}

// ────────────────────────────────────────────────────────────
//  FreeRTOS task
// ────────────────────────────────────────────────────────────

static void mic_task(void *arg) {
    const size_t stereo_samples = ANALYSIS_SAMPLES * 2;
    const size_t buf_bytes = stereo_samples * sizeof(int32_t);
    int32_t *buf = (int32_t *)malloc(buf_bytes);
    int32_t *mono = (int32_t *)malloc(ANALYSIS_SAMPLES * sizeof(int32_t));
    if (!buf || !mono) {
        ESP_LOGE(TAG, "Failed to allocate mic read buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Mic task running  (SR=%d, buf=%d stereo frames)",
             MIC_SAMPLE_RATE, ANALYSIS_SAMPLES);

    while (1) {
        size_t bytes_read = 0;
        esp_err_t err = i2s_read(MIC_I2S_NUM, buf, buf_bytes, &bytes_read, portMAX_DELAY);
        if (err == ESP_OK && bytes_read > 0) {
            size_t total_words = bytes_read / sizeof(int32_t);
            size_t mono_count = 0;
            for (size_t i = 1; i < total_words && mono_count < ANALYSIS_SAMPLES; i += 2) {
                mono[mono_count++] = buf[i];
            }

            process_samples(mono, mono_count);
        }
    }
}

// ────────────────────────────────────────────────────────────
//  Public API
// ────────────────────────────────────────────────────────────

void mic_service_init(void) {
    // ── Disconnect 32K crystal oscillator from GPIO0/1 pads ──
    CLEAR_PERI_REG_MASK(RTC_CNTL_EXT_XTL_CONF_REG,
        RTC_CNTL_XPD_XTAL_32K | RTC_CNTL_XTAL32K_XPD_FORCE |
        RTC_CNTL_ENCKINIT_XTAL_32K | RTC_CNTL_DBUF_XTAL_32K);

    // ── I2S driver config (TX+RX master) ──
    // ESP32-C3 quirk: RX-only mode generates clocks internally but does NOT
    // output them to GPIO via the signal matrix.  Adding TX mode makes the
    // TX clock generator drive I2SO_BCK_OUT to the physical pin.
    i2s_config_t i2s_cfg = {};
    i2s_cfg.mode              = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    i2s_cfg.sample_rate       = MIC_SAMPLE_RATE;
    i2s_cfg.bits_per_sample   = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_cfg.bits_per_chan     = I2S_BITS_PER_CHAN_32BIT;
    i2s_cfg.channel_format    = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.dma_buf_count     = MIC_DMA_BUF_COUNT;
    i2s_cfg.dma_buf_len       = MIC_DMA_BUF_LEN;
    i2s_cfg.use_apll          = false;
    i2s_cfg.intr_alloc_flags  = ESP_INTR_FLAG_LEVEL1;

    esp_err_t err = i2s_driver_install(MIC_I2S_NUM, &i2s_cfg, 0, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    // ── Pin mapping ──
    i2s_pin_config_t pin_cfg = {};
    pin_cfg.mck_io_num   = I2S_PIN_NO_CHANGE;
    pin_cfg.bck_io_num   = MIC_BCLK_PIN;
    pin_cfg.ws_io_num    = MIC_WS_PIN;
    pin_cfg.data_out_num = I2S_PIN_NO_CHANGE;
    pin_cfg.data_in_num  = MIC_DATA_PIN;

    err = i2s_set_pin(MIC_I2S_NUM, &pin_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_set_pin failed: %s", esp_err_to_name(err));
        return;
    }

    i2s_zero_dma_buffer(MIC_I2S_NUM);

    // ── FIX: tx_stop_en disables BCK/WS output when TX FIFO is empty. ──
    // Since we only do RX, the TX FIFO is always empty, so clear it.
    // sig_loopback stays enabled so RX sees the TX-generated clocks.
    I2S0.tx_conf.tx_stop_en = 0;
    I2S0.tx_conf.tx_update = 1;
    while (I2S0.tx_conf.tx_update);

    ESP_LOGI(TAG, "I2S initialised  (BCLK=GPIO%d, WS=GPIO%d, DIN=GPIO%d, SR=%d)",
             MIC_BCLK_PIN, MIC_WS_PIN, MIC_DATA_PIN, MIC_SAMPLE_RATE);

    xTaskCreate(mic_task, "mic_task", 4096, NULL, 3, NULL);
}

uint8_t mic_get_volume(void) {
    return s_volume;
}

bool mic_get_beat(void) {
    bool b = s_beat;
    if (b) s_beat = false;   // auto-clear after read
    return b;
}

uint16_t mic_get_bpm(void) {
    return s_bpm;
}
