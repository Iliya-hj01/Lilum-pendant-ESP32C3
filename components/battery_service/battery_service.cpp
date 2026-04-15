#include "battery_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <math.h>

static const char *TAG = "BAT";

// ── I2C (shared bus, already initialised by IMU service) ───
#define BAT_I2C_NUM         I2C_NUM_0

// ── IP5306 I2C address & registers ─────────────────────────
#define IP5306_ADDR         0x75U

#define IP5306_REG_SYS_CTL0 0x00U
#define IP5306_REG_SYS_CTL1 0x01U
#define IP5306_REG_SYS_CTL2 0x02U
#define IP5306_REG_READ0    0x70U   // bit 3 = charging
#define IP5306_REG_READ1    0x71U   // bit 3 = charge full
#define IP5306_REG_READ2    0x72U   // bit 5 = light load, bit 0 = low-power shutdown
#define IP5306_REG_READ3    0x78U   // battery level (4 discrete steps)
#define IP5306_REG_KEY      0x77U   // bit 0 = short press, bit 1 = long press, bit 2 = double-click
#define IP5306_REG_CHG_CTL0 0x20U
#define IP5306_REG_CHG_CTL1 0x21U
#define IP5306_REG_CHG_DIG  0x24U   // bits 0-4 = VIN max current

// Bit masks – REG_READ0
#define CHARGE_ENABLE_BIT   0x08U

// Bit masks – REG_READ1
#define CHARGE_FULL_BIT     0x08U

// Bit masks – REG_KEY (0x77)
#define KEY_SHORT_PRESS_BIT  0x01U
#define KEY_LONG_PRESS_BIT   0x02U
#define KEY_DOUBLE_CLICK_BIT 0x04U

// Bit masks – REG_SYS_CTL0
#define SYS_CTL0_BIT5      (1U << 5)  // boost mode enable
#define SYS_CTL0_BIT2      (1U << 1)  // boost output enable
#define SYS_CTL0_BIT3      (1U << 2)  // power on load
#define CHARGER_EN_BIT      0x10U      // bit 4 = charger on/off

// Bit masks – REG_SYS_CTL1
#define SYS_CTL1_BIT8      (1U << 7)  // boost control signal
#define SYS_CTL1_BIT5      (1U << 5)  // short press boost switch

// Charging current: I_mA = (value × 100) + 50
// 0.25 A = 250 mA → value = (250 − 50) / 100 = 2
#define CHG_CURRENT_BITS    0x02U
#define CHG_CURRENT_MASK    0x1FU   // bits 0-4

// ── NTC temperature sensing ────────────────────────────────
// NTC wired: 3.3 V ── 60.4 kΩ ──┬── GPIO0 (ADC) ── NTC ── GND
//                                │
#define NTC_PULLUP_OHM      60400.0f
#define NTC_VCC              3.3f

// NTC B-parameter model: R0=10 kΩ @ T0=25 °C, B=3435 K
#define NTC_R0              10000.0f
#define NTC_T0_K            298.15f      // 25 °C
#define NTC_BETA            3435.0f

// Operating-temperature limits (for status display)
#define TEMP_MIN_C           0.0f
#define TEMP_MAX_C          45.0f

// ── Charging temperature thresholds (easily modifiable) ────
#define CHG_TEMP_MIN_C       0.0f    // disable charging below this
#define CHG_TEMP_MAX_C      33.0f    // disable charging above this

// ADC channel for GPIO0 on ESP32-C3 → ADC1 channel 0
#define NTC_ADC_CHANNEL     ADC1_CHANNEL_0
#define NTC_ADC_ATTEN       ADC_ATTEN_DB_6   // ~0–1300 mV range
#define NTC_ADC_WIDTH       ADC_WIDTH_BIT_12

// ── Periodic task ──────────────────────────────────────────
#define BAT_POLL_MS         1000     // 1-second poll interval
#define BAT_TASK_STACK      3072

// ── Shared state ───────────────────────────────────────────
static volatile int8_t  s_bat_level     = -1;
static volatile bool    s_charging      = false;
static volatile bool    s_charge_full   = false;
static volatile float   s_temperature   = NAN;
static volatile bool    s_temp_ok       = false;
static volatile bool    s_key_short     = false;   // latched, cleared on read
static volatile bool    s_key_long      = false;
static volatile bool    s_key_double    = false;
static volatile bool    s_chg_inhibited = false;    // true when charging disabled due to temp
static volatile battery_charge_status_t s_charge_status = BAT_CHG_NOT_CHARGING;

// ── ADC handle ─────────────────────────────────────────────
static esp_adc_cal_characteristics_t s_adc_chars;

// ── I2C helpers ────────────────────────────────────────────

static esp_err_t ip5306_read_reg(uint8_t reg, uint8_t *out)
{
    return i2c_master_write_read_device(BAT_I2C_NUM, IP5306_ADDR,
                                        &reg, 1, out, 1,
                                        pdMS_TO_TICKS(50));
}

static esp_err_t ip5306_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(BAT_I2C_NUM, IP5306_ADDR,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(50));
}

static esp_err_t ip5306_updateBits(uint8_t reg, uint8_t mask, bool high)
{
    uint8_t data;
    esp_err_t err = ip5306_read_reg(reg, &data);
    if (err != ESP_OK) return err;
    if (high)
        data |= mask;
    else
        data &= ~mask;
    return ip5306_write_reg(reg, data);
}

// ── IP5306 read helpers ────────────────────────────────────

static bool ip5306_probe(void)
{
    uint8_t data;
    esp_err_t err = ip5306_read_reg(IP5306_REG_SYS_CTL0, &data);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "IP5306 found (SYS_CTL0 = 0x%02X)", data);
        return true;
    }
    ESP_LOGE(TAG, "IP5306 not found on I2C (err %d)", err);
    return false;
}

static int8_t ip5306_battery_level(void)
{
    uint8_t data;
    if (ip5306_read_reg(IP5306_REG_READ3, &data) != ESP_OK) return -1;
    switch (data & 0xF0) {
        case 0x00: return 100;
        case 0x80: return  75;
        case 0xC0: return  50;
        case 0xE0: return  25;
        default:   return   0;
    }
}

static bool ip5306_is_charging(void)
{
    uint8_t data;
    if (ip5306_read_reg(IP5306_REG_READ0, &data) != ESP_OK) return false;
    return (data & CHARGE_ENABLE_BIT) != 0;
}

static bool ip5306_is_full(void)
{
    uint8_t data;
    if (ip5306_read_reg(IP5306_REG_READ1, &data) != ESP_OK) return false;
    return (data & CHARGE_FULL_BIT) != 0;
}

// ── NTC temperature ────────────────────────────────────────

#define NTC_NUM_SAMPLES     32
#define NTC_SAMPLE_DELAY_US 100

static float ntc_read_temperature(void)
{
    // Multi-sample with filtering: discard raw == 0 reads (common on ESP32-C3
    // when the ADC hasn't settled or there's momentary bus contention).
    uint32_t sum = 0;
    int      valid = 0;

    for (int i = 0; i < NTC_NUM_SAMPLES; i++) {
        int raw = adc1_get_raw(NTC_ADC_CHANNEL);
        if (raw > 0) {          // 0 and negative are invalid/saturated-low
            sum += (uint32_t)raw;
            valid++;
        }
        ets_delay_us(NTC_SAMPLE_DELAY_US);
    }

    if (valid == 0) return NAN;

    int avg_raw = (int)(sum / valid);

    // Convert averaged raw ADC to millivolts using calibration
    uint32_t mv = esp_adc_cal_raw_to_voltage(avg_raw, &s_adc_chars);

    float v_adc = (float)mv / 1000.0f;

    // Voltage divider: V_adc = VCC × R_ntc / (R_ntc + R_pullup)
    // → R_ntc = R_pullup × V_adc / (VCC − V_adc)
    if (v_adc >= NTC_VCC) return NAN;  // open-circuit guard
    float r_ntc = NTC_PULLUP_OHM * v_adc / (NTC_VCC - v_adc);

    if (r_ntc <= 0.0f) return NAN;

    // B-parameter equation: 1/T = 1/T0 + (1/B) × ln(R/R0)
    float temp_k = 1.0f / (1.0f / NTC_T0_K + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0));
    return temp_k - 273.15f;
}

// ── Charge current enforcement ─────────────────────────────

static void ip5306_set_charge_current(void)
{
    uint8_t data;
    if (ip5306_read_reg(IP5306_REG_CHG_DIG, &data) == ESP_OK) {
        uint8_t cur = data & CHG_CURRENT_MASK;
        if (cur != CHG_CURRENT_BITS) {
            data = (data & ~CHG_CURRENT_MASK) | CHG_CURRENT_BITS;
            ip5306_write_reg(IP5306_REG_CHG_DIG, data);
            ESP_LOGW(TAG, "Charge current was 0x%02X, corrected to 0.25A", cur);
        }
    }
}

// ── IP5306 initial configuration ───────────────────────────

static void ip5306_configure(void)
{
    // SYS_CTL0
    ip5306_updateBits(IP5306_REG_SYS_CTL0, SYS_CTL0_BIT5, true);  // enable boost mode
    ip5306_updateBits(IP5306_REG_SYS_CTL0, SYS_CTL0_BIT2, true);  // enable boost output
    ip5306_updateBits(IP5306_REG_SYS_CTL0, SYS_CTL0_BIT3, false); // disable power on load

    // SYS_CTL1
    ip5306_updateBits(IP5306_REG_SYS_CTL1, SYS_CTL1_BIT8, false); // disable boost control signal
    ip5306_updateBits(IP5306_REG_SYS_CTL1, SYS_CTL1_BIT5, true);  // enable short press boost

    // Set charging current to 0.25 A
    ip5306_set_charge_current();
}

// ── ADC initialisation ─────────────────────────────────────

static bool adc_init(void)
{
    // Configure ADC1 width
    adc1_config_width(NTC_ADC_WIDTH);

    // Configure channel attenuation
    adc1_config_channel_atten(NTC_ADC_CHANNEL, NTC_ADC_ATTEN);

    // Characterise ADC for voltage conversion
    esp_adc_cal_characterize(ADC_UNIT_1, NTC_ADC_ATTEN, NTC_ADC_WIDTH,
                             1100, &s_adc_chars);

    ESP_LOGI(TAG, "ADC1 CH%d initialised (GPIO0, atten=%d)", NTC_ADC_CHANNEL, NTC_ADC_ATTEN);
    return true;
}

// ── Periodic monitoring task ───────────────────────────────

static void battery_task(void *arg)
{
    (void)arg;

    TickType_t xLastWake = xTaskGetTickCount();

    while (true) {
        // ── IP5306 status ──
        s_bat_level   = ip5306_battery_level();
        s_charging    = ip5306_is_charging();
        s_charge_full = ip5306_is_full();

        // ── Enforce 0.25 A charging current ──
        ip5306_set_charge_current();

        // ── Button-press detection via register 0x77 ──
        uint8_t key = 0;
        if (ip5306_read_reg(IP5306_REG_KEY, &key) == ESP_OK && (key & 0x07)) {
            if (key & KEY_SHORT_PRESS_BIT) {
                s_key_short = true;
                ESP_LOGI(TAG, "Power button SHORT press detected");
            }
            if (key & KEY_LONG_PRESS_BIT) {
                s_key_long = true;
                ESP_LOGI(TAG, "Power button LONG press detected");
            }
            if (key & KEY_DOUBLE_CLICK_BIT) {
                s_key_double = true;
                ESP_LOGI(TAG, "Power button DOUBLE-CLICK detected");
            }
            // Clear the event flags by writing the bits back
            ip5306_write_reg(IP5306_REG_KEY, key & 0x07);
        }

        // ── NTC temperature ──
        float t = ntc_read_temperature();
        s_temperature = t;
        s_temp_ok = (!isnan(t)) && (t >= TEMP_MIN_C) && (t <= TEMP_MAX_C);

        // ── Temperature-based charging control ──
        if (!isnan(t)) {
            bool out_of_range = (t < CHG_TEMP_MIN_C) || (t > CHG_TEMP_MAX_C);
            if (out_of_range && !s_chg_inhibited) {
                // Disable charger
                ip5306_updateBits(IP5306_REG_SYS_CTL0, CHARGER_EN_BIT, false);
                s_chg_inhibited = true;
                ESP_LOGW(TAG, "Charging DISABLED — temp %.1f°C outside [%.0f, %.0f]",
                         t, CHG_TEMP_MIN_C, CHG_TEMP_MAX_C);
            } else if (!out_of_range && s_chg_inhibited) {
                // Re-enable charger
                ip5306_updateBits(IP5306_REG_SYS_CTL0, CHARGER_EN_BIT, true);
                s_chg_inhibited = false;
                ESP_LOGI(TAG, "Charging RE-ENABLED — temp %.1f°C back in range", t);
            }
        }

        // ── Update overall charge status ──
        if (s_chg_inhibited) {
            s_charge_status = BAT_CHG_TEMP_INHIBITED;
        } else if (!s_temp_ok && !s_charging) {
            // Not charging AND temperature out of allowed range
            s_charge_status = BAT_CHG_TEMP_CRITICAL;
        } else if (s_charge_full) {
            s_charge_status = BAT_CHG_COMPLETE;
        } else if (s_charging) {
            s_charge_status = BAT_CHG_CHARGING_OK;
        } else {
            s_charge_status = BAT_CHG_NOT_CHARGING;
        }

        // ── Status log ──
        ESP_LOGI(TAG, "Batt: %d%% | %s%s | Temp: %.1f°C %s",
                 (int)s_bat_level,
                 s_charge_full ? "FULL" : (s_charging ? "CHARGING" : "DISCHARGING"),
                 s_chg_inhibited ? " (CHG OFF-TEMP)" : "",
                 s_temperature,
                 s_temp_ok ? "(OK)" : "(OUT OF RANGE)");

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(BAT_POLL_MS));
    }
}

// ── Public API ─────────────────────────────────────────────

void battery_service_init(void)
{
    // ADC for NTC
    if (!adc_init()) {
        ESP_LOGE(TAG, "NTC ADC init failed — temperature readings unavailable");
    }

    // IP5306 probe & configure
    if (!ip5306_probe()) {
        ESP_LOGE(TAG, "IP5306 not found — battery readings unavailable");
    } else {
        ip5306_configure();
    }

    xTaskCreate(battery_task, "bat_task", BAT_TASK_STACK, NULL, 1, NULL);
    ESP_LOGI(TAG, "Battery service started");
}

int8_t battery_get_level(void)
{
    return s_bat_level;
}

bool battery_is_charging(void)
{
    return s_charging;
}

bool battery_is_full(void)
{
    return s_charge_full;
}

float battery_get_temperature(void)
{
    return s_temperature;
}

bool battery_temp_in_range(void)
{
    return s_temp_ok;
}

battery_charge_status_t battery_get_charge_status(void)
{
    return s_charge_status;
}

bool battery_key_short_press(void)
{
    bool k = s_key_short;
    if (k) s_key_short = false;
    return k;
}

bool battery_key_long_press(void)
{
    bool k = s_key_long;
    if (k) s_key_long = false;
    return k;
}

bool battery_key_double_click(void)
{
    bool k = s_key_double;
    if (k) s_key_double = false;
    return k;
}
