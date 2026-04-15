#include "IMU_service.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "IMU";

// ── I2C configuration ──────────────────────────────────────
#define IMU_I2C_NUM         I2C_NUM_0
#define IMU_SDA_PIN         GPIO_NUM_6
#define IMU_SCL_PIN         GPIO_NUM_7
#define IMU_INT1_PIN        GPIO_NUM_5
#define IMU_I2C_FREQ_HZ     400000      // 400 kHz Fast-mode

// LSM6DS3TR-C 7-bit I2C addresses
// SA0/SDO=GND → 0x6A,  SA0/SDO=VCC → 0x6B
#define LSM6DS3TR_C_ADDR_L  0x6AU
#define LSM6DS3TR_C_ADDR_H  0x6BU

// Resolved at runtime by imu_probe()
static uint8_t s_imu_addr = 0;

// ── Key registers ──────────────────────────────────────────
#define REG_WHO_AM_I        0x0FU
#define REG_CTRL1_XL        0x10U   // Accelerometer ODR & FS
#define REG_CTRL2_G         0x11U   // Gyroscope ODR & FS
#define REG_CTRL3_C         0x12U   // BDU, IF_INC, SW_RESET
#define REG_STATUS_REG      0x1EU
#define REG_OUTX_L_XL       0x28U   // Accel output (6 bytes)
#define REG_INT1_CTRL       0x0DU

#define WHO_AM_I_VALUE      0x6AU

// ── Accelerometer sensitivity at ±2 g: 0.061 mg/LSB ───────
#define ACCEL_SENSITIVITY   0.061f

// ── XY axis rotation (IMU → PCB frame) ─────────────────────
// The IMU's X/Y axes are rotated by this angle relative to the
// PCB axes.  Z is already aligned.
#define IMU_XY_ROTATION_DEG  48.0f
static const float s_cos_theta = cosf(IMU_XY_ROTATION_DEG * (float)M_PI / 180.0f);
static const float s_sin_theta = sinf(IMU_XY_ROTATION_DEG * (float)M_PI / 180.0f);

// ── Sampling / smoothing ───────────────────────────────────
#define IMU_POLL_MS         10      // 100 Hz polling
#define ACCEL_SMOOTH_ALPHA  0.2f

// ── Shared state ───────────────────────────────────────────
static volatile float s_accel_x_mg = 0.0f;
static volatile float s_accel_y_mg = 0.0f;
static volatile float s_accel_z_mg = 0.0f;

// ── Gesture: double-dip detection ──────────────────────────
// Tune these values to adjust gesture sensitivity.
#define GESTURE_FACE_DOWN_Z_MG       (1000.0f)  // expected Z when face-down
#define GESTURE_Z_TOLERANCE_MG        200.0f     // max |Z − expected| to be "face-down"
#define GESTURE_DIP_MIN_INTENSITY_MG  300.0f     // Z must spike this much above rest
#define GESTURE_DIP_MAX_DURATION_MS   300        // max ms a single dip may last
#define GESTURE_BETWEEN_DIPS_MIN_MS   80         // min gap between dips (reject bounce)
#define GESTURE_BETWEEN_DIPS_MAX_MS   500        // max gap between dips
#define GESTURE_COOLDOWN_MS           1000       // lockout after detection

typedef enum {
    GS_IDLE,        // not face-down
    GS_READY,       // face-down, waiting for first dip
    GS_DIP1,        // first dip in progress
    GS_BETWEEN,     // gap between dips
    GS_DIP2,        // second dip in progress
    GS_COOLDOWN     // gesture fired, cooling down
} gesture_state_t;

static gesture_state_t s_gs        = GS_IDLE;
static TickType_t      s_gs_ts     = 0;
static volatile bool   s_double_dip = false; // set on detection, cleared on read
static volatile bool   s_gesture_enabled = false; // runtime enable/disable

static const char *gs_names[] = {
    "IDLE", "READY", "DIP1", "BETWEEN", "DIP2", "COOLDOWN"
};

// ── I2C helpers ────────────────────────────────────────────

static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(IMU_I2C_NUM, s_imu_addr,
                                      buf, sizeof(buf),
                                      pdMS_TO_TICKS(50));
}

static esp_err_t imu_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(IMU_I2C_NUM, s_imu_addr,
                                        &reg, 1, data, len,
                                        pdMS_TO_TICKS(50));
}

static bool imu_probe(void)
{
    const uint8_t addrs[] = { LSM6DS3TR_C_ADDR_L, LSM6DS3TR_C_ADDR_H };
    for (size_t i = 0; i < sizeof(addrs); i++) {
        uint8_t who = 0;
        uint8_t reg = REG_WHO_AM_I;
        esp_err_t err = i2c_master_write_read_device(
            IMU_I2C_NUM, addrs[i], &reg, 1, &who, 1, pdMS_TO_TICKS(50));
        ESP_LOGI(TAG, "Probing 0x%02X → err=%d, WHO_AM_I=0x%02X", addrs[i], err, who);
        if (err == ESP_OK && who == WHO_AM_I_VALUE) {
            s_imu_addr = addrs[i];
            ESP_LOGI(TAG, "LSM6DS3TR-C found at 0x%02X", s_imu_addr);
            return true;
        }
    }
    ESP_LOGE(TAG, "LSM6DS3TR-C not found on either address!");
    return false;
}

// ── Sensor initialisation ──────────────────────────────────

static bool imu_hw_init(void)
{
    if (!imu_probe()) {
        return false;
    }

    // Software reset
    imu_write_reg(REG_CTRL3_C, 0x01);     // SW_RESET
    vTaskDelay(pdMS_TO_TICKS(20));

    // CTRL3_C: BDU=1, IF_INC=1 (auto-increment on multi-byte read)
    imu_write_reg(REG_CTRL3_C, 0x44);

    // CTRL1_XL: ODR=104 Hz (0x40), FS=±2 g (0x00) → 0x40
    imu_write_reg(REG_CTRL1_XL, 0x40);

    // CTRL2_G : keep gyro off for now (power save)
    imu_write_reg(REG_CTRL2_G, 0x00);

    // INT1_CTRL: route accel data-ready to INT1 (bit 0)
    imu_write_reg(REG_INT1_CTRL, 0x01);

    ESP_LOGI(TAG, "IMU configured: ±2 g, 104 Hz, INT1→DRDY_XL");
    return true;
}

// ── Gesture state machine (called once per sample) ─────────

static void gesture_process(float z_raw_mg)
{
    TickType_t now = xTaskGetTickCount();
    float dip_threshold = GESTURE_FACE_DOWN_Z_MG + GESTURE_DIP_MIN_INTENSITY_MG;
    bool face_down = fabsf(z_raw_mg - GESTURE_FACE_DOWN_Z_MG) < GESTURE_Z_TOLERANCE_MG;
    bool in_dip    = (z_raw_mg > dip_threshold);

    switch (s_gs) {
    case GS_IDLE:
        if (face_down) {
            s_gs = GS_READY;
        }
        break;

    case GS_READY:
        if (!face_down && !in_dip) {
            s_gs = GS_IDLE;
        } else if (in_dip) {
            s_gs    = GS_DIP1;
            s_gs_ts = now;
        }
        break;

    case GS_DIP1:
        if ((now - s_gs_ts) > pdMS_TO_TICKS(GESTURE_DIP_MAX_DURATION_MS)) {
            s_gs = face_down ? GS_READY : GS_IDLE;
        } else if (!in_dip) {
            s_gs    = GS_BETWEEN;
            s_gs_ts = now;
        }
        break;

    case GS_BETWEEN: {
        uint32_t elapsed = (now - s_gs_ts);
        if (elapsed > pdMS_TO_TICKS(GESTURE_BETWEEN_DIPS_MAX_MS)) {
            s_gs = face_down ? GS_READY : GS_IDLE;
        } else if (in_dip && elapsed >= pdMS_TO_TICKS(GESTURE_BETWEEN_DIPS_MIN_MS)) {
            s_gs    = GS_DIP2;
            s_gs_ts = now;
        }
        break;
    }

    case GS_DIP2:
        if ((now - s_gs_ts) > pdMS_TO_TICKS(GESTURE_DIP_MAX_DURATION_MS)) {
            s_gs = face_down ? GS_READY : GS_IDLE;
        } else if (!in_dip) {
            // ── Double-dip detected! ──
            s_double_dip = true;
            ESP_LOGI(TAG, ">>> DOUBLE DIP DETECTED <<<");
            s_gs    = GS_COOLDOWN;
            s_gs_ts = now;
        }
        break;

    case GS_COOLDOWN:
        if ((now - s_gs_ts) > pdMS_TO_TICKS(GESTURE_COOLDOWN_MS)) {
            s_gs = face_down ? GS_READY : GS_IDLE;
        }
        break;
    }
}

// ── FreeRTOS task ──────────────────────────────────────────

static void imu_task(void *arg)
{
    (void)arg;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    bool first = true;

    TickType_t xLastWake = xTaskGetTickCount();
    while (true) {
        uint8_t raw[6];
        esp_err_t err = imu_read_reg(REG_OUTX_L_XL, raw, 6);
        if (err == ESP_OK) {
            int16_t ax = (int16_t)(raw[0] | ((int16_t)raw[1] << 8));
            int16_t ay = (int16_t)(raw[2] | ((int16_t)raw[3] << 8));
            int16_t az = (int16_t)(raw[4] | ((int16_t)raw[5] << 8));

            float raw_x = ax * ACCEL_SENSITIVITY;
            float raw_y = ay * ACCEL_SENSITIVITY;
            float az_mg = az * ACCEL_SENSITIVITY;

            // Gesture detection on raw (unsmoothed) Z
            if (s_gesture_enabled) gesture_process(az_mg);

            // Rotate X/Y from IMU frame to PCB frame
            float ax_mg = raw_x * s_cos_theta - raw_y * s_sin_theta;
            float ay_mg = raw_x * s_sin_theta + raw_y * s_cos_theta;

            if (first) {
                sx = ax_mg; sy = ay_mg; sz = az_mg;
                first = false;
            } else {
                sx = sx * (1.0f - ACCEL_SMOOTH_ALPHA) + ax_mg * ACCEL_SMOOTH_ALPHA;
                sy = sy * (1.0f - ACCEL_SMOOTH_ALPHA) + ay_mg * ACCEL_SMOOTH_ALPHA;
                sz = sz * (1.0f - ACCEL_SMOOTH_ALPHA) + az_mg * ACCEL_SMOOTH_ALPHA;
            }
            s_accel_x_mg = sx;
            s_accel_y_mg = sy;
            s_accel_z_mg = sz;
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(IMU_POLL_MS));
    }
}

// ── Public API ─────────────────────────────────────────────

void imu_service_init(void)
{
    // ── I2C master bus ──
    i2c_config_t conf = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = IMU_SDA_PIN;
    conf.scl_io_num       = IMU_SCL_PIN;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = IMU_I2C_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(IMU_I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(IMU_I2C_NUM, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C%d initialised (SDA=GPIO%d, SCL=GPIO%d, %d Hz)",
             IMU_I2C_NUM, IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQ_HZ);

    // ── INT1 pin (input, not using interrupt yet) ──
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << IMU_INT1_PIN;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);

    if (!imu_hw_init()) {
        ESP_LOGE(TAG, "IMU initialisation failed — task not started");
        return;
    }

    xTaskCreate(imu_task, "imu_task", 3072, NULL, 2, NULL);
}

void imu_get_accel_mg(float *ax, float *ay, float *az)
{
    if (ax) *ax = s_accel_x_mg;
    if (ay) *ay = s_accel_y_mg;
    if (az) *az = s_accel_z_mg;
}

bool imu_gesture_double_dip(void)
{
    bool d = s_double_dip;
    if (d) s_double_dip = false;
    return d;
}

const char *imu_gesture_state_str(void)
{
    return gs_names[s_gs];
}

void imu_gesture_set_enabled(bool enable)
{
    s_gesture_enabled = enable;
    if (!enable) {
        s_gs = GS_IDLE;
        s_double_dip = false;
    }
}

bool imu_gesture_is_enabled(void)
{
    return s_gesture_enabled;
}
