/* Reference snippets only. Copy into matching USER CODE sections; do not compile this file. */

/* USER CODE BEGIN Includes */
#include "app_eskf.h"
#include "mpu6050_adapter.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static app_eskf_t g_estimator;
static mpu6050_calibration_t g_imu_cal;
static volatile uint8_t g_imu_ready;
static mpu6050_raw_t g_imu_raw;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
app_eskf_init(&g_estimator);
g_imu_cal = mpu6050_default_calibration();
/* Replace defaults using calibration identified in Phase B. */
/* Start sensor DRDY/timer and the first I2C/SPI DMA transfer here. */
/* USER CODE END 2 */

/* DMA callback: capture raw bytes + measurement timestamp, then set g_imu_ready.
   Keep matrix operations outside interrupt context. */

/* USER CODE BEGIN WHILE */
if (g_imu_ready) {
    imu_sample_t imu;
    __disable_irq();
    mpu6050_raw_t local = g_imu_raw;
    g_imu_ready = 0;
    __enable_irq();
    mpu6050_convert(&local, platform_measurement_time_us(), &g_imu_cal, &imu, 0);
    eskf_status_t st = app_eskf_push_imu(&g_estimator, &imu);
    eskf_health_level_t health = eskf_health_check(&g_estimator.health,
        &g_estimator.filter, st, platform_imu_timed_out());
    if (health == ESKF_HEALTH_FAULT) platform_enter_manual_or_safe_mode();
}
/* USER CODE END WHILE */

/* GPS/baro/mag callbacks should enqueue timestamped samples. app_eskf rewinds its
   fixed history and repropagates delayed samples; ESKF_LATE_MEASUREMENT means the
   sample is older than that history and must not be applied to current state. */
