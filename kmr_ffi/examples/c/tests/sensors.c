/* Controller of tests/sensors.rs. */
#include "kmr.h"

/* The sensors as the kmr_* functions give them; mirrored as `SensorsOut` in tests/sensors.rs. */
typedef struct {
    uint64_t imu_age;
    uint64_t imu_dropped;
    uint64_t mic_dropped;
    uint64_t mic_len;
    kmr_axes accel;
    kmr_axes gyro;
    uint32_t has_imu;
    uint32_t has_imu_past_slots;
    uint32_t has_accel;
    uint32_t has_gyro;
    uint32_t has_mic;
    uint32_t mic_rate_hz;
    int16_t samples[4];
} sensors_out;

/* Copies IMU 0 and microphone 0 through the API; ctx points at a reference to a sensors_out. */
void copy_sensors(const kmr_time *time, kmr_state *state, const kmr_sensors *sensors, void *ctx) {
    sensors_out *out = *(sensors_out **)ctx;
    const kmr_imu_reading *imu = kmr_imu(sensors, 0);
    const kmr_mic_reading *mic = kmr_mic(sensors, 0);
    size_t k, len = 0;
    out->has_imu = imu != NULL;
    out->has_imu_past_slots = kmr_imu(sensors, 1) != NULL || kmr_imu(sensors, KMR_MAX_IMUS) != NULL;
    out->has_accel = 0;
    out->has_gyro = 0;
    out->accel.x = out->accel.y = out->accel.z = 0.0f;
    out->gyro.x = out->gyro.y = out->gyro.z = 0.0f;
    if (imu != NULL) {
        out->has_accel = kmr_imu_accel(imu, &out->accel);
        out->has_gyro = kmr_imu_gyro(imu, &out->gyro);
        out->imu_age = kmr_imu_age(imu);
        out->imu_dropped = kmr_imu_dropped(imu);
    }
    out->has_mic = mic != NULL;
    out->mic_len = 0;
    for (k = 0; k < 4; ++k) {
        out->samples[k] = 0;
    }
    if (mic != NULL) {
        const int16_t *samples = kmr_mic_samples(mic, &len);
        out->mic_len = len;
        for (k = 0; k < len && k < 4; ++k) {
            out->samples[k] = samples[k];
        }
        out->mic_rate_hz = kmr_mic_rate_hz(mic);
        out->mic_dropped = kmr_mic_dropped(mic);
    }
}
