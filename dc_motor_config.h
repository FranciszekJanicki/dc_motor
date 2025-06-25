#ifndef DC_MOTOR_DC_MOTOR_CONFIG_H
#define DC_MOTOR_DC_MOTOR_CONFIG_H

#include <stdint.h>

#ifndef FLOAT32
#define FLOAT32
typedef float float32_t;
#endif // FLOAT32

typedef enum {
    DC_MOTOR_ERR_OK = 0,
    DC_MOTOR_ERR_FAIL = 1 << 0,
    DC_MOTOR_ERR_NULL = 1 << 1,
} dc_motor_err_t;

typedef enum {
    DC_MOTOR_DIRECTION_FORWARD,
    DC_MOTOR_DIRECTION_BACKWARD,
    DC_MOTOR_DIRECTION_STOP,
} dc_motor_direction_t;

typedef struct {
    float32_t voltage;
    float32_t prev_position;
    float32_t prev_speed;
    dc_motor_direction_t direction;
} dc_motor_state_t;

typedef struct {
    float32_t min_position;
    float32_t max_position;
    float32_t min_speed;
    float32_t max_speed;
    float32_t min_acceleration;
    float32_t max_acceleration;
    float32_t ref_voltage;
} dc_motor_config_t;

typedef struct {
    void* device_user;
    dc_motor_err_t (*device_initialize)(void*);
    dc_motor_err_t (*device_deinitialize)(void*);
    dc_motor_err_t (*device_set_voltage)(void*, float32_t);
    dc_motor_err_t (*device_set_direction)(void*, dc_motor_direction_t);
} dc_motor_interface_t;

#endif // DC_MOTOR_DC_MOTOR_CONFIG_H