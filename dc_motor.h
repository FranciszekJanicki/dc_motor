#ifndef DC_MOTOR_DC_MOTOR_H
#define DC_MOTOR_DC_MOTOR_H

#include "dc_motor_config.h"
#include <stdbool.h>

typedef struct {
  dc_motor_config_t config;
  dc_motor_interface_t interface;
  dc_motor_state_t state;
} dc_motor_t;

dc_motor_err_t dc_motor_initialize(dc_motor_t *motor,
                                   dc_motor_config_t const *config,
                                   dc_motor_interface_t const *interface,
                                   float32_t start_position);
dc_motor_err_t dc_motor_deinitialize(dc_motor_t *motor);

dc_motor_err_t dc_motor_reset(dc_motor_t *motor);

void dc_motor_update_dc_count(dc_motor_t *motor);

dc_motor_err_t dc_motor_set_position(dc_motor_t *motor, float32_t position,
                                     float32_t delta_time);
dc_motor_err_t dc_motor_set_speed(dc_motor_t *motor, float32_t speed);
dc_motor_err_t dc_motor_set_acceleration(dc_motor_t *motor,
                                         float32_t acceleration,
                                         float32_t delta_time);

float32_t dc_motor_get_position(dc_motor_t *motor);
float32_t dc_motor_get_speed(dc_motor_t *motor, float32_t delta_time);
float32_t dc_motor_get_acceleration(dc_motor_t *motor, float32_t delta_time);

#endif // DC_MOTOR_DC_MOTOR_H