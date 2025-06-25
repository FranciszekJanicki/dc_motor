#include "dc_motor.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>

static dc_motor_err_t dc_motor_device_initialize(dc_motor_t const* motor)
{
    return motor->interface.device_initialize
               ? motor->interface.device_initialize(motor->interface.device_user)
               : DC_MOTOR_ERR_NULL;
}

static dc_motor_err_t dc_motor_device_deinitialize(dc_motor_t const* motor)
{
    return motor->interface.device_deinitialize
               ? motor->interface.device_deinitialize(motor->interface.device_user)
               : DC_MOTOR_ERR_NULL;
}

static dc_motor_err_t dc_motor_device_set_voltage(dc_motor_t const* motor, float32_t voltage)
{
    return motor->interface.device_set_voltage
               ? motor->interface.device_set_voltage(motor->interface.device_user, voltage)
               : DC_MOTOR_ERR_NULL;
}

static dc_motor_err_t dc_motor_device_set_direction(dc_motor_t const* motor,
                                                    dc_motor_direction_t direction)
{
    return motor->interface.device_set_direction
               ? motor->interface.device_set_direction(motor->interface.device_user, direction)
               : DC_MOTOR_ERR_NULL;
}

static dc_motor_err_t dc_motor_set_direction(dc_motor_t* motor, dc_motor_direction_t direction)
{
    if (direction == motor->state.direction) {
        return DC_MOTOR_ERR_OK;
    }

    motor->state.direction = direction;

    return dc_motor_device_set_direction(motor, direction);
}

static dc_motor_err_t dc_motor_set_voltage(dc_motor_t* motor, float32_t voltage)
{
    if (voltage == motor->state.voltage) {
        return DC_MOTOR_ERR_OK;
    }

    motor->state.voltage = voltage;

    return dc_motor_device_set_voltage(motor, voltage);
}

static inline float32_t dc_motor_clamp_position(dc_motor_t const* motor, float32_t position)
{
    if (position < motor->config.min_position) {
        position = motor->config.min_position;
    } else if (position > motor->config.max_position) {
        position = motor->config.max_position;
    }

    return position;
}

static inline float32_t dc_motor_clamp_speed(dc_motor_t const* motor, float32_t speed)
{
    if (speed != 0.0F) {
        if (fabsf(speed) < motor->config.min_speed) {
            speed = copysignf(motor->config.min_speed, speed);
        } else if (fabsf(speed) > motor->config.max_speed) {
            speed = copysignf(motor->config.max_speed, speed);
        }
    }

    return speed;
}

static inline float32_t dc_motor_clamp_acceleration(dc_motor_t const* motor, float32_t acceleration)
{
    if (acceleration != 0.0F) {
        if (fabsf(acceleration) < motor->config.min_acceleration) {
            acceleration = copysignf(motor->config.min_acceleration, acceleration);
        } else if (fabsf(acceleration) > motor->config.max_acceleration) {
            acceleration = copysignf(motor->config.max_acceleration, acceleration);
        }
    }

    return acceleration;
}

static inline dc_motor_direction_t dc_motor_speed_to_direction(dc_motor_t const* motor,
                                                               float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed /*||
        fabsf(speed) < (motor->config.dc_change / delta_time)*/) {
        return DC_MOTOR_DIRECTION_STOP;
    }

    return speed > 0.0F ? DC_MOTOR_DIRECTION_FORWARD : DC_MOTOR_DIRECTION_BACKWARD;
}

static inline uint32_t dc_motor_speed_to_voltage(dc_motor_t const* motor, float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed /*||
        fabsf(speed) < (motor->config.dc_change / delta_time)*/) {
        return 0U;
    }

    return (uint32_t)fabsf(speed / motor->config.dc_change);
}

static inline float32_t dc_motor_wrap_position(float32_t position)
{
    position = fmodf(position, 360.0F);

    while (position < 0.0F) {
        position += 360.0F;
    }
    if (position >= 360.0F) {
        position -= 360.0F;
    }

    return position;
}

static inline int64_t dc_motor_position_to_dc_count(dc_motor_t const* motor, float32_t position)
{
    assert(motor->config.dc_change > 0.0F);

    float32_t dc_count = dc_motor_wrap_position(position) / motor->config.dc_change;

    return (int64_t)roundf(dc_count);
}

dc_motor_err_t dc_motor_initialize(dc_motor_t* motor,
                                   dc_motor_config_t const* config,
                                   dc_motor_interface_t const* interface)
{
    assert(motor && config && interface);

    memset(motor, 0, sizeof(*motor));
    memcpy(&motor->config, config, sizeof(*config));
    memcpy(&motor->interface, interface, sizeof(*interface));

    motor->state.voltage = 0.0F;
    motor->state.direction = DC_MOTOR_DIRECTION_STOP;

    return dc_motor_device_initialize(motor);
}

dc_motor_err_t dc_motor_deinitialize(dc_motor_t* motor)
{
    assert(motor);

    dc_motor_err_t err = dc_motor_device_deinitialize(motor);

    memset(motor, 0, sizeof(*motor));

    return err;
}

dc_motor_err_t dc_motor_reset(dc_motor_t* motor)
{
    assert(motor);

    motor->state.voltage = 0UL;
    motor->state.dc_count = 0L;
    motor->state.direction = DC_MOTOR_DIRECTION_STOP;

    return dc_motor_set_direction(motor, motor->state.direction);
}

void dc_motor_update_dc_count(dc_motor_t* motor)
{
    assert(motor);

    if (motor->state.direction == DC_MOTOR_DIRECTION_BACKWARD &&
        motor->state.dc_count != LLONG_MIN) {
        motor->state.dc_count--;
    } else if (motor->state.direction == DC_MOTOR_DIRECTION_FORWARD &&
               motor->state.dc_count != LLONG_MAX) {
        motor->state.dc_count++;
    }
}

dc_motor_err_t dc_motor_set_position(dc_motor_t* motor, float32_t position, float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    position = dc_motor_clamp_position(motor, position);

    float32_t current_position = dc_motor_get_position(motor);
    float32_t speed = (position - current_position) / delta_time;

    return dc_motor_set_speed(motor, speed);
}

dc_motor_err_t dc_motor_set_speed(dc_motor_t* motor, float32_t speed)
{
    assert(motor);

    dc_motor_direction_t direction = dc_motor_speed_to_direction(motor, speed);

    dc_motor_err_t err = dc_motor_set_direction(motor, direction);
    if (err != DC_MOTOR_ERR_OK || direction == DC_MOTOR_DIRECTION_STOP) {
        return err;
    }

    speed = dc_motor_clamp_speed(motor, speed);
    float32_t voltage = dc_motor_speed_to_voltage(motor, speed);

    return dc_motor_set_voltage(motor, voltage);
}

dc_motor_err_t dc_motor_set_acceleration(dc_motor_t* motor,
                                         float32_t acceleration,
                                         float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    acceleration = dc_motor_clamp_acceleration(motor, acceleration);

    float32_t current_acceleration = dc_motor_get_acceleration(motor, delta_time);
    float32_t speed = (acceleration + current_acceleration) * delta_time / 2.0F;

    return dc_motor_set_speed(motor, speed);
}

float32_t dc_motor_get_position(dc_motor_t* motor)
{
    assert(motor);

    float32_t position = dc_motor_dc_count_to_position(motor, motor->state.dc_count);

    return position;
}

float32_t dc_motor_get_speed(dc_motor_t* motor, float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    float32_t position = dc_motor_get_position(motor);
    float32_t speed = (position - motor->state.prev_position) / delta_time;

    motor->state.prev_position = position;

    return speed;
}

float32_t dc_motor_get_acceleration(dc_motor_t* motor, float32_t delta_time)
{
    assert(motor && delta_time > 0.0F);

    float32_t speed = dc_motor_get_speed(motor, delta_time);
    float32_t acceleration = (speed - motor->state.prev_speed) / delta_time;

    motor->state.prev_speed = speed;

    return acceleration;
}