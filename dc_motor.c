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

static inline dc_motor_direction_t dc_motor_speed_to_direction(dc_motor_t const* motor,
                                                               float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed) {
        return DC_MOTOR_DIRECTION_STOP;
    }

    return speed > 0.0F ? DC_MOTOR_DIRECTION_FORWARD : DC_MOTOR_DIRECTION_BACKWARD;
}

static inline float32_t dc_motor_speed_to_voltage(dc_motor_t const* motor, float32_t speed)
{
    if (fabsf(speed) < motor->config.min_speed) {
        return 0U;
    }

    return (fabsf(speed) - motor->config.min_speed) * motor->config.ref_voltage /
           (motor->config.max_speed - motor->config.min_speed);
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

    dc_motor_err_t err = dc_motor_set_direction(motor, DC_MOTOR_DIRECTION_STOP);
    err |= dc_motor_set_voltage(motor, 0.0F);

    return err;
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