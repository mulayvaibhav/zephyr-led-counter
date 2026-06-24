#include "vehicle_control_manager.h"

#include <string.h>
#include <stdlib.h>

#define DEFAULT_SPEED_LIMIT_PCT       (40u)
#define DEFAULT_TTL_MS                (400u)
#define DEFAULT_RAMP_UP_PCT_PER_SEC   (80u)
#define DEFAULT_RAMP_DOWN_PCT_PER_SEC (160u)
#define DEFAULT_MAX_UPDATE_DT_MS      (100u)

static vehicle_control_manager_t mgr;

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint8_t clamp_u8(uint8_t value, uint8_t min_value, uint8_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static int16_t abs_i16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static bool different_sign_nonzero(int16_t a, int16_t b)
{
    return ((a > 0) && (b < 0)) || ((a < 0) && (b > 0));
}

static int16_t move_toward_i16(int16_t current, int16_t target, uint16_t step)
{
    if (current == target) {
        return current;
    }

    if (step == 0u) {
        step = 1u;
    }

    if (current < target) {
        int32_t next = (int32_t)current + (int32_t)step;
        if (next > target) {
            next = target;
        }
        return (int16_t)next;
    }

    int32_t next = (int32_t)current - (int32_t)step;
    if (next < target) {
        next = target;
    }

    return (int16_t)next;
}

static uint16_t calculate_step(uint16_t rate_pct_per_sec, uint32_t dt_ms)
{
    uint32_t step = ((uint32_t)rate_pct_per_sec * dt_ms) / 1000u;

    if (step == 0u) {
        step = 1u;
    }

    if (step > 100u) {
        step = 100u;
    }

    return (uint16_t)step;
}

static int16_t slew_limit_motor_pct(int16_t current,
                                    int16_t target,
                                    uint16_t step_up,
                                    uint16_t step_down)
{
    int16_t effective_target = target;
    uint16_t step;

    /*
     * Do not jump directly from forward to reverse.
     * First ramp down to zero, then ramp up in the opposite direction.
     */
    if (different_sign_nonzero(current, target)) {
        effective_target = 0;
        step = step_down;
    } else {
        if (abs_i16(target) > abs_i16(current)) {
            step = step_up;
        } else {
            step = step_down;
        }
    }

    return move_toward_i16(current, effective_target, step);
}

static void calculate_differential_targets(vehicle_control_manager_t *mgr)
{
    int32_t left;
    int32_t right;
    int32_t max_abs;
    int32_t speed_limit;

    int16_t linear_x = clamp_i16(mgr->active_cmd.linear_x,
                                 VEHICLE_AXIS_MIN,
                                 VEHICLE_AXIS_MAX);

    int16_t angular_z = clamp_i16(mgr->active_cmd.angular_z,
                                  VEHICLE_AXIS_MIN,
                                  VEHICLE_AXIS_MAX);

    /*
     * Sign convention:
     *
     * angular_z > 0 means turn/rotate left.
     *
     * For pivot-left:
     *   linear_x  = 0
     *   angular_z = +1000
     *
     * We want:
     *   left  = -1000
     *   right = +1000
     *
     * So:
     *   left  = linear_x - angular_z
     *   right = linear_x + angular_z
     */
    left = (int32_t)linear_x - (int32_t)angular_z;
    right = (int32_t)linear_x + (int32_t)angular_z;

    /*
     * Normalize so that mixing never exceeds +/-1000.
     */
    max_abs = labs(left);
    if (labs(right) > max_abs) {
        max_abs = labs(right);
    }

    if (max_abs > VEHICLE_AXIS_MAX) {
        left = (left * VEHICLE_AXIS_MAX) / max_abs;
        right = (right * VEHICLE_AXIS_MAX) / max_abs;
    }

    speed_limit = clamp_u8(mgr->current_speed_limit_pct,
                           VEHICLE_SPEED_MIN_PCT,
                           VEHICLE_SPEED_MAX_PCT);

    /*
     * Convert normalized request into signed motor percentage.
     * Still not direct PWM; this is only the target before ramp limiting.
     */
    mgr->target_left_pct = (int16_t)((left * speed_limit) / VEHICLE_AXIS_MAX);
    mgr->target_right_pct = (int16_t)((right * speed_limit) / VEHICLE_AXIS_MAX);

    mgr->target_left_pct = clamp_i16(mgr->target_left_pct, -100, +100);
    mgr->target_right_pct = clamp_i16(mgr->target_right_pct, -100, +100);
}

static bool command_has_timed_out(const vehicle_control_manager_t *mgr,
                                  uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!mgr->has_active_motion_cmd) {
        return true;
    }

    elapsed_ms = now_ms - mgr->last_command_time_ms;

    return elapsed_ms > mgr->active_cmd.ttl_ms;
}

vehicle_control_manager_t * get_vehicle_manager_inst( void ) {
    return &mgr;
}

void vehicle_control_manager_init(const vehicle_control_config_t *config)
{
    memset(&mgr, 0, sizeof(vehicle_control_manager_t));

    if (config != NULL) {
        mgr.config = *config;
    } else {
        mgr.config.default_speed_limit_pct = DEFAULT_SPEED_LIMIT_PCT;
        mgr.config.default_ttl_ms = DEFAULT_TTL_MS;
        mgr.config.ramp_up_pct_per_sec = DEFAULT_RAMP_UP_PCT_PER_SEC;
        mgr.config.ramp_down_pct_per_sec = DEFAULT_RAMP_DOWN_PCT_PER_SEC;
        mgr.config.max_update_dt_ms = DEFAULT_MAX_UPDATE_DT_MS;
    }

    mgr.config.default_speed_limit_pct =
        clamp_u8(mgr.config.default_speed_limit_pct, 0u, 100u);

    if (mgr.config.default_ttl_ms == 0u) {
        mgr.config.default_ttl_ms = DEFAULT_TTL_MS;
    }

    if (mgr.config.ramp_up_pct_per_sec == 0u) {
        mgr.config.ramp_up_pct_per_sec = DEFAULT_RAMP_UP_PCT_PER_SEC;
    }

    if (mgr.config.ramp_down_pct_per_sec == 0u) {
        mgr.config.ramp_down_pct_per_sec = DEFAULT_RAMP_DOWN_PCT_PER_SEC;
    }

    if (mgr.config.max_update_dt_ms == 0u) {
        mgr.config.max_update_dt_ms = DEFAULT_MAX_UPDATE_DT_MS;
    }

    mgr.current_speed_limit_pct = mgr.config.default_speed_limit_pct;
}

void vehicle_control_manager_handle_command(vehicle_control_manager_t *mgr,
                                            const vehicle_motion_command_t *cmd)
{
    if ((mgr == NULL) || (cmd == NULL)) {
        return;
    }

    if (cmd->version != VEHICLE_COMMAND_VERSION) {
        return;
    }

    switch (cmd->command_type) {
    case VEHICLE_CMD_SET_SPEED:
        mgr->current_speed_limit_pct =
            clamp_u8(cmd->speed_limit_pct, VEHICLE_SPEED_MIN_PCT, VEHICLE_SPEED_MAX_PCT);
        break;

    case VEHICLE_CMD_STOP:
        /*
         * For this prototype:
         * C clears emergency stop and requests normal stop.
         */
        mgr->emergency_stop_latched = false;
        mgr->has_active_motion_cmd = false;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
        break;

    case VEHICLE_CMD_EMERGENCY_STOP:
        /*
         * Emergency stop is immediate and latched.
         * Motors go to zero immediately, not via normal ramp.
         */
        mgr->emergency_stop_latched = true;
        mgr->has_active_motion_cmd = false;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
        mgr->current_left_pct = 0;
        mgr->current_right_pct = 0;
        break;

    case VEHICLE_CMD_MOTION:
        if (mgr->emergency_stop_latched) {
            return;
        }

        mgr->active_cmd = *cmd;

        mgr->active_cmd.linear_x =
            clamp_i16(mgr->active_cmd.linear_x, VEHICLE_AXIS_MIN, VEHICLE_AXIS_MAX);

        mgr->active_cmd.angular_z =
            clamp_i16(mgr->active_cmd.angular_z, VEHICLE_AXIS_MIN, VEHICLE_AXIS_MAX);

        if (mgr->active_cmd.ttl_ms == 0u) {
            mgr->active_cmd.ttl_ms = mgr->config.default_ttl_ms;
        }

        if (mgr->active_cmd.speed_limit_pct > 0u) {
            mgr->current_speed_limit_pct =
                clamp_u8(mgr->active_cmd.speed_limit_pct,
                         VEHICLE_SPEED_MIN_PCT,
                         VEHICLE_SPEED_MAX_PCT);
        }

        mgr->last_command_time_ms = cmd->timestamp_ms;
        mgr->has_active_motion_cmd = true;
        mgr->command_timeout_active = false;
        break;

    case VEHICLE_CMD_HEARTBEAT:
    default:
        break;
    }
}

vehicle_motor_output_t vehicle_control_manager_update(vehicle_control_manager_t *mgr,
                                                      uint32_t now_ms)
{
    vehicle_motor_output_t output = {0};
    uint32_t dt_ms;
    uint16_t step_up;
    uint16_t step_down;

    if (mgr == NULL) {
        return output;
    }

    if (mgr->last_update_time_ms == 0u) {
        mgr->last_update_time_ms = now_ms;
    }

    dt_ms = now_ms - mgr->last_update_time_ms;
    mgr->last_update_time_ms = now_ms;

    dt_ms = clamp_u16((uint16_t)dt_ms, 1u, mgr->config.max_update_dt_ms);

    if (mgr->emergency_stop_latched) {
        mgr->current_left_pct = 0;
        mgr->current_right_pct = 0;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;

        output.left_pct = 0;
        output.right_pct = 0;
        output.emergency_stop_active = true;
        output.command_timeout_active = false;
        return output;
    }

    if (command_has_timed_out(mgr, now_ms)) {
        mgr->command_timeout_active = true;
        mgr->target_left_pct = 0;
        mgr->target_right_pct = 0;
    } else {
        mgr->command_timeout_active = false;
        calculate_differential_targets(mgr);
    }

    step_up = calculate_step(mgr->config.ramp_up_pct_per_sec, dt_ms);
    step_down = calculate_step(mgr->config.ramp_down_pct_per_sec, dt_ms);

    mgr->current_left_pct =
        slew_limit_motor_pct(mgr->current_left_pct,
                             mgr->target_left_pct,
                             step_up,
                             step_down);

    mgr->current_right_pct =
        slew_limit_motor_pct(mgr->current_right_pct,
                             mgr->target_right_pct,
                             step_up,
                             step_down);

    output.left_pct = mgr->current_left_pct;
    output.right_pct = mgr->current_right_pct;
    output.emergency_stop_active = mgr->emergency_stop_latched;
    output.command_timeout_active = mgr->command_timeout_active;

    return output;
}

void vehicle_control_manager_clear_emergency_stop(vehicle_control_manager_t *mgr)
{
    if (mgr == NULL) {
        return;
    }

    mgr->emergency_stop_latched = false;
    mgr->has_active_motion_cmd = false;
    mgr->target_left_pct = 0;
    mgr->target_right_pct = 0;
    mgr->current_left_pct = 0;
    mgr->current_right_pct = 0;
}