#ifndef VEHICLE_CONTROL_MANAGER_H
#define VEHICLE_CONTROL_MANAGER_H

#include <stdint.h>
#include <stdbool.h>
#include "vehicle_command.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t default_speed_limit_pct;
    uint16_t default_ttl_ms;

    /*
     * Ramp values are percentage points per second.
     *
     * Example:
     *   ramp_up_pct_per_sec = 100
     * means:
     *   0% to 100% takes around 1 second.
     */
    uint16_t ramp_up_pct_per_sec;
    uint16_t ramp_down_pct_per_sec;

    /*
     * Minimum update period guard.
     * Helps avoid huge jumps if time calculation goes wrong.
     */
    uint16_t max_update_dt_ms;
} vehicle_control_config_t;

typedef struct {
    vehicle_control_config_t config;

    vehicle_motion_command_t active_cmd;

    uint8_t current_speed_limit_pct;

    bool has_active_motion_cmd;
    bool emergency_stop_latched;
    bool command_timeout_active;

    uint32_t last_command_time_ms;
    uint32_t last_update_time_ms;

    int16_t current_left_pct;
    int16_t current_right_pct;

    int16_t target_left_pct;
    int16_t target_right_pct;
} vehicle_control_manager_t;

vehicle_control_manager_t * get_vehicle_manager_inst( void );

void vehicle_control_manager_init(const vehicle_control_config_t *config);

void vehicle_control_manager_handle_command(vehicle_control_manager_t *mgr,
                                            const vehicle_motion_command_t *cmd);

vehicle_motor_output_t vehicle_control_manager_update(vehicle_control_manager_t *mgr,
                                                      uint32_t now_ms);

void vehicle_control_manager_clear_emergency_stop(vehicle_control_manager_t *mgr);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_CONTROL_MANAGER_H */