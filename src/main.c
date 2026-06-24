#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "bluetooth_rx.h"
#include "vehicle_control_manager.h"

int main(void)
{
    vehicle_control_config_t config = {
        .default_speed_limit_pct = 40,
        .default_ttl_ms = 400,

        /*
         * Gentle initial values:
         * 0% to 100% takes about 1.25 seconds.
         * 100% to 0% takes about 0.6 seconds.
         */
        .ramp_up_pct_per_sec = 80,
        .ramp_down_pct_per_sec = 160,

        .max_update_dt_ms = 100,
    };

    bluetooth_rx_init();
    /* Initialize Vehicle command manager */
    vehicle_control_manager_init( &config );

    while (1) {        
        k_sleep(K_SECONDS(1));
    }
}