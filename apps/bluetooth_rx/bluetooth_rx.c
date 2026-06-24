#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include "bluetooth_rx.h"
#include "vehicle_command.h"
#include "vehicle_command_parser.h"
#include "vehicle_control_manager.h"

#define BT_UART_NODE DT_NODELABEL(usart0)

#define MY_STACK_SIZE 1024
#define MY_PRIORITY   5

bool command_ready = false;
static const struct device *bt_uart = DEVICE_DT_GET(BT_UART_NODE);
static char cmd_buf[4] = {0}; // 3 chars + null terminator

K_THREAD_STACK_DEFINE(bt_stack_area, MY_STACK_SIZE);
static struct k_thread bt_thread_data;
static k_tid_t bt_thread_id;

static void run_bluethooth_thread(void *p1, void *p2, void *p3);

static void parse_direction_stream(char incoming_char)
{
    // Shift the buffer left by 1 position to make room for the new character
    cmd_buf[0] = cmd_buf[1];
    cmd_buf[1] = cmd_buf[2];
    cmd_buf[2] = incoming_char;
    cmd_buf[3] = '\0'; // Ensure string is always null-terminated

    // Check 3-character commands using strcmp
    if (strcmp(cmd_buf, "DWN") == 0) {
        printk("Command Detected: DWN\n");
        command_ready = true;
    } 
    else if (strcmp(cmd_buf, "LFT") == 0) {
        printk("Command Detected: LFT\n");
        command_ready = true;
    } 
    else if (strcmp(cmd_buf, "RGT") == 0) {
        printk("Command Detected: RGT\n");
        command_ready = true;
    }
    // Check 2-character command ("UP") using the last two positions
    else if (cmd_buf[1] == 'U' && cmd_buf[2] == 'P') {
        printk("Command Detected: UP\n");
        cmd_buf[0] = cmd_buf[1];
        cmd_buf[1] = cmd_buf[2];
        cmd_buf[2] = '\0';
        command_ready = true;
    }
}

static void bt_uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    static uint32_t count = 0;
    ARG_UNUSED(user_data);
    // Directs Zephyr to update internal peripheral tracking status
    uart_irq_update(dev);

    // CRITICAL: Loop while data is available in the hardware FIFO
    while (uart_irq_rx_ready(dev)) {
        while (uart_fifo_read(dev, &c, 1) > 0) {
            parse_direction_stream(c);
            if (c >= 32 && c <= 126) {
                printk("BT RX: char='%c', hex=0x%02X\n", c, c);
            } else {
                printk("BT RX: non-printable, hex=0x%02X\n", c);
            }
        }
    }

    printk("%d\n", ++count);
}

void bluetooth_rx_init(void) {
    if (!device_is_ready(bt_uart)) {
        printk("Bluetooth UART not ready\n");
        return;
    }

    uart_irq_callback_user_data_set(bt_uart, bt_uart_cb, NULL);
    uart_irq_rx_enable(bt_uart);

    bt_thread_id = k_thread_create(
        &bt_thread_data,
        bt_stack_area,
        K_THREAD_STACK_SIZEOF(bt_stack_area),
        run_bluethooth_thread,
        NULL, NULL, NULL,
        MY_PRIORITY,
        0,
        K_NO_WAIT
    );

    printk("Bluetooth UART test started\n");
}

static void run_bluethooth_thread(void *p1, void *p2, void *p3)
{
    vehicle_motion_command_t out_cmd;
    vehicle_control_manager_t * vehicle_manger = NULL;
    bool parse_status = false;

    while (1) {
        if( NULL == vehicle_manger ) {
            vehicle_manger = get_vehicle_manager_inst();
        }

        if(( NULL != vehicle_manger ) && ( true == command_ready )) {
            parse_status = vehicle_parse_ascii_command( cmd_buf, 
                                         VEHICLE_SOURCE_HC05_UART,
                                         k_uptime_get_32(),
                                         &out_cmd );
            if( true == parse_status ) {
                vehicle_control_manager_handle_command( vehicle_manger,
                                                        &out_cmd );                                                        
            }
            
            parse_status  = false;
            command_ready = false;
            memset( &out_cmd, 0x0, sizeof( vehicle_motion_command_t ));
        }

        k_msleep(50);
    }
}