/* PCNT example -- Rotary Encoder

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rotary_encoder_interface.h"
#include <rcl/error_handling.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "rcCheck.h"

#define NUM_ENCODERS 4
#define TICKS_PER_REV 4096

typedef struct {
    rotary_encoder_t* encoder;
    int pin_a, pin_b, ID;
} encoder_interface_t;

encoder_interface_t front_left_encoder_itfc =  {.pin_a = 35, .pin_b = 34, .ID = 1};
encoder_interface_t back_right_encoder_itfc =  {.pin_a = 39, .pin_b = 36, .ID = 2};
encoder_interface_t front_right_encoder_itfc = {.pin_a = 23, .pin_b = 22, .ID = 3};
encoder_interface_t back_left_encoder_itfc =   {.pin_a = 2, .pin_b = 15, .ID = 4};

std_msgs__msg__Float64MultiArray outgoing_wheel_positions;
rcl_publisher_t motor_joint_state_publisher;
rcl_timer_t publisher_timer;

rotary_encoder_t* setup_encoder(encoder_interface_t encoder_interface)
{
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)encoder_interface.ID, encoder_interface.pin_a, encoder_interface.pin_b);

    rotary_encoder_t *encoder = NULL;
    
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder -> set_glitch_filter(encoder, 1));

    ESP_ERROR_CHECK(encoder -> start(encoder));

    return encoder;
}


static void init_motor_pub_msg()
{
    outgoing_wheel_positions.data.data = (double *) malloc(NUM_ENCODERS*sizeof(double)); // I don't like this, but I have no choice...
    outgoing_wheel_positions.data.size = NUM_ENCODERS;
    for (int i = 0; i < NUM_ENCODERS; ++i)
    {
        outgoing_wheel_positions.data.data[i] = 0.0;
    }
}

int get_encoder_ticks(rotary_encoder_t *encoder)
{
    return encoder -> get_counter_value(encoder);
}


double get_encoder_radians(rotary_encoder_t *encoder)
{
    int ticks = get_encoder_ticks(encoder);
    double revs = 1.0 * ticks / TICKS_PER_REV;
    double radians = revs * 2 * M_PI;

    return radians;
}

static void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
   outgoing_wheel_positions.data.data[0] = get_encoder_radians(front_left_encoder_itfc.encoder);
   outgoing_wheel_positions.data.data[1] = get_encoder_radians(back_right_encoder_itfc.encoder);
   outgoing_wheel_positions.data.data[2] = get_encoder_radians(front_right_encoder_itfc.encoder);
   outgoing_wheel_positions.data.data[3] = get_encoder_radians(back_left_encoder_itfc.encoder);

    rcl_publish(
        &motor_joint_state_publisher,
        &outgoing_wheel_positions,
        NULL
    );
}

void init_ros_encoder_interface(rcl_node_t *node_handle, rclc_support_t *support, rclc_executor_t *executor)
{
    init_motor_pub_msg();

    RCCHECK(
        rclc_publisher_init_default(
            &motor_joint_state_publisher,
            node_handle,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
            "clearpath_motor_positions"
        )
    );

    RCCHECK(
        rclc_timer_init_default(
            &publisher_timer,
            support,
            RCL_MS_TO_NS(100),
            publisher_timer_callback
        )
    );

    RCCHECK(
        rclc_executor_add_timer(
            executor,
            &publisher_timer
        )
    );

    front_left_encoder_itfc.encoder =  setup_encoder(front_left_encoder_itfc);
    back_right_encoder_itfc.encoder =  setup_encoder(back_right_encoder_itfc);
    front_right_encoder_itfc.encoder = setup_encoder(front_right_encoder_itfc);
    back_left_encoder_itfc.encoder =   setup_encoder(back_left_encoder_itfc);
}
