#include "rotary_encoder_interface.h"

/* PCNT example -- Rotary Encoder

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/error_handling.h>

#include <std_msgs/msg/float64_multi_array.h>
#include "rcCheck.h"


#define NUM_MOTORS 4

rotary_encoder_t* fr_encoder;
rotary_encoder_t* fl_encoder;
rotary_encoder_t* bl_encoder;
rotary_encoder_t* br_encoder;
std_msgs__msg__Float64MultiArray outgoing_wheel_positions;
rcl_publisher_t motor_joint_state_publisher;
rcl_timer_t publisher_timer;


static const char *TAG = "example";

rotary_encoder_t* setupEncoder(int pinA, int pinB, int encoderID)
{
    uint32_t pcnt_unit = encoderID;

    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, pinA, pinB);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder->set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder->start(encoder));

    return encoder;
}

int getEncoderValue(rotary_encoder_t *encoder)
{
    return encoder->get_counter_value(encoder);
}

static void init_motor_pub_msg()
{
    outgoing_wheel_positions.data.data = (double *) malloc(NUM_MOTORS*sizeof(double)); // I don't like this, but I have no choice...
    outgoing_wheel_positions.data.size = NUM_MOTORS;
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        outgoing_wheel_positions.data.data[i] = 0.0;
    }
}

static void publisher_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{

   outgoing_wheel_positions.data.data[0] = getEncoderValue(fr_encoder); // get_rad();
   outgoing_wheel_positions.data.data[1] = getEncoderValue(fl_encoder); // get_rad();
   outgoing_wheel_positions.data.data[2] = getEncoderValue(bl_encoder); // get_rad();
   outgoing_wheel_positions.data.data[3] = getEncoderValue(br_encoder); // get_rad();

    rcl_publish(&motor_joint_state_publisher,
        &outgoing_wheel_positions,
        NULL
    );
}

void startEncoders(rcl_node_t *node_handle, rclc_support_t *support,
                              rcl_allocator_t *allocator, rclc_executor_t *executor)
{
    init_motor_pub_msg();

    RCCHECK(rclc_publisher_init_default(
        &motor_joint_state_publisher,
        node_handle,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "clearpath_motor_positions"));

    RCCHECK(rclc_timer_init_default(
        &publisher_timer,
        support,
        RCL_MS_TO_NS(100),
        publisher_timer_callback));

    RCCHECK(rclc_executor_add_timer(
        executor,
        &publisher_timer));

    fl_encoder = setupEncoder(34, 35, 1);
    fr_encoder = setupEncoder(39, 36, 2);
    bl_encoder = setupEncoder(23 ,22, 3);
    br_encoder = setupEncoder(15, 2, 4);
}
