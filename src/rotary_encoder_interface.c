#include "rotary_encoder_interface.h"

#include <rcl_interfaces/msg/log.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float64_multi_array.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "rotary_encoder.h"
#include "rcCheck.h"

#define NUM_MOTORS 4

rotary_encoder_t* fl_encoder;
rotary_encoder_t* fr_encoder;
rotary_encoder_t* bl_encoder;
rotary_encoder_t* br_encoder;
std_msgs__msg__Float64MultiArray outgoing_wheel_positions;
rcl_publisher_t motor_joint_state_publisher;
rcl_timer_t encoders_publisher_timer;

rotary_encoder_t* createEncoder(int pinA, int pinB, int uniqueEncoderID)
{
    // Create rotary encoder instance
    rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)uniqueEncoderID, pinA, pinB);
    rotary_encoder_t *encoder = NULL;
    ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

    // Filter out glitch (1us)
    ESP_ERROR_CHECK(encoder -> set_glitch_filter(encoder, 1));

    // Start encoder
    ESP_ERROR_CHECK(encoder -> start(encoder));

    return encoder;
}

void init_encoders()
{
    fl_encoder = createEncoder(39, 36, 0);
    fr_encoder = createEncoder(34, 35, 1);
    bl_encoder = createEncoder(23, 22, 2);
    br_encoder = createEncoder(15, 2, 3);   
}

static void encoder_callback(rcl_timer_t *timer, int64_t last_call_time)
{
   outgoing_wheel_positions.data.data[0] = fl_encoder -> get_counter_value(fl_encoder); 
   outgoing_wheel_positions.data.data[1] = fr_encoder -> get_counter_value(fr_encoder); 
   outgoing_wheel_positions.data.data[2] = bl_encoder -> get_counter_value(bl_encoder); 
   outgoing_wheel_positions.data.data[3] = br_encoder -> get_counter_value(br_encoder); 
    
    rcl_publish(
        &motor_joint_state_publisher,
        &outgoing_wheel_positions,
        NULL
    );
}

void init_encoders_messaging(rcl_node_t *node_handle, rclc_support_t *support, rclc_executor_t *executor)
{
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
            &encoders_publisher_timer,
            support,
            RCL_MS_TO_NS(100),
            encoder_callback
        )
    );

    RCCHECK(
        rclc_executor_add_timer(
            executor,
            &encoders_publisher_timer
        )
    );

    outgoing_wheel_positions.data.data = (double *)malloc(NUM_MOTORS * sizeof(double)); // I don't like this, but I have no choice...
    outgoing_wheel_positions.data.size = NUM_MOTORS;
    
    for (int i = 0; i < NUM_MOTORS; ++i)
    {
        outgoing_wheel_positions.data.data[i] = 0.0;
    }
}