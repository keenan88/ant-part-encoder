#pragma once

#include <unistd.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "rotary_encoder.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifndef M_PI
#define M_PI 3.14159265358979
#endif

void init_ros_encoder_interface(rcl_node_t *node_handle, rclc_support_t *support, rclc_executor_t *executor);
