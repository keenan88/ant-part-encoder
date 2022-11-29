#pragma once

#include <unistd.h>
#include <std_msgs/msg/float64_multi_array.h>
#include "rotary_encoder.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//#include "rotary_encoder.h"

#ifndef M_PI
#define M_PI 3.14159265358979
#endif


//rotary_encoder_t* setupEncoder(int pinA, int pinB, int encoderID);

int getEncoderValue(rotary_encoder_t *encoder);

void startEncoders(rcl_node_t *node_handle, rclc_support_t *support,
                              rcl_allocator_t *allocator, rclc_executor_t *executor);
