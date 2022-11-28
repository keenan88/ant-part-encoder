#pragma once

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void init_encoders();

void init_encoders_messaging(rcl_node_t *node_handle, rclc_support_t *support, rclc_executor_t *executor);