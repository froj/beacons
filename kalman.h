
#ifndef BEACON_KALMAN_H
#define BEACON_KALMAN_H

#include "positioning.h"
#include "platform-abstraction/mutex.h"

// position and associated covariances predicted by kalman filter
//
// also used for initialization of kalman filter
typedef struct {
    float x;
    float y;
    float var_x;
    float var_y;
    float cov_xy;
} robot_pos_t;

// ATTENTION : this type is only exported to allow static allocation
// of kalman_robot_handle_t
//
// internal state used by kalman filter
typedef struct {
    float _x;
    float _y;
    float _v_x;
    float _v_y;
} robot_state_t;

// WARNING : this type is only exported to allow static allocation
// of kalman_robot_handle_t
//
// | a b |
// | c d |
typedef struct {
    float _a; 
    float _b;
    float _c;
    float _d;
} matrix2d_t;

// WARNING : this type is only exported to allow static allocation
// of kalman_robot_handle_t
typedef struct {
    matrix2d_t _cov_a;
    matrix2d_t _cov_b;
    matrix2d_t _cov_c;
    matrix2d_t _cov_d;
} covariance_t;


// WARNING : this type should be opaque, its only here to 
// allow static allocation by user
typedef struct {
    mutex_t _mutex;
    robot_state_t _state;
    covariance_t _state_covariance;
    matrix2d_t _measurement_covariance;
} kalman_robot_handle_t;

// intializes all fields of 'handle'
// 'initial_config' holds starting position (and associated covariances) 
//
// call this function before any call to 'kalman_update' or bad things will
// probably happen
//
// kalman_init, kalman_update and kalman_update_measurement_covariance cannot
// be called with the same handle concurrently (blocked by mutex)
void kalman_init(
        kalman_robot_handle_t * handle,
        const robot_pos_t * initial_config);

// updates state estimates with information provided by 'measurement'
// 'delta_t' should be the time since the last update
// 
// writes update estimate of position (and associated covariance) to
// memory pointed by 'dest'
//
// kalman_init, kalman_update and kalman_update_measurement_covariance cannot
// be called with the same handle concurrently (blocked by mutex)
void kalman_update(
        kalman_robot_handle_t * handle,
        const position_t * measurement,
        float delta_t,
        robot_pos_t * dest);

// update measurement covariance in case you don't want to use
// the default provided in 'beacon_config.h'
//
// kalman_init, kalman_update and kalman_update_measurement_covariance cannot
// be called with the same handle concurrently (blocked by mutex)
void kalman_update_measurement_covariance(
        kalman_robot_handle_t * handle,
        float var_x,
        float var_y,
        float cov_xy);

#endif
