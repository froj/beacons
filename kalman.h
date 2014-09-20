
#ifndef BEACON_KALMAN_H
#define BEACON_KALMAN_H

#include "positioning.h"
#include "platform-abstraction/mutex.h"

typedef struct {
    float x;
    float y;
    float var_x;
    float var_y;
    float cov_xy;
} robot_pos_t;

typedef struct {
    float x;
    float y;
    float v_x;
    float v_y;
} robot_state_t;

/*
 * | a b |
 * | c d |
 */
typedef struct {
    float a; 
    float b;
    float c;
    float d;
} matrix2d_t;

/*
 * Note:
 * since covariance matrices are symmetric cov_b == transpose(cov_c)
 */
typedef struct {
    matrix2d_t cov_a;
    matrix2d_t cov_b;
    matrix2d_t cov_c;
    matrix2d_t cov_d;
} covariance_t;


typedef struct {
    mutex_t mutex;
    robot_state_t state;
    covariance_t state_covariance;
    matrix2d_t measurement_covariance;
} kalman_robot_handle_t;

void kalman_init(
        kalman_robot_handle_t * handle,
        const robot_pos_t * initial_config);
void kalman_update(
        kalman_robot_handle_t * handle,
        const position_t * measurement,
        float delta_t,
        robot_pos_t * dest);
void kalman_update_measurement_covariance(
        kalman_robot_handle_t * handle,
        float var_x,
        float var_y,
        float cov_xy);

#endif
