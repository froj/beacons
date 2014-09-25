
#include <stdlib.h>
#include <stdint.h>

#include "../positioning.h"
#include "../kalman.h"

#define POINT_A_X (3.0f)
#define POINT_A_Y (1.0f)

#define POINT_B_X (0.0f)
#define POINT_B_Y (2.0f)

#define POINT_C_X (0.0f)
#define POINT_C_Y (0.0f)


static kalman_robot_handle_t handle;
static robot_pos_t current;
static reference_triangle_t ref_triangle;

static position_t triangle_a = {POINT_A_X, POINT_A_Y};
static position_t triangle_b = {POINT_B_X, POINT_B_Y};
static position_t triangle_c = {POINT_C_X, POINT_C_Y};

float get_x(void)
{
    return current.x;
}

float get_y(void)
{
    return current.y;
}

float get_var_x(void)
{
    return current.var_x;
}

float get_var_y(void)
{
    return current.var_y;
}

float get_cov_xy(void)
{
    return current.cov_xy;
}

void setup(float x, float y)
{
    robot_pos_t init_pos;
    init_pos.x = x;
    init_pos.y = y;
    init_pos.var_x = 0.0f;
    init_pos.var_y = 0.0f;
    init_pos.cov_xy = 0.0f;

    kalman_init(&handle, &init_pos);
    positioning_reference_triangle_from_points(
            &triangle_a,
            &triangle_b,
            &triangle_c,
            &ref_triangle);
}

void update_meas_cov(float var_x, float var_y, float cov_xy)
{
    kalman_update_measurement_covariance(&handle, var_x, var_y, cov_xy);
}

void update_state(
        float alpha,
        float beta,
        float gamma,
        float delta_t,
        uint8_t considerMeas)
{
    position_t meas;
    positioning_from_angles(alpha, beta, gamma, &ref_triangle, &meas);

    if(considerMeas != 0) {
        kalman_update(&handle, &meas, delta_t, &current);
    } else {
        kalman_update(&handle, NULL, delta_t, &current);
    }
}
