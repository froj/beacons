
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "kalman.h"
#include "beacon_config.h"

// data types

/*
 * | k1 |
 * | k2 |
 */
typedef struct {
    matrix2d_t _k1;
    matrix2d_t _k2;
} kalman_gain_t;

typedef struct {
    float _x;
    float _y;
} vec2d_t;

// public function prototypes
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

// private function prototypes
static uint8_t feq(float a, float b);

// kalman functions
static void predict_state(
        const robot_state_t * src,
        float delta_t,
        robot_state_t * dest);
static void predict_covariance(
        const covariance_t * src,
        const covariance_t * process_noise_cov,
        float delta_t,
        covariance_t * dest);
static void kalman_gain(
        const covariance_t * predicted_covariance,
        const matrix2d_t * measurement_noise_cov,
        kalman_gain_t * dest);
static void update_state(
        const robot_state_t * predicted,
        const kalman_gain_t * gain,
        const vec2d_t * residual,
        robot_state_t * dest);
static void update_covariance(
        const covariance_t * predicted_cov,
        const kalman_gain_t * gain,
        covariance_t * dest);
static void process_noise_covariance(
        float delta_t,
        covariance_t * dest);

// 2x2 matrix functionality
static void m_mult(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest);
static void m_add(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest);
static void m_diff(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest);
static void m_trans(const matrix2d_t * m1, matrix2d_t * dest);
static uint8_t m_inv(const matrix2d_t * m1, matrix2d_t * dest);
static void m_scalar_mult(
        float scalar,
        const matrix2d_t * m1,
        matrix2d_t * dest);
static void m_vec_mult(
        const matrix2d_t * m,
        const vec2d_t * v,
        vec2d_t * dest);
static void m_init_identity(matrix2d_t * dest);

static void cov_add(
        const covariance_t * c1,
        const covariance_t * c2,
        covariance_t * dest);


// public function implementations

void kalman_init(
        kalman_robot_handle_t * handle,
        const robot_pos_t * initial_config)
{
    // initialize mutex
    os_mutex_init(&(handle->_mutex));

    os_mutex_take(&(handle->_mutex));

    // set initial state of robot
    handle->_state._x = initial_config->x;
    handle->_state._y = initial_config->y;
    // assume that the robot is standing still initially
    handle->_state._v_x = 0.0f;
    handle->_state._v_y = 0.0f;

    // set initial state covariance
    handle->_state_covariance._cov_a._a = initial_config->var_x;
    handle->_state_covariance._cov_a._b = initial_config->cov_xy;
    handle->_state_covariance._cov_a._c = initial_config->cov_xy;
    handle->_state_covariance._cov_a._d = initial_config->var_y;

    handle->_state_covariance._cov_b._a = 0.0f;
    handle->_state_covariance._cov_b._b = 0.0f;
    handle->_state_covariance._cov_b._c = 0.0f;
    handle->_state_covariance._cov_b._d = 0.0f;

    handle->_state_covariance._cov_c._a = 0.0f;
    handle->_state_covariance._cov_c._b = 0.0f;
    handle->_state_covariance._cov_c._c = 0.0f;
    handle->_state_covariance._cov_c._d = 0.0f;

    handle->_state_covariance._cov_d._a = 0.0f;
    handle->_state_covariance._cov_d._b = 0.0f;
    handle->_state_covariance._cov_d._c = 0.0f;
    handle->_state_covariance._cov_d._d = 0.0f;

    // set default measurment covariance
    handle->_measurement_covariance._a = MEAS_VAR_X;
    handle->_measurement_covariance._b = MEAS_COV_XY;
    handle->_measurement_covariance._c = MEAS_COV_XY;
    handle->_measurement_covariance._d = MEAS_VAR_Y;

    os_mutex_release(&(handle->_mutex));
}

void kalman_update(
        kalman_robot_handle_t * handle,
        const position_t * measurement,
        float delta_t,
        robot_pos_t * dest)
{
    os_mutex_take(&(handle->_mutex));

    // predict new state
    robot_state_t pred_state;
    predict_state(&(handle->_state), delta_t, &pred_state);

    // predict new covariance
    covariance_t proc_noise_cov;
    process_noise_covariance(delta_t, &proc_noise_cov);
    covariance_t pred_cov;
    predict_covariance(
            &(handle->_state_covariance),
            &proc_noise_cov,
            delta_t,
            &pred_cov);

    // compute kalman gain
    kalman_gain_t gain;
    kalman_gain(&pred_cov, &(handle->_measurement_covariance), &gain);

    // compute difference between prediction and measurement
    vec2d_t residual;
    residual._x = measurement->x - pred_state._x;
    residual._y = measurement->y - pred_state._y;

    // estimate new state considering measurement
    update_state(&pred_state, &gain, &residual, &(handle->_state));

    update_covariance(&pred_cov, &gain, &(handle->_state_covariance));

    // write resulting position (and associated variances) to dest
    dest->x = handle->_state._x;
    dest->y = handle->_state._y;
    dest->var_x = handle->_state_covariance._cov_a._a;
    dest->var_y = handle->_state_covariance._cov_a._d;
    dest->cov_xy = handle->_state_covariance._cov_a._b;

    os_mutex_release(&(handle->_mutex));
}

void kalman_update_measurement_covariance(
        kalman_robot_handle_t * handle,
        float var_x,
        float var_y,
        float cov_xy)
{
    os_mutex_take(&(handle->_mutex));

    handle->_measurement_covariance._a = var_x;
    handle->_measurement_covariance._b = cov_xy;
    handle->_measurement_covariance._c = cov_xy;
    handle->_measurement_covariance._d = var_x;

    os_mutex_release(&(handle->_mutex));
}

// private function implementations

static uint8_t feq(float a, float b)
{
    static const float EPSILON = 0.0001f;
    return fabs(a - b) < EPSILON;
}

// kalman functions
static void predict_state(
        const robot_state_t * src,
        float delta_t,
        robot_state_t * dest)
{
    float n_x = src->_x + delta_t * src->_v_x;
    float n_y = src->_y + delta_t * src->_v_y;

    dest->_x = n_x;
    dest->_y = n_y;
    dest->_v_x = src->_v_x;
    dest->_v_y = src->_v_y;
}

static void predict_covariance(
        const covariance_t * src,
        const covariance_t * process_noise_cov,
        float delta_t,
        covariance_t * dest)
{
    // Ddest = Dsrc
    dest->_cov_d = src->_cov_d;

    // D = dt*D
    matrix2d_t dtD;
    m_scalar_mult(delta_t, &(src->_cov_d), &dtD);

    // Bdest = Bsrc + dt*Dsrc
    m_add(&(src->_cov_b), &dtD, &(dest->_cov_b));

    // Cdest = Csrc + dt*Dsrc
    m_add(&(src->_cov_c), &dtD, &(dest->_cov_c));

    // D = dt*dt*D
    m_scalar_mult(delta_t, &dtD, &dtD);

    // B = dt*B
    matrix2d_t dtB;
    m_scalar_mult(delta_t, &(src->_cov_b), &dtB);

    // C = dt*C
    matrix2d_t dtC;
    m_scalar_mult(delta_t, &(src->_cov_c), &dtC);

    // Adest = Asrc + dt*B + dt*C + dt*dt*D
    m_add(&dtD, &dtB, &dtD);
    m_add(&dtD, &dtC, &dtD);
    m_add(&dtD, &(src->_cov_a), &(dest->_cov_a));

    // add noise covariance Q
    cov_add(dest, process_noise_cov, dest);
}

static void kalman_gain(
        const covariance_t * predicted_covariance,
        const matrix2d_t * measurement_noise_cov,
        kalman_gain_t * dest)
{
    matrix2d_t residual;

    m_add(&(predicted_covariance->_cov_a), measurement_noise_cov, &residual);
    uint8_t inverse_success = m_inv(&residual, &residual);

    m_mult(&(predicted_covariance->_cov_a), &residual, &(dest->_k1));
    m_mult(&(predicted_covariance->_cov_c), &residual, &(dest->_k2));
}

static void update_state(
        const robot_state_t * predicted,
        const kalman_gain_t * gain,
        const vec2d_t * residual,
        robot_state_t * dest)
{
    vec2d_t upper;
    m_vec_mult(&(gain->_k1), residual, &upper);

    vec2d_t lower;
    m_vec_mult(&(gain->_k2), residual, &lower);

    dest->_x = predicted->_x + upper._x;
    dest->_y = predicted->_y + upper._y;

    dest->_v_x = predicted->_v_x + lower._x;
    dest->_v_y = predicted->_v_y + lower._y;
}

static void update_covariance(
        const covariance_t * predicted_cov,
        const kalman_gain_t * gain,
        covariance_t * dest)
{
    // Adest = Asrc - K1*Asrc
    m_mult(&(gain->_k1), &(predicted_cov->_cov_a), &(dest->_cov_a));
    m_diff(&(predicted_cov->_cov_a), &(dest->_cov_a), &(dest->_cov_a));

    // Bdest = Bsrc - K1*Bsrc
    m_mult(&(gain->_k1), &(predicted_cov->_cov_b), &(dest->_cov_b));
    m_diff(&(predicted_cov->_cov_b), &(dest->_cov_b), &(dest->_cov_b));

    // Cdest = Csrc - K2*Asrc
    m_mult(&(gain->_k2), &(predicted_cov->_cov_a), &(dest->_cov_c));
    m_diff(&(predicted_cov->_cov_c), &(dest->_cov_c), &(dest->_cov_c));

    // Ddest = Dsrc - K2*Bsrc
    m_mult(&(gain->_k2), &(predicted_cov->_cov_b), &(dest->_cov_d));
    m_diff(&(predicted_cov->_cov_d), &(dest->_cov_d), &(dest->_cov_d));
}

static void process_noise_covariance(
        float delta_t,
        covariance_t * dest)
{
    static const float base_factor = 0.0625f * MAX_ACC; // 1/16 * a

    m_init_identity(&(dest->_cov_a));
    m_init_identity(&(dest->_cov_b));
    m_init_identity(&(dest->_cov_c));
    m_init_identity(&(dest->_cov_d));

    float factor = 0.25f * delta_t * delta_t * delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->_cov_a), &(dest->_cov_a));
    factor = 0.5f * delta_t * delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->_cov_b), &(dest->_cov_b));
    m_scalar_mult(factor * base_factor, &(dest->_cov_c), &(dest->_cov_c));
    factor = delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->_cov_d), &(dest->_cov_d));
}


// matrix function implementation

static void m_mult(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    // incase m1 == dest or m2 == dest
    float n_a = m1->_a * m2->_a + m1->_b * m2->_c;
    float n_b = m1->_a * m2->_b + m1->_b * m2->_d;
    float n_c = m1->_c * m2->_a + m1->_d * m2->_c;
    float n_d = m1->_c * m2->_b + m1->_d * m2->_d;

    dest->_a = n_a;
    dest->_b = n_b;
    dest->_c = n_c;
    dest->_d = n_d;
}

static void m_add(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    dest->_a = m1->_a + m2->_a;
    dest->_b = m1->_b + m2->_b;
    dest->_c = m1->_c + m2->_c;
    dest->_d = m1->_d + m2->_d;
}

static void m_diff(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    dest->_a = m1->_a - m2->_a;
    dest->_b = m1->_b - m2->_b;
    dest->_c = m1->_c - m2->_c;
    dest->_d = m1->_d - m2->_d;
}

static void m_trans(const matrix2d_t * m1, matrix2d_t * dest)
{
    // incase m1 == dest
    float new_b = m1->_c;
    float new_c = m1->_b;

    dest->_a = m1->_a;
    dest->_b = new_b;
    dest->_c = new_c;
    dest->_d = m1->_d;
}

static uint8_t m_inv(const matrix2d_t * m1, matrix2d_t * dest)
{
    float det = m1->_a * m1->_d - m1->_b * m1->_c;

    if(feq(det, 0.0f))
    {
        return 0;
    }

    det = 1 / det;

    float n_a = det * m1->_d;
    float n_b = det * m1->_b;
    float n_c = det * m1->_c;
    float n_d = det * m1->_a;

    dest->_a = n_a;
    dest->_b = - n_b;
    dest->_c = - n_c;
    dest->_d = n_d;

    return 1;
}

static void m_scalar_mult(
        float scalar,
        const matrix2d_t * m1,
        matrix2d_t * dest)
{
    dest->_a = scalar * m1->_a;
    dest->_b = scalar * m1->_b;
    dest->_c = scalar * m1->_c;
    dest->_d = scalar * m1->_d;
}

static void m_vec_mult(
        const matrix2d_t * m,
        const vec2d_t * v,
        vec2d_t * dest)
{
    float n_x = m->_a * v->_x + m->_b * v->_y;
    float n_y = m->_c * v->_x + m->_d * v->_y;

    dest->_x = n_x;
    dest->_y = n_y;
}

static void m_init_identity(matrix2d_t * dest)
{
    dest->_a = 1.0f;
    dest->_b = 0.0f;
    dest->_c = 0.0f;
    dest->_d = 1.0f;
}

static void cov_add(
        const covariance_t * c1,
        const covariance_t * c2,
        covariance_t * dest)
{
    m_add(&(c1->_cov_a), &(c2->_cov_a), &(dest->_cov_a));
    m_add(&(c1->_cov_b), &(c2->_cov_b), &(dest->_cov_b));
    m_add(&(c1->_cov_c), &(c2->_cov_c), &(dest->_cov_c));
    m_add(&(c1->_cov_d), &(c2->_cov_d), &(dest->_cov_d));
}
