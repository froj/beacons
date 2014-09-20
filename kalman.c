
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
    matrix2d_t k1;
    matrix2d_t k2;
} kalman_gain_t;

typedef struct {
    float x;
    float y;
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
    os_mutex_init(&(handle->mutex));

    os_mutex_take(&(handle->mutex));

    // set initial state of robot
    handle->state.x = initial_config->x;
    handle->state.y = initial_config->y;
    handle->state.v_x = 0.0f;
    handle->state.v_y = 0.0f;

    // set initial state covariance
    handle->state_covariance.cov_a.a = initial_config->var_x;
    handle->state_covariance.cov_a.b = initial_config->cov_xy;
    handle->state_covariance.cov_a.c = initial_config->cov_xy;
    handle->state_covariance.cov_a.d = initial_config->var_y;

    handle->state_covariance.cov_b.a = 0.0f;
    handle->state_covariance.cov_b.b = 0.0f;
    handle->state_covariance.cov_b.c = 0.0f;
    handle->state_covariance.cov_b.d = 0.0f;

    handle->state_covariance.cov_c.a = 0.0f;
    handle->state_covariance.cov_c.b = 0.0f;
    handle->state_covariance.cov_c.c = 0.0f;
    handle->state_covariance.cov_c.d = 0.0f;

    handle->state_covariance.cov_d.a = 0.0f;
    handle->state_covariance.cov_d.b = 0.0f;
    handle->state_covariance.cov_d.c = 0.0f;
    handle->state_covariance.cov_d.d = 0.0f;

    // set default measurment covariance
    handle->measurement_covariance.a = MEAS_VAR_X;
    handle->measurement_covariance.b = MEAS_COV_XY;
    handle->measurement_covariance.c = MEAS_COV_XY;
    handle->measurement_covariance.d = MEAS_VAR_Y;

    os_mutex_release(&(handle->mutex));
}

void kalman_update(
        kalman_robot_handle_t * handle,
        const position_t * measurement,
        float delta_t,
        robot_pos_t * dest)
{
    os_mutex_take(&(handle->mutex));

    robot_state_t pred_state;
    predict_state(&(handle->state), delta_t, &pred_state);

    covariance_t proc_noise_cov;
    process_noise_covariance(delta_t, &proc_noise_cov);
    covariance_t pred_cov;
    predict_covariance(
            &(handle->state_covariance),
            &proc_noise_cov,
            delta_t,
            &pred_cov);

    kalman_gain_t gain;
    kalman_gain(&pred_cov, &(handle->measurement_covariance), &gain);

    vec2d_t residual;
    residual.x = measurement->x - pred_state.x;
    residual.y = measurement->y - pred_state.y;

    update_state(&pred_state, &gain, &residual, &(handle->state));

    update_covariance(&pred_cov, &gain, &(handle->state_covariance));

    dest->x = handle->state.x;
    dest->y = handle->state.y;
    dest->var_x = handle->state_covariance.cov_a.a;
    dest->var_y = handle->state_covariance.cov_a.d;
    dest->cov_xy = handle->state_covariance.cov_a.b;

    os_mutex_release(&(handle->mutex));
}

void kalman_update_measurement_covariance(
        kalman_robot_handle_t * handle,
        float var_x,
        float var_y,
        float cov_xy)
{
    os_mutex_take(&(handle->mutex));

    handle->measurement_covariance.a = var_x;
    handle->measurement_covariance.b = cov_xy;
    handle->measurement_covariance.c = cov_xy;
    handle->measurement_covariance.d = var_x;

    os_mutex_release(&(handle->mutex));
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
    float n_x = src->x + delta_t * src->v_x;
    float n_y = src->y + delta_t * src->v_y;

    dest->x = n_x;
    dest->y = n_y;
    dest->v_x = src->v_x;
    dest->v_y = src->v_y;
}

static void predict_covariance(
        const covariance_t * src,
        const covariance_t * process_noise_cov,
        float delta_t,
        covariance_t * dest)
{
    // Ddest = Dsrc
    dest->cov_d = src->cov_d;

    // D = dt*D
    matrix2d_t dtD;
    m_scalar_mult(delta_t, &(src->cov_d), &dtD);

    // Bdest = Bsrc + dt*Dsrc
    m_add(&(src->cov_b), &dtD, &(dest->cov_b));

    // Cdest = Csrc + dt*Dsrc
    m_add(&(src->cov_c), &dtD, &(dest->cov_c));

    // D = dt*dt*D
    m_scalar_mult(delta_t, &dtD, &dtD);

    // B = dt*B
    matrix2d_t dtB;
    m_scalar_mult(delta_t, &(src->cov_b), &dtB);

    // C = dt*C
    matrix2d_t dtC;
    m_scalar_mult(delta_t, &(src->cov_c), &dtC);

    // Adest = Asrc + dt*B + dt*C + dt*dt*D
    m_add(&dtD, &dtB, &dtD);
    m_add(&dtD, &dtC, &dtD);
    m_add(&dtD, &(src->cov_a), &(dest->cov_a));

    // add noise covariance Q
    cov_add(dest, process_noise_cov, dest);
}

static void kalman_gain(
        const covariance_t * predicted_covariance,
        const matrix2d_t * measurement_noise_cov,
        kalman_gain_t * dest)
{
    matrix2d_t residual;

    m_add(&(predicted_covariance->cov_a), measurement_noise_cov, &residual);
    uint8_t inverse_success = m_inv(&residual, &residual);

    m_mult(&(predicted_covariance->cov_a), &residual, &(dest->k1));
    m_mult(&(predicted_covariance->cov_c), &residual, &(dest->k2));
}

static void update_state(
        const robot_state_t * predicted,
        const kalman_gain_t * gain,
        const vec2d_t * residual,
        robot_state_t * dest)
{
    vec2d_t upper;
    m_vec_mult(&(gain->k1), residual, &upper);

    vec2d_t lower;
    m_vec_mult(&(gain->k2), residual, &lower);

    dest->x = predicted->x + upper.x;
    dest->y = predicted->y + upper.y;

    dest->v_x = predicted->v_x + lower.x;
    dest->v_y = predicted->v_y + lower.y;
}

static void update_covariance(
        const covariance_t * predicted_cov,
        const kalman_gain_t * gain,
        covariance_t * dest)
{
    // Adest = Asrc - K1*Asrc
    m_mult(&(gain->k1), &(predicted_cov->cov_a), &(dest->cov_a));
    m_diff(&(predicted_cov->cov_a), &(dest->cov_a), &(dest->cov_a));

    // Bdest = Bsrc - K1*Bsrc
    m_mult(&(gain->k1), &(predicted_cov->cov_b), &(dest->cov_b));
    m_diff(&(predicted_cov->cov_b), &(dest->cov_b), &(dest->cov_b));

    // Cdest = Csrc - K2*Asrc
    m_mult(&(gain->k2), &(predicted_cov->cov_a), &(dest->cov_c));
    m_diff(&(predicted_cov->cov_c), &(dest->cov_c), &(dest->cov_c));

    // Ddest = Dsrc - K2*Bsrc
    m_mult(&(gain->k2), &(predicted_cov->cov_b), &(dest->cov_d));
    m_diff(&(predicted_cov->cov_d), &(dest->cov_d), &(dest->cov_d));
}

static void process_noise_covariance(
        float delta_t,
        covariance_t * dest)
{
    static const float base_factor = 0.0625f * MAX_ACC; // 1/16 * a

    m_init_identity(&(dest->cov_a));
    m_init_identity(&(dest->cov_b));
    m_init_identity(&(dest->cov_c));
    m_init_identity(&(dest->cov_d));

    float factor = 0.25f * delta_t * delta_t * delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->cov_a), &(dest->cov_a));
    factor = 0.5f * delta_t * delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->cov_b), &(dest->cov_b));
    m_scalar_mult(factor * base_factor, &(dest->cov_c), &(dest->cov_c));
    factor = delta_t * delta_t;
    m_scalar_mult(factor * base_factor, &(dest->cov_d), &(dest->cov_d));
}


// matrix function implementation

static void m_mult(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    // incase m1 == dest or m2 == dest
    float n_a = m1->a * m2->a + m1->b * m2->c;
    float n_b = m1->a * m2->b + m1->b * m2->d;
    float n_c = m1->c * m2->a + m1->d * m2->c;
    float n_d = m1->c * m2->b + m1->d * m2->d;

    dest->a = n_a;
    dest->b = n_b;
    dest->c = n_c;
    dest->d = n_d;
}

static void m_add(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    dest->a = m1->a + m2->a;
    dest->b = m1->b + m2->b;
    dest->c = m1->c + m2->c;
    dest->d = m1->d + m2->d;
}

static void m_diff(
        const matrix2d_t * m1,
        const matrix2d_t * m2,
        matrix2d_t * dest)
{
    dest->a = m1->a - m2->a;
    dest->b = m1->b - m2->b;
    dest->c = m1->c - m2->c;
    dest->d = m1->d - m2->d;
}

static void m_trans(const matrix2d_t * m1, matrix2d_t * dest)
{
    // incase m1 == dest
    float new_b = m1->c;
    float new_c = m1->b;

    dest->a = m1->a;
    dest->b = new_b;
    dest->c = new_c;
    dest->d = m1->d;
}

static uint8_t m_inv(const matrix2d_t * m1, matrix2d_t * dest)
{
    float det = m1->a * m1->d - m1->b * m1->c;

    if(feq(det, 0.0f))
    {
        return 0;
    }

    det = 1 / det;

    float n_a = det * m1->d;
    float n_b = det * m1->b;
    float n_c = det * m1->c;
    float n_d = det * m1->a;

    dest->a = n_a;
    dest->b = - n_b;
    dest->c = - n_c;
    dest->d = n_d;

    return 1;
}

static void m_scalar_mult(
        float scalar,
        const matrix2d_t * m1,
        matrix2d_t * dest)
{
    dest->a = scalar * m1->a;
    dest->b = scalar * m1->b;
    dest->c = scalar * m1->c;
    dest->d = scalar * m1->d;
}

static void m_vec_mult(
        const matrix2d_t * m,
        const vec2d_t * v,
        vec2d_t * dest)
{
    float n_x = m->a * v->x + m->b * v->y;
    float n_y = m->c * v->x + m->d * v->y;

    dest->x = n_x;
    dest->y = n_y;
}

static void m_init_identity(matrix2d_t * dest)
{
    dest->a = 1.0f;
    dest->b = 0.0f;
    dest->c = 0.0f;
    dest->d = 1.0f;
}

static void cov_add(
        const covariance_t * c1,
        const covariance_t * c2,
        covariance_t * dest)
{
    m_add(&(c1->cov_a), &(c2->cov_a), &(dest->cov_a));
    m_add(&(c1->cov_b), &(c2->cov_b), &(dest->cov_b));
    m_add(&(c1->cov_c), &(c2->cov_c), &(dest->cov_c));
    m_add(&(c1->cov_d), &(c2->cov_d), &(dest->cov_d));
}
