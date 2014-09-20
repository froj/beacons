
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "kalman.h"

// data types
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

typedef struct {
    float x;
    float y;
    float v_x;
    float v_y;
} robot_state_t;

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

static void cov_add(
        const covariance_t * c1,
        const covariance_t * c2,
        covariance_t * dest);


// public function implementations


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
