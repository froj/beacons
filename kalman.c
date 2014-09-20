
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "kalman.h"

// public function prototypes


// private function prototypes
static uint8_t feq(float a, float b);

// 2x2 matrix functionality

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



// public function implementations


// private function implementations

static uint8_t feq(float a, float b)
{
    static const float EPSILON = 0.0001f;
    return fabs(a - b) < EPSILON;
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
