#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "positioning.h"

// public function prototypes
uint8_t positioning_from_angles(float alpha, float beta, float gamma, const reference_triangle_t * t, position_t * output);
void positioning_reference_triangle_from_points(const position_t * a, const position_t * b, const position_t * c, reference_triangle_t * output);
// private function prototypes
static uint8_t feq(float a, float b);
static float cotangent_from_points(const position_t * a, const position_t * b, const position_t * c);
static float dot_product(const position_t * a, const position_t * b);
static float cot(float alpha);

/*
 * implementation of public functions
 */
void positioning_reference_triangle_from_points(const position_t * a, const position_t * b, const position_t * c, reference_triangle_t * output)
{
    if(output == NULL || a == NULL || b == NULL || c == NULL)
    {
        return;
    }

    float cot_at_a = cotangent_from_points(b, a, c);
    float cot_at_b = cotangent_from_points(a, b, c);
    float cot_at_c = cotangent_from_points(b, c, a);

    reference_triangle_t t = {a, b, c, cot_at_a, cot_at_b, cot_at_c};

    memcpy(output, &t, sizeof(reference_triangle_t));
}

uint8_t positioning_from_angles(float alpha, float beta, float gamma, const reference_triangle_t * t, position_t * output)
{
    if(output == NULL || t == NULL)
    {
        return 0;
    }

    uint8_t valid = 1;

    // 
    float cot_alpha = cot(alpha);
    float cot_beta = cot(beta);
    float cot_gamma = cot(gamma);
    if(feq(cot_alpha, t->cotangent_at_a) || feq(cot_beta, t->cotangent_at_b) || feq(cot_gamma, t->cotangent_at_c))
    {
        valid = 0;
    }

    // compute barycentric coordinates from angles
    float barycentric_a = 1.0f / (t->cotangent_at_a - cot_alpha);
    float barycentric_b = 1.0f / (t->cotangent_at_b - cot_beta);
    float barycentric_c = 1.0f / (t->cotangent_at_c - cot_gamma);

    // normalize barycentric coordinates
    float magnitude = barycentric_a + barycentric_b + barycentric_c;
    barycentric_a /= magnitude;
    barycentric_b /= magnitude;
    barycentric_c /= magnitude;

    position_t result;

    float x = barycentric_a * t->point_a->x + barycentric_b * t->point_b->x + barycentric_c * t->point_c->x;
    float y = barycentric_a * t->point_a->y + barycentric_b * t->point_b->y + barycentric_c * t->point_c->y;

    position_t pos = {x, y};

    memcpy(output, &pos, sizeof(position_t));

    return valid;
}

/*
 * implementation of private functions
 */

//see: http://stackoverflow.com/questions/3738384/stable-cotangent
static inline float cot(float alpha) 
{
    return tan(M_PI_2 - alpha);
}

// helper function to compare floats for approximate equality
static uint8_t feq(float a, float b) 
{
    static const float EPSILON = 0.0001f;
    return fabs(a - b) < EPSILON;
}

// standard dot product function
static float dot_product(const position_t * a, const position_t * b)
{
    return a->x * b->x + a->y * b->y;
}

// compute the cotangent of the angle ABC (angle at B)
static float cotangent_from_points(const position_t * a, const position_t * b, const position_t * c)
{
    // get vector from point b to point a, BA
    position_t ba = {a->x - b->x, a->y - b->y};

    // get vector from point b to point c, BC
    position_t bc = {c->x - b->x, c->y - b->y};

    // dot product of BA and BC
    float dot_ba_bc = dot_product(&ba, &bc);
    // length of BA
    float len_ba = sqrt(dot_product(&ba, &ba));
    // length of BC
    float len_bc = sqrt(dot_product(&bc, &bc));

    // cosine between BA and BC
    float cosine = dot_ba_bc / (len_ba * len_bc);
    // sine between BA and BC
    float sine = sqrt(1 - (cosine * cosine));

    // cot = cos/sin
    return cosine / sine;
}

