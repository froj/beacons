#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "positioning.h"

#define POINT_A_X (0.0f)
#define POINT_A_Y (1.0f)

#define POINT_B_X (3.0f)
#define POINT_B_Y (0.0f)

#define POINT_C_X (3.0f)
#define POINT_C_Y (2.0f)


// public function prototypes
Position positioning_position_from_angles(float alpha, float beta, float gamma);
// private function prototypes
void init(void);
static uint8_t feq(float a, float b);
static float cotangent_from_points(const Position * a, const Position * b, const Position * c);
static float dot_product(const Position * a, const Position * b);
static float cot(float alpha);

// variables global to this module
static Position point_a;
static Position point_b;
static Position point_c;

static float cot_at_a;
static float cot_at_b;
static float cot_at_c;

/*
 * implementation of public functions
 */

Position positioning_position_from_angles(float alpha, float beta, float gamma)
{
    static uint8_t is_init = 0;
    if(!is_init) {
        init();
        is_init = 1;
    }

    // compute barycentric coordinates from angles
    float barycentric_a = 1 / (cot_at_a - cot(alpha));
    float barycentric_b = 1 / (cot_at_b - cot(beta));
    float barycentric_c = 1 / (cot_at_c - cot(gamma));

    // normalize barycentric coordinates
    float magnitude = barycentric_a + barycentric_b + barycentric_c;
    barycentric_a /= magnitude;
    barycentric_b /= magnitude;
    barycentric_c /= magnitude;

    Position result;

    result.x = barycentric_a * point_a.x + barycentric_b * point_b.x + barycentric_c * point_c.x;
    result.y = barycentric_a * point_a.y + barycentric_b * point_b.y + barycentric_c * point_c.y;

    return result;
}

/*
 * implementation of private functions
 */

static inline float cot(float alpha) 
{
    return 1/tan(alpha);
}

// helper function to compare floats for approximate equality
static uint8_t feq(float a, float b) 
{
    static const float EPSILON = 0.0001f;
    return fabs(a - b) < EPSILON;
}

// standard dot product function
static float dot_product(const Position * a, const Position * b)
{
    return a->x * b->x + a->y * b->y;
}

// compute the cotangent of the angle ABC (angle at B)
static float cotangent_from_points(const Position * a, const Position * b, const Position * c)
{
    // get vector from point b to point a, BA
    Position ba;
    ba.x = a->x - b->x;
    ba.y= a->y - b->y;

    // get vector from point b to point c, BC
    Position bc;
    bc.x = c->x - b->x;
    bc.y = c->y - b->y;

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

// initialize and setup module variabes
void init(void)
{
    point_a.x = POINT_A_X;
    point_a.y = POINT_A_Y;

    point_b.x = POINT_B_X;
    point_b.y = POINT_B_Y;

    point_c.x = POINT_C_X;
    point_c.y = POINT_C_Y;

    cot_at_a = cotangent_from_points(&point_b, &point_a, &point_c);

    cot_at_b = cotangent_from_points(&point_a, &point_b, &point_c);

    cot_at_a = cotangent_from_points(&point_b, &point_c, &point_a);
}
