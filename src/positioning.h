
#ifndef POSITIONING_H
#define POSITIONING_H

typedef struct {
    const float x;
    const float y;
} position_t;

typedef struct {
    const position_t * const point_a;
    const position_t * const point_b;
    const position_t * const point_c;
    const float cotangent_at_a;
    const float cotangent_at_b;
    const float cotangent_at_c;
} reference_triangle_t;

// initializes a reference_triangle_t from points a, b and c
// computes cotangents of angles at each point
// copies the result to the memory location specified by 'output'
// 
// doesn't do anything if any of the input parameters are NULL
void positioning_reference_triangle_from_points(
        const position_t * a, 
        const position_t * b, 
        const position_t * c, 
        reference_triangle_t * output);

// computes cartesian coordinates from angles with respect to 
// a reference triangle
// copies result to the memory location specified by 'output'
// 
// returns 1 if the result is to be trusted and can safely be used
// returns 0 if either input params are invalid (angles don't sum to 2 Pi,
// pointers are NULL) or result lies on/near the circumcircle of the 
// reference triangle in which case the position cannot be computed
// reliably
uint8_t positioning_from_angles(
        float alpha, float beta, 
        float gamma, 
        const reference_triangle_t * t, 
        position_t * output);

#endif

