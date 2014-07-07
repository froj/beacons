
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

void positioning_reference_triangle_from_points(const position_t * a, const position_t * b, const position_t * c, reference_triangle_t * output);

uint8_t positioning_from_angles(float alpha, float beta, float gamma, const reference_triangle_t * t, position_t * output);

#endif

