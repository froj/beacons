
#ifndef POSITIONING_H
#define POSITIONING_H

#define POINT_A_X (0.0f)
#define POINT_A_Y (0.0f)

#define POINT_B_X (3.0f)
#define POINT_B_Y (1.0f)

#define POINT_C_X (0.0f)
#define POINT_C_Y (2.0f)

typedef struct {
    float x;
    float y;
} position_t;

position_t positioning_position_from_angles(float alpha, float beta, float gamma);

#endif

