
#ifndef POSITIONING_H
#define POSITIONING_H

typedef struct {
    float x;
    float y;
} Position;

Position positioning_position_from_angles(float alpha, float beta, float gamma);

#endif

