#ifndef BEACON_ANGLES_H_
#define BEACON_ANGLES_H_
/*
 * This module calculates angles from a stream of timestamps that signify
 * the passing of the laser at the different beacons.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <platform-abstraction/semaphore.h>
#include <platform-abstraction/mutex.h>

typedef struct {
    uint32_t _time_a;
    uint32_t _time_b;
    uint32_t _time_c;

    uint32_t _time_a_old;
    uint32_t _time_b_old;
    uint32_t _time_c_old;

    uint32_t _minimal_period;

    mutex_t access;
    semaphore_t measurement_ready;

    float alpha;
    float beta;
    float gamma;

} beacon_angles_t;


enum beacon_nb {
    A, B, C
};

void beacon_angles_init(beacon_angles_t *angles);
void beacon_angles_update_timestamp(beacon_angles_t *angles,
                                    enum beacon_nb beacon,
                                    uint32_t time);

void beacon_angles_set_minimal_period(beacon_angles_t *angles, uint32_t period);
int beacon_angles_calculate(beacon_angles_t *angles);

#ifdef __cplusplus
}
#endif

#endif
