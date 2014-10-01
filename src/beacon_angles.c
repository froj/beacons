#include "beacon_angles.h"
#include <math.h>
#include <stdio.h>
#include <platform-abstraction/criticalsection.h>

void beacon_angles_init(beacon_angles_t *angles)
{
    angles->_time_a = 0;
    angles->_time_b = 0;
    angles->_time_c = 0;

    angles->_time_a_old = 0;
    angles->_time_b_old = 0;
    angles->_time_c_old = 0;

    angles->_minimal_period = 0;

    angles->alpha = 0.0;
    angles->beta = 0.0;
    angles->gamma = 0.0;

    os_mutex_init(&angles->access);
    os_semaphore_init(&angles->measurement_ready, 0);
}

void beacon_angles_update_timestamp(beacon_angles_t *angles,
                                    enum beacon_nb beacon,
                                    uint32_t time)
{
    switch (beacon) {
        case A:
            if((time - angles->_time_a) > angles->_minimal_period){
                angles->_time_a_old = angles->_time_a;
                angles->_time_a = time;
                os_semaphore_signal(&angles->measurement_ready);
            }
            break;
        case B:
            if((time - angles->_time_b) > angles->_minimal_period){
                angles->_time_b_old = angles->_time_b;
                angles->_time_b = time;
            }
            break;
        case C:
            if((time - angles->_time_c) > angles->_minimal_period){
                angles->_time_c_old = angles->_time_c;
                angles->_time_c = time;
            }
            break;
    }
}

void beacon_angles_set_minimal_period(beacon_angles_t *angles, uint32_t period)
{
    angles->_minimal_period = period;
}

int beacon_angles_calculate(beacon_angles_t *angles)
{
    uint32_t my_time_a;
    uint32_t my_time_b;
    uint32_t my_time_c;
    uint32_t my_time_a_old;
    uint32_t my_time_b_old;
    uint32_t my_time_c_old;

    CRITICAL_SECTION_ALLOC()

    CRITICAL_SECTION_ENTER()
    my_time_a     = angles->_time_a;
    my_time_b     = angles->_time_b;
    my_time_c     = angles->_time_c;
    my_time_a_old = angles->_time_a_old;
    my_time_b_old = angles->_time_b_old;
    my_time_c_old = angles->_time_c_old;
    CRITICAL_SECTION_EXIT()

    if((my_time_a - my_time_b_old > my_time_a - my_time_c_old)
        && (my_time_a - my_time_c_old > my_time_a - my_time_a_old)
        && (my_time_a - my_time_a_old > my_time_a - my_time_b)
        && (my_time_a - my_time_b > my_time_a - my_time_c)){

        float period;

        period = my_time_a - my_time_a_old;
        period /= 2 * M_PI;

        os_mutex_take(&angles->access);
        angles->alpha = (my_time_c - my_time_b) / period;
        angles->beta = (my_time_a - my_time_c) / period;
        angles->gamma = (my_time_b - my_time_a_old) / period;
        os_mutex_release(&angles->access);

        return true;
    }

    return false;
}
