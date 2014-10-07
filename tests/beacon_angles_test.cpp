#include "CppUTest/TestHarness.h"
#include <stdio.h>

extern "C" {
#include "../src/beacon_angles.h"
#include <math.h>
}


TEST_GROUP(BeaconAnglesTestGroup)
{
    beacon_angles_t angles;

    void setup(void)
    {
        beacon_angles_init(&angles);
    }
};

TEST(BeaconAnglesTestGroup, CanInitToZero)
{
    CHECK_EQUAL(0, angles._time_a);
    CHECK_EQUAL(0, angles._time_b);
    CHECK_EQUAL(0, angles._time_c);
    CHECK_EQUAL(0, angles._time_a_old);
    CHECK_EQUAL(0, angles._time_b_old);
    CHECK_EQUAL(0, angles._time_c_old);
    CHECK_EQUAL(0, angles._minimal_period);
    CHECK_EQUAL(0, angles.alpha);
    CHECK_EQUAL(0, angles.beta);
    CHECK_EQUAL(0, angles.gamma);
    CHECK_FALSE(os_semaphore_try(&angles.measurement_ready));
}

TEST(BeaconAnglesTestGroup, CanSetMinimalPeriod)
{
    uint32_t minimal_period = 50000;
    beacon_angles_set_minimal_period(&angles, minimal_period);
    CHECK_EQUAL(minimal_period, angles._minimal_period);
}

TEST(BeaconAnglesTestGroup, CanUpdateTimestamp)
{
    uint32_t new_timestamp_a = 20000;
    uint32_t old_timestamp_a = angles._time_a;
    beacon_angles_update_timestamp(&angles, A, new_timestamp_a);
    CHECK_EQUAL(new_timestamp_a, angles._time_a);
    CHECK_EQUAL(old_timestamp_a, angles._time_a_old);

    uint32_t new_timestamp_b = 20000;
    uint32_t old_timestamp_b = angles._time_b;
    beacon_angles_update_timestamp(&angles, B, new_timestamp_b);
    CHECK_EQUAL(new_timestamp_b, angles._time_b);
    CHECK_EQUAL(old_timestamp_b, angles._time_b_old);

    uint32_t new_timestamp_c = 20000;
    uint32_t old_timestamp_c = angles._time_c;
    beacon_angles_update_timestamp(&angles, C, new_timestamp_c);
    CHECK_EQUAL(new_timestamp_c, angles._time_c);
    CHECK_EQUAL(old_timestamp_c, angles._time_c_old);
}

TEST(BeaconAnglesTestGroup, CanDetectBadEdge)
{
    uint32_t good_edge = 30000;
    uint32_t minimal_period = 20000;
    uint32_t bad_edge = good_edge + minimal_period - 1;
    beacon_angles_set_minimal_period(&angles, minimal_period);

    beacon_angles_update_timestamp(&angles, A, good_edge);
    beacon_angles_update_timestamp(&angles, A, bad_edge);
    CHECK_EQUAL(good_edge, angles._time_a);

    beacon_angles_update_timestamp(&angles, B, good_edge);
    beacon_angles_update_timestamp(&angles, B, bad_edge);
    CHECK_EQUAL(good_edge, angles._time_b);

    beacon_angles_update_timestamp(&angles, C, good_edge);
    beacon_angles_update_timestamp(&angles, C, bad_edge);
    CHECK_EQUAL(good_edge, angles._time_c);
}

TEST(BeaconAnglesTestGroup, CanSignalMesurementReady)
{
    beacon_angles_update_timestamp(&angles, B, 10000);
    beacon_angles_update_timestamp(&angles, C, 20000);
    CHECK_FALSE(os_semaphore_try(&angles.measurement_ready));
    beacon_angles_update_timestamp(&angles, A, 30000);
    CHECK(os_semaphore_try(&angles.measurement_ready));
    CHECK_FALSE(os_semaphore_try(&angles.measurement_ready));
}

TEST(BeaconAnglesTestGroup, CanCalculateAngles)
{
    float alpha = 120 * M_PI / 180;
    float beta  = 120 * M_PI / 180;
    float gamma = 120 * M_PI / 180;

    uint32_t time_offset = 20000;
    uint32_t period = 60000;

    beacon_angles_update_timestamp(&angles, B, 0);
    beacon_angles_update_timestamp(&angles, C, time_offset / 2);
    beacon_angles_update_timestamp(&angles, A, time_offset);
    beacon_angles_update_timestamp(&angles, B, time_offset +
                                               period * gamma / M_PI / 2);
    beacon_angles_update_timestamp(&angles, C, time_offset +
                                               period * (gamma + beta) / M_PI / 2);
    beacon_angles_update_timestamp(&angles, A, time_offset + period);

    CHECK_TRUE(os_semaphore_try(&angles.measurement_ready));
    CHECK_TRUE(beacon_angles_calculate(&angles));

    DOUBLES_EQUAL(alpha, angles.alpha, M_PI / 1800);
    DOUBLES_EQUAL(beta, angles.beta, M_PI / 1800);
    DOUBLES_EQUAL(gamma, angles.gamma, M_PI / 1800);
}

TEST(BeaconAnglesTestGroup, CanDetectMissingBeacon)
{

    beacon_angles_update_timestamp(&angles, A, -20);
    beacon_angles_update_timestamp(&angles, B,   0);
    beacon_angles_update_timestamp(&angles, C, -50);
    beacon_angles_update_timestamp(&angles, A,  80);

    CHECK_TRUE(os_semaphore_try(&angles.measurement_ready));
    CHECK_FALSE(beacon_angles_calculate(&angles));
}
