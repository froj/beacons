
#include "CppUTest/TestHarness.h"

extern "C" {
#include "../src/kalman.h"
#include "../src/beacon_config.h"
}

#define FLOAT_COMPARE_TOLERANCE (0.0001f)

TEST_GROUP(KalmanBadInput)
{
    void setup(void)
    {

    }

    void teardown(void)
    {

    }
};

TEST(KalmanBadInput, initHandleNULL)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;
    CHECK(kalman_init(NULL, &init_pos) == 0);
}

TEST(KalmanBadInput, initConfigNULL)
{
    kalman_robot_handle_t handle;
    CHECK(kalman_init(&handle, NULL) == 0);
}

TEST(KalmanBadInput, initAllNULL)
{
    CHECK(kalman_init(NULL, NULL) == 0);
}

TEST(KalmanBadInput, updateMeasCovHandleNULL)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    CHECK(kalman_update_measurement_covariance(NULL, 0.0f, 0.0f, 0.0f) == 0);
}

TEST(KalmanBadInput, updateHandleNULL)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    position_t meas = {init_pos.x, init_pos.y};

    float delta_t = 1.0f;

    robot_pos_t dest;

    CHECK(kalman_update(NULL, &meas, delta_t, &dest) == 0);
}

TEST(KalmanBadInput, updateDestNull)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    position_t meas = {init_pos.x, init_pos.y};

    float delta_t = 1.0f;

    robot_pos_t dest;

    CHECK(kalman_update(&handle, &meas, delta_t, NULL) == 0);
}

TEST(KalmanBadInput, updateTimeNegative)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    position_t meas = {init_pos.x, init_pos.y};

    float delta_t = -1.0f;

    robot_pos_t dest;

    CHECK(kalman_update(&handle, &meas, delta_t, &dest) == 0);
}

TEST(KalmanBadInput, updateMeasNullSuccess)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    position_t meas = {init_pos.x, init_pos.y};

    float delta_t = 1.0f;

    robot_pos_t dest;

    CHECK(kalman_update(&handle, NULL, delta_t, &dest) == 1);
}

TEST(KalmanBadInput, updateTimeZeroSuccess)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    position_t meas = {init_pos.x, init_pos.y};

    float delta_t = 0.0f;

    robot_pos_t dest;

    CHECK(kalman_update(&handle, &meas, delta_t, &dest) == 1);
}

TEST_GROUP(KalmanInitializationTestGroup)
{
    void setup(void)
    {
    }

    void teardown(void)
    {
    }
};

TEST(KalmanInitializationTestGroup, kalman_init)
{
    robot_pos_t init_pos;
    init_pos.x = 0.0f;
    init_pos.y = 0.0f;
    init_pos.var_x = 1.0f;
    init_pos.var_y = 1.0f;
    init_pos.cov_xy = 0.0f;

    kalman_robot_handle_t handle;

    kalman_init(&handle, &init_pos);

    // test initialized state
    DOUBLES_EQUAL(init_pos.x, handle._state._x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.y, handle._state._y, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, handle._state._v_x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, handle._state._v_y, FLOAT_COMPARE_TOLERANCE);

    // test initialized covariance matrix
    matrix2d_t to_test = handle._state_covariance._cov_a;
    DOUBLES_EQUAL(init_pos.var_x, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.var_y, to_test._d, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.cov_xy, to_test._b, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.cov_xy, to_test._c, FLOAT_COMPARE_TOLERANCE);
    to_test = handle._state_covariance._cov_b;
    DOUBLES_EQUAL(0.0f, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._b, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._c, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._d, FLOAT_COMPARE_TOLERANCE);
    to_test = handle._state_covariance._cov_c;
    DOUBLES_EQUAL(0.0f, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._b, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._c, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._d, FLOAT_COMPARE_TOLERANCE);
    to_test = handle._state_covariance._cov_d;
    DOUBLES_EQUAL(0.0f, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._b, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._c, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(0.0f, to_test._d, FLOAT_COMPARE_TOLERANCE);

    // test initialized measurement covariance
    to_test = handle._measurement_covariance;
    DOUBLES_EQUAL(MEAS_VAR_X, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(MEAS_VAR_Y, to_test._d, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(MEAS_COV_XY, to_test._b, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(MEAS_COV_XY, to_test._c, FLOAT_COMPARE_TOLERANCE);
}

TEST_GROUP(KalmanMeasurementCovUpdate)
{
    kalman_robot_handle_t handle;
    void setup(void)
    {
        robot_pos_t init_pos;
        init_pos.x = 0.0f;
        init_pos.y = 0.0f;
        init_pos.var_x = 1.0f;
        init_pos.var_y = 1.0f;
        init_pos.cov_xy = 0.0f;

        kalman_init(&handle, &init_pos);
    }

    void teardown(void)
    {

    }
};

TEST(KalmanMeasurementCovUpdate, kalman_update_measurement_covariance)
{
    float new_var_x = 0.1f;
    float new_var_y = 0.5f;
    float new_cov_xy = 0.7f;

    kalman_update_measurement_covariance(
            &handle,
            new_var_x,
            new_var_y,
            new_cov_xy);

    // values should now equal new values
    matrix2d_t to_test = handle._measurement_covariance;
    DOUBLES_EQUAL(new_var_x, to_test._a, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(new_var_y, to_test._d, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(new_cov_xy, to_test._c, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(new_cov_xy, to_test._b, FLOAT_COMPARE_TOLERANCE);
}

TEST_GROUP(KalmanUpdate)
{
    robot_pos_t init_pos;
    kalman_robot_handle_t handle;

    void setup(void)
    {
        init_pos.x = 0.0f;
        init_pos.y = 0.0f;
        init_pos.var_x = 1.0f;
        init_pos.var_y = 1.0f;
        init_pos.cov_xy = 0.0f;

        kalman_init(&handle, &init_pos);
    }

    void teardown(void)
    {

    }
};

TEST(KalmanUpdate, identity)
{
    // suppose measurements are exact
    kalman_update_measurement_covariance(&handle, 0.0f, 0.0f, 0.0f);

    // suppose measurement is equal to position
    position_t meas = {init_pos.x, init_pos.y};

    // suppose no time has passed
    float delta_t = 0.0f;

    robot_pos_t dest;

    kalman_update(&handle, &meas, delta_t, &dest);

    // then kalman operation is identity operation
    DOUBLES_EQUAL(init_pos.x, dest.x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.y, dest.y, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.var_x, dest.var_x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.var_y, dest.var_y, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.cov_xy, dest.cov_xy, FLOAT_COMPARE_TOLERANCE);
}

TEST(KalmanUpdate, noTime1)
{
    // measurements are not exact anymore
    kalman_update_measurement_covariance(&handle, 1.0f, 1.0f, 1.0f);

    // measurement is equal to position
    position_t meas = {init_pos.x, init_pos.y};

    // no time has passed
    float delta_t = 0.0f;

    robot_pos_t dest;
    kalman_update(&handle, &meas, delta_t, &dest);
    // estimated position remains the same
    DOUBLES_EQUAL(init_pos.x, dest.x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.y, dest.y, FLOAT_COMPARE_TOLERANCE);
    // estimated covariances change
    float one_third = (1.0f)/(3.0f);
    DOUBLES_EQUAL(init_pos.var_x, one_third, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.var_y, one_third, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.cov_xy, one_third, FLOAT_COMPARE_TOLERANCE);
}

TEST(KalmanUpdate, noTime2)
{
    // measurements are not exact anymore
    kalman_update_measurement_covariance(&handle, 1.0f, 1.0f, 1.0f);

    // measurement is slightly off --> y = (0.1, 0.1)^T
    position_t meas = {init_pos.x + 0.1f, init_pos.y + 0.1f};

    // no time has passed
    float delta_t = 0.0f;

    robot_pos_t dest;
    kalman_update(&handle, &meas, delta_t, &dest);
    // estimated position changes
    float one_over_thirty = (1.0f)/(30.0f);
    DOUBLES_EQUAL(init_pos.x + one_over_thirty, dest.x, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.y + one_over_thirty, dest.y, FLOAT_COMPARE_TOLERANCE);
    // estimated covariances change -- same as in previous test
    float one_third = (1.0f)/(3.0f);
    DOUBLES_EQUAL(init_pos.var_x, one_third, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.var_y, one_third, FLOAT_COMPARE_TOLERANCE);
    DOUBLES_EQUAL(init_pos.cov_xy, one_third, FLOAT_COMPARE_TOLERANCE);
}
