
#include "CppUTest/TestHarness.h"

extern "C" {
#include "../kalman.h"
#include "../beacon_config.h"
}

#define FLOAT_COMPARE_TOLERANCE (0.0001f)

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
