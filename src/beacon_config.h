
#ifndef BEACON_CONFIG
#define BEACON_CONFIG

#define KALMAN_INIT_POS_X   (0.0f)  // [m]
#define KALMAN_INIT_POS_Y   (0.0f)  // [m]
#define KALMAN_INIT_POS_VAR (1.0f)  // [m^2]

#define KALMAN_TRANS_FREQ    (50)        // [Hz]

#define BEACON_POS_A    {3.0f, 1.0f}    // {[m], [m]}
#define BEACON_POS_B    {0.0f, 2.0f}    // {[m], [m]}
#define BEACON_POS_C    {0.0f, 0.0f}    // {[m], [m]}

#define MAX_ACC (1.0f)  // [m/s/s]
#define PROC_NOISE_PROP (0.0625f)

#define MEAS_VAR_X (0.05f * 0.05f)  // [m^2]
#define MEAS_VAR_Y (0.05f * 0.05f)  // [m^2]
#define MEAS_COV_XY (0.0f)

#define OUTPUT_FREQ     (10)    // [Hz]

#endif
