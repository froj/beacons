"python wrapper for beacon library"

import ctypes

BEACONS = ctypes.cdll.LoadLibrary("./build/libbeacon.so")

GETX = BEACONS.get_x
GETX.argtypes = []
GETX.restype = ctypes.c_float

GETY = BEACONS.get_y
GETY.argtypes = []
GETY.restype = ctypes.c_float

GETVARX = BEACONS.get_var_x
GETVARX.argtypes = []
GETVARX.restype = ctypes.c_float

GETVARY = BEACONS.get_var_y
GETVARY.argtypes = []
GETVARY.restype = ctypes.c_float

GETCOVXY = BEACONS.get_cov_xy
GETCOVXY.argtypes = []
GETCOVXY.restype = ctypes.c_float

SETUP = BEACONS.setup
SETUP.argtypes = [ctypes.c_float, ctypes.c_float]
SETUP.restype = None

UPDATEMEASCOV = BEACONS.update_meas_cov
UPDATEMEASCOV.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
UPDATEMEASCOV.restype = None

UPDATESTATE = BEACONS.update_state
UPDATESTATE.argtypes = [
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_float,
    ctypes.c_ubyte,
]
UPDATESTATE.restype = None

SET_MAX_ACC = BEACONS.set_max_acc
SET_MAX_ACC.argtypes = [ctypes.c_float]
SET_MAX_ACC.restype = None

SET_PROC_NOISE_PROP = BEACONS.set_proc_noise_prop
SET_PROC_NOISE_PROP.argtypes = [ctypes.c_float]
SET_PROC_NOISE_PROP.restype = None

def setup(pos_x, pos_y):
    "initialize beacon lib"
    SETUP(pos_x, pos_y)

def update_meas_cov(var_x, var_y, cov_xy):
    "update measurement covariance used by kalman filter"
    UPDATEMEASCOV(var_x, var_y, cov_xy)

def next_state(alpha, beta, gamma, delta_t, use_meas):
    "get next state after kalman update"
    UPDATESTATE(alpha, beta, gamma, delta_t, use_meas)
    return (GETX(), GETY(), GETVARX(), GETVARY(), GETCOVXY())

def set_max_acc(acc):
    "set max acc of robot"
    SET_MAX_ACC(acc)

def set_proc_noise_prop(prop):
    "look at the damn method name >.<"
    SET_PROC_NOISE_PROP(prop)
