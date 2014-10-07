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
