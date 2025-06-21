import numpy as np
import matplotlib.pyplot as plt
import time

MAP_SIZE=200
RESOLUTION=0.1
MAX_RANGE=15.0
LOG_ODDS_FREE=0
LOG_ODDS_FREE=0
LOG_ODDS_OCC=0
LOG_ODDS_MIN=0
LOG_ODDS_MAX=0

#intialize grid
log_odds=np.zeros((MAP_SIZE,MAP_SIZE))

walls = [
    ((0, 0), (10, 0)),
    ((10, 0), (10, 10)),
    ((10, 10), (0, 10)), 
    ((0, 10), (0, 0)),  
    ((3, 3), (7, 3)),
    ((7, 3), (7, 7)),
    ((7, 7), (3, 7)),
    ((3, 7), (3, 3)),
]