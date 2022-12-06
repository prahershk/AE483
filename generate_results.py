import numpy as np
import csv
import json
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def load_hardware_data(filename, t_min_offset=0, t_max_offset=0, only_in_flight=False):
    # load raw data
    with open(filename, 'r') as f:
        data = json.load(f)

    # convert lists to numpy arrays
    for val in data.values():
        for key in val.keys():
            val[key] = np.array(val[key])

    # create an array of times at which to subsample
    t_min = -np.inf
    t_max = np.inf
    for key, val in data.items():
        t_min = max(t_min, val['time'][0])
        t_max = min(t_max, val['time'][-1])
    t_min += t_min_offset * 1000
    t_max -= t_max_offset * 1000
    nt = int(1 + np.floor((t_max - t_min) / 10.))
    t = np.arange(0, 10 * nt, 10) / 1000.
    resampled_data = {'time': t}

    # resample raw data with linear interpolation
    for k, v in data.items():
        f = interp1d((v['time'] - t_min) / 1000., v['data'])
        resampled_data[k] = f(t)
    
    # truncate to times when o_z_des is positive
    if only_in_flight:
        i = []
        for k in ['ae483log.o_z_des', 'ctrltarget.z']:
            if k in resampled_data.keys():
                j = np.argwhere(resampled_data[k] > 0).flatten()
                if len(j) > len(i):
                    i = j
        if len(i) < 2:
            raise Exception(
                'Failed to get "only_in_flight" data.\n' + \
                ' - Did you remember to log "ae483log.o_z_des" and was it ever positive?\n' + \
                ' - Did you remember to log "ctrltarget.z" and was it ever positive?\n'
            )
        for key in resampled_data.keys():
            resampled_data[key] = resampled_data[key][i[0]:i[-1]]
        
    # return the resampled data
    return resampled_data

# flight test data
data = load_hardware_data('hardware_data.json', only_in_flight=False)

# state estimates from custom observer
o_x = data['ae483log.o_x']
o_y = data['ae483log.o_y']

with open('preflight_data.csv', newline='') as csvfile:
    data = list(csv.reader(csvfile))

data = data[1]

x_lim = float(data[1])
y_lim = float(data[2])

expected_x = np.block([np.zeros(10),
                       np.linspace(0, x_lim, 10),
                       np.ones(10),
                       np.flip(np.linspace(0.25 * x_lim, x_lim, 10)),
                       0.25 * x_lim * np.ones(10),
                       np.linspace(0.25 * x_lim, 0.75 * x_lim, 10),
                       0.75 * x_lim * np.ones(10),
                       np.flip(np.linspace(0.5 * x_lim, 0.75 * x_lim, 10))])
expected_y = np.block([np.linspace(0, y_lim, 10),
                       np.ones(10),
                       np.flip(np.linspace(0, y_lim, 10)),
                       np.zeros(10),
                       np.linspace(0, 0.75 * y_lim, 10),
                       0.75 * y_lim * np.ones(10),
                       np.flip(np.linspace(0.25 * y_lim, 0.75 * y_lim, 10)),
                       0.25 * y_lim * np.ones(10)])

plt.plot(o_x, o_y)
plt.plot(expected_x, expected_y, '--')
plt.xlabel('$x$-Position (m)')
plt.ylabel('$y$-Position (m)')
plt.savefig('results.png')