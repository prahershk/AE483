import logging
import time
import json
import numpy as np
import cflib.crtp
import csv
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/30/2M/E7E7E7E7E7'

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # State estimates (custom observer)
    'ae483log.o_x',
    'ae483log.o_y',
    'ae483log.o_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Setpoint (default controller)
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Setpoint (custom controller)
    'ae483log.o_x_des',
    'ae483log.o_y_des',
    'ae483log.o_z_des',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
    # Loco positions
    'ae483.log.x0',
    'ae483.log.x1',
    'ae483.log.x2',
    'ae483.log.x3',
    'ae483.log.x4',
    'ae483.log.x5',
    'ae483.log.x6',
    'ae483.log.x7',
    'ae483.log.z0',
    'ae483.log.z1',
    'ae483.log.z2',
    'ae483.log.z3',
    'ae483.log.z4',
    'ae483.log.z5',
    'ae483.log.z6',
    'ae483.log.z7',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=False):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.fully_connected.add_callback(self.fully_connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def connected(self, uri):
        print(f'Connected to {uri}')
    
    def fully_connected(self, uri):
        print(f'Fully connected to {uri}')
        self.is_fully_connected = True

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)

        # Reset the ae483 observer
        self.cf.param.set_value('ae483par.reset_observer', 1)

        # Enable the controller (1 for default controller, 4 for ae483 controller)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 4)
            self.cf.param.set_value('powerDist.motorSetEnable', 1)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)
            self.cf.param.set_value('powerDist.motorSetEnable', 0)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_fully_connected = False

    def log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
    
    def move_smooth(self, p1, p2, yaw, speed):
        print(f'Move smoothly from {p1} to {p2} with yaw {yaw} degrees at {speed} meters / second')
        p1 = np.array(p1)
        p2 = np.array(p2)
        
        # Compute distance from p1 to p2
        distance_from_p1_to_p2 = np.linalg.norm(p2 - p1)
        
        # Compute time it takes to move from p1 to p2 at desired speed
        time_from_p1_to_p2 = distance_from_p1_to_p2 / speed
        
        start_time = time.time()
        while True:
            current_time = time.time()
            
            # Compute what fraction of the distance from p1 to p2 should have
            # been travelled by the current time
            s = (current_time - start_time) / time_from_p1_to_p2
            
            # Compute where the drone should be at the current time, in the
            # coordinates of the world frame
            p = (1 - s) * p1 + s * p2
            
            self.cf.commander.send_position_setpoint(p[0], p[1], p[2], yaw)
            if s >= 1:
                return
            else:
                time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)


if __name__ == '__main__':
    # Initialize everything
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=False, use_observer=False) # <-- FIXME
    while not client.is_fully_connected:
        time.sleep(0.1)

    # Leave time at the start to initialize
    client.stop(1.0)
    
    z_constant = 0.4
    
    with open('preflight_data.csv', newline='') as csvfile:
        data = list(csv.reader(csvfile))

    data = data[1]

    speed = float(data[0])
    x_lim = float(data[1])
    y_lim = float(data[2])
    pattern = data[3]
 
    # If the data sent from the GUI requests Square Pattern
    if pattern == "['Square Pattern Single-Unit']":
        print("Starting Square Search Pattern")
        x_square = [0,0,x_lim,x_lim,x_lim/4,x_lim/4,3*x_lim/4,3*x_lim/4,x_lim/2,x_lim/2]
        y_square = [0,y_lim,y_lim,0,0,3*y_lim/4,3*y_lim/4,y_lim/4,y_lim/4,y_lim/2]
        z_square = np.zeros(len(x_square))+z_constant
    
        client.move(0.0, 0.0, z_constant/3, 0.0, 1.0)
        
        i = 0
        while i < len(x_square):
            if i == len(x_square)-2:
                client.move(x_square[i], y_square[i], z_square[i]/3, 0.0, 1.0)
                break

            if i == 0:
                client.move_smooth([0.0, 0.0, z_constant/3], [x_square[i], y_square[i], z_square[i]], 0.0, 0.2)

            client.move_smooth([x_square[i], y_square[i], z_square[i]], [x_square[i+1], y_square[i+1], z_square[i+1]], 0.0, 0.2)
            client.move(x_square[i+1], y_square[i+1], z_square[i+1], 0.0, 1.0)
            i = i+1
        
    # If the data sent from the GUI requests Sector Pattern
    if pattern == "['Sector Pattern Single-Unit']":
        x_center = x_lim/2
        y_center = y_lim/2
        h = x_lim/2
        k = y_lim/2

        x_sector = np.array([0-x_center,h*np.cos(4*np.pi/3),h*np.cos(np.pi/3),h*np.cos(0),h*np.cos(np.pi),h*np.cos(2*np.pi/3),h*np.cos(5*np.pi/3),h*np.cos(4*np.pi/3)])
        x_sector = x_sector+x_center

        y_sector = np.array([0-y_center,k*np.sin(4*np.pi/3),k*np.sin(np.pi/3),k*np.sin(0),k*np.sin(np.pi),k*np.sin(2*np.pi/3),k*np.sin(5*np.pi/3),k*np.sin(4*np.pi/3)])
        y_sector = y_sector+y_center

        z_sector = np.zeros(len(x_sector))+z_constant

        client.move(0.0, 0.0, z_constant/3, 0.0, 1.0)
        print("Starting Radioactive Search Pattern")
        i = 0
        while i < len(x_sector):
            if i == len(x_sector)-2:
                client.move(x_sector[i], y_sector[i], z_sector[i]/3, 0.0, 1.0)
                break

            if i == 0:
                client.move_smooth([0.0, 0.0, z_constant/3], [x_sector[i], y_sector[i], z_sector[i]], 0.0, 0.2)

            client.move_smooth([x_sector[i], y_sector[i], z_sector[i]], [x_sector[i+1], y_sector[i+1], z_sector[i+1]], 0.0, 0.2)
            client.move(x_sector[i+1], y_sector[i+1], z_sector[i+1], 0.0, 1.0)
            i = i+1

    # If the data sent from the GUI requests Parallel Pattern
    if pattern == "['Parallel Single-Unit Spiral']":
        x_center = x_lim/2
        y_center = y_lim/2
        count = 20
        h = np.linspace(x_lim/2,0,count)
        k = np.linspace(y_lim/2,0,count)

        t = np.linspace(3*np.pi/2, 11*np.pi/2,count)
        
        x_spiral = []
        y_spiral = []
        
        i = 0
        while i < len(t):
            x_spiral.append(h[i]*np.cos(t[i]))
            y_spiral.append(k[i]*np.sin(t[i]))
            i = i+1
        
        x_spiral = np.array([x_spiral])+x_center
        y_spiral = np.array([y_spiral])+y_center
        z_spiral = np.zeros(len(x_spiral))+z_constant

        client.move(0.0, 0.0, z_constant/3, 0.0, 1.0)

        print("Starting Spiral Search Pattern")
        i = 0
        while i < len(x_spiral):
            if i == len(x_spiral)-2:
                client.move(x_spiral[i], y_spiral[i], z_spiral[i]/3, 0.0, 1.0)
                break

            if i == 0:
                client.move_smooth([0.0, 0.0, z_constant/3], [x_spiral[i], y_spiral[i], z_spiral[i]], 0.0, 0.2)

            client.move_smooth([x_spiral[i], y_spiral[i], z_spiral[i]], [x_spiral[i+1], y_spiral[i+1], z_spiral[i+1]], 0.0, 0.2)
            client.move(x_spiral[i+1], y_spiral[i+1], z_spiral[i+1], 0.0, 0.2)
            i = i+1
       
    """
    print(pattern)
    if pattern == "['Square Pattern Single-Unit']":
        # For move tests
        # - take off and hover (with zero yaw)
        client.move(0.0, 0.0, 0.15, 0.0, 1.0)
        client.move_smooth([0.0, 0.0, 0.15], [0.0, 0.0, 0.5], 0.0, 0.2)
        client.move(0.0, 0.0, 0.5, 0.0, 1.0)
        # - fly in a square five times (with a pause at each corner)
        num_squares = 1
        for i in range(num_squares):
            client.move_smooth([0.0, 0.0, 0.5], [0.5, 0.0, 0.5], 0.0, 0.2)
            client.move(0.5, 0.0, 0.5, 0.0, 1.0)
            client.move_smooth([0.5, 0.0, 0.5], [0.5, 0.5, 0.5], 0.0, 0.2)
            client.move(0.5, 0.5, 0.5, 0.0, 1.0)
            client.move_smooth([0.5, 0.5, 0.5], [0.0, 0.5, 0.5], 0.0, 0.2)
            client.move(0.0, 0.5, 0.5, 0.0, 1.0)
            client.move_smooth([0.0, 0.5, 0.5], [0.0, 0.0, 0.5], 0.0, 0.2)
            client.move(0.0, 0.0, 0.5, 0.0, 1.0)
        # - go back to hover (with zero yaw) and prepare to land
        client.move_smooth([0.0, 0.0, 0.5], [0.0, 0.0, 0.15], 0.0, 0.2)
        client.move(0.0, 0.0, 0.15, 0.0, 1.0)
    """

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('hardware_data.json')
