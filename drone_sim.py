import drone
import scipy.integrate

## ======================
## Detached drone process
## ======================

def run(drone,dt,time_scaling):
    '''Only use when running as separate process'''
    rate = time_scaling*dt
    last_update = self.time
    while(self.run==True):
        time.sleep(0)
        self.time = datetime.datetime.now()
        if (self.time-last_update).total_seconds() > rate:
            self.update(dt)
            last_update = self.time

def start_message_process(self, rcv_msg_queue):
    read_msgs_p = mp.Process(target=receive_msg, args=(rcv_msg_queue,))
    p.start()

def receive_msg(self, rcv_msg_queue):
    '''Read message from stdin'''
    msg = sys.stdin.readline()
    rcv_msg_queue.put(msg)

def state_dot(time, state):
    state_dot = np.zeros(12)
    # The velocities(t+1 x_dots equal the t x_dots)
    state_dot[0] = self.state[3]
    state_dot[1] = self.state[4]
    state_dot[2] = self.state[5]
    # The acceleration
    x_dotdot = np.array([0,0,-self.config['weight']*self.g]) + \
        np.dot(self.rotation_matrix(self.state[6:9]), \
            np.array([0,0,( \
                self.props['m1'].thrust + \
                self.props['m2'].thrust + \
                self.props['m3'].thrust + \
                self.props['m4'].thrust)] \
            ) \
        )/self.config['weight']
    state_dot[3] = x_dotdot[0]
    state_dot[4] = x_dotdot[1]
    state_dot[5] = x_dotdot[2]
    # The angular rates(t+1 theta_dots equal the t theta_dots)
    state_dot[6] = self.state[9]
    state_dot[7] = self.state[10]
    state_dot[8] = self.state[11]
    # The angular accelerations
    omega = self.state[9:12]
    tau = np.array([ \
        self.config['L']*(self.props['m1'].thrust-self.props['m3'].thrust), \
        self.config['L']*(self.props['m2'].thrust-self.props['m4'].thrust), \
        self.b*(\
            self.props['m1'].thrust-\
            self.props['m2'].thrust+\
            self.props['m3'].thrust-\
            self.props['m4'].thrust)])
    omega_dot = np.dot(self.config['invI'], (tau - np.cross(omega, np.dot(self.config['I'],omega))))
    state_dot[9] = omega_dot[0]
    state_dot[10] = omega_dot[1]
    state_dot[11] = omega_dot[2]
    return state_dot

def parse_args():
    parser = argparse.ArgumentParser(description="Drone Process")
    parser.add_argument("--id", help='Drone id, a unique string identifier', default='q')
    parser.add_argument("--config", help='Drone config, path to a json file defining drone characteristics', default='quad_sim/quadcopters.json')
    parser.add_argument("--time_scale", type=float, default=-0.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    config = json.load(args.config)
    ode = scipy.integrate.ode(state_dot).set_integrator('vode',nsteps=500,method='bdf')
    drone = Drone(args.id, config, ode)
    run(drone, QUAD_DYNAMICS_UPDATE, TIME_SCALING)