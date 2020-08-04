import drone
import numpy as np
import math
import scipy.integrate
import time
import datetime
import threading

class Swarm():
    # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self,drones):
        self.drones = {}
        self.ode = scipy.integrate.ode(self.state_dot).set_integrator('vode',nsteps=500,method='bdf')
        self.time = datetime.datetime.now()
        self.lock = None
        for key in drones:
            self.drones[key] = drone.Drone(key=key,drone=drones[key],swarm=self)

    def rotation_matrix(self,angles):
        ct = math.cos(angles[0])
        cp = math.cos(angles[1])
        cg = math.cos(angles[2])
        st = math.sin(angles[0])
        sp = math.sin(angles[1])
        sg = math.sin(angles[2])
        R_x = np.array([[1,0,0],[0,ct,-st],[0,st,ct]])
        R_y = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        R_z = np.array([[cg,-sg,0],[sg,cg,0],[0,0,1]])
        R = np.dot(R_z, np.dot( R_y, R_x ))
        return R

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2 * np.pi ) - np.pi )

    def state_dot(self, time, state, key):
        state_dot = np.zeros(12)
        # The velocities(t+1 x_dots equal the t x_dots)
        state_dot[0] = self.drones[key].state[3]
        state_dot[1] = self.drones[key].state[4]
        state_dot[2] = self.drones[key].state[5]
        # The acceleration
        x_dotdot = np.array([0,0,-self.drones[key].config['weight']*self.drones[key].g]) + \
            np.dot(self.rotation_matrix(self.drones[key].state[6:9]), \
                np.array([0,0,( \
                    self.drones[key].props['m1'].thrust + \
                    self.drones[key].props['m2'].thrust + \
                    self.drones[key].props['m3'].thrust + \
                    self.drones[key].props['m4'].thrust)] \
                ) \
            )/self.drones[key].config['weight']
        state_dot[3] = x_dotdot[0]
        state_dot[4] = x_dotdot[1]
        state_dot[5] = x_dotdot[2]
        # The angular rates(t+1 theta_dots equal the t theta_dots)
        state_dot[6] = self.drones[key].state[9]
        state_dot[7] = self.drones[key].state[10]
        state_dot[8] = self.drones[key].state[11]
        # The angular accelerations
        omega = self.drones[key].state[9:12]
        tau = np.array([ \
            self.drones[key].config['L']*(self.drones[key].props['m1'].thrust-self.drones[key].props['m3'].thrust), \
            self.drones[key].config['L']*(self.drones[key].props['m2'].thrust-self.drones[key].props['m4'].thrust), \
            self.drones[key].b*(\
                self.drones[key].props['m1'].thrust-\
                self.drones[key].props['m2'].thrust+\
                self.drones[key].props['m3'].thrust-\
                self.drones[key].props['m4'].thrust)])
        omega_dot = np.dot(self.drones[key].config['invI'], (tau - np.cross(omega, np.dot(self.drones[key].config['I'],omega))))
        state_dot[9] = omega_dot[0]
        state_dot[10] = omega_dot[1]
        state_dot[11] = omega_dot[2]
        return state_dot

    # def update(self, dt):
    #     for key in self.drones:
    #         self.ode.set_initial_value(self.drones[key].state,0).set_f_params(key)
    #         self.drones[key].state = self.ode.integrate(self.ode.t + dt)
    #         self.drones[key].state[6:9] = self.wrap_angle(self.drones[key].state[6:9])
    #         self.drones[key].state[2] = max(0,self.drones[key].state[2])

    def set_motor_speeds(self,drone_key,speeds):
        logger = logging.getLogger()
        logger.debug('Set motor speeds for: {}, speeds: {}'.format(drone_key, speeds))
        self.drones[drone_key].set_motor_speeds(speeds)

    def get_position(self,drone_key):
        return self.drones[drone_key].get_position()

    def get_linear_rate(self,drone_key):
        return self.drones[drone_key].get_linear_rate()

    def get_orientation(self,drone_key):
        return self.drones[drone_key].get_orientation()

    def get_angular_rate(self,drone_key):
        return self.drones[drone_key].get_angular_rate()

    def get_state(self,drone_key):
        return self.drones[drone_key].get_state()

    def set_position(self,drone_key,position):
        self.drones[drone_key].set_position(position)

    def set_orientation(self,drone_key,orientation):
        self.drones[drone_key].set_orientation(orientation)

    def get_time(self):
        return self.time

    # def thread_run(self,dt,time_scaling):
    #     rate = time_scaling*dt
    #     last_update = self.time
    #     while(self.run==True):
    #         time.sleep(0)
    #         self.time = datetime.datetime.now()
    #         if (self.time-last_update).total_seconds() > rate:
    #             self.update(dt)
    #             last_update = self.time

    def start_threads(self,dt=0.002,time_scaling=1):
        self.lock = threading.Lock()
        for key in self.drones:
            self.drones[key].start_thread(dt,time_scaling,self.lock) 

    def stop_threads(self):
        for key in self.drones:
            self.drones[key].stop_thread()
