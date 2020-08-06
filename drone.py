import numpy as np
import math
import scipy.integrate
import time
import datetime
import threading
import multiprocessing as mp
import logging
import json
import sys

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.02 # seconds

class Propeller():
    def __init__(self, prop_dia, prop_pitch, thrust_unit='N'):
        self.dia = prop_dia
        self.pitch = prop_pitch
        self.thrust_unit = thrust_unit
        self.speed = 0 #RPM
        self.thrust = 0

    def set_speed(self,speed):
        self.speed = speed
        # From http://www.electricrcaircraftguy.com/2013/09/propeller-static-dynamic-thrust-equation.html
        self.thrust = 4.392e-8 * self.speed * math.pow(self.dia,3.5)/(math.sqrt(self.pitch))
        self.thrust = self.thrust*(4.23e-4 * self.speed * self.pitch)
        if self.thrust_unit == 'Kg':
            self.thrust = self.thrust*0.101972

class Drone():
    # State space representation: [x y z x_dot y_dot z_dot theta phi gamma theta_dot phi_dot gamma_dot]
    # From Quadcopter Dynamics, Simulation, and Control by Andrew Gibiansky
    def __init__(self,key,ode,config,gravity=9.81,b=0.0245):
        self.key = key
        self.config = config[key]
        self.g = gravity
        self.b = b
        self.ode = ode
        self.time = datetime.datetime.now()

        self.state = np.zeros(12)
        self.state[0:3] = self.config['position']
        self.state[6:9] = self.config['orientation']
        self.props = {
            'm1': Propeller(self.config['prop_size'][0],self.config['prop_size'][1]),
            'm2': Propeller(self.config['prop_size'][0],self.config['prop_size'][1]),
            'm3': Propeller(self.config['prop_size'][0],self.config['prop_size'][1]),
            'm4': Propeller(self.config['prop_size'][0],self.config['prop_size'][1])
        }
        # From Quadrotor Dynamics and Control by Randal Beard
        ixx=((2*self.config['weight']*self.config['r']**2)/5)+(2*self.config['weight']*self.config['L']**2)
        iyy=ixx
        izz=((2*self.config['weight']*self.config['r']**2)/5)+(4*self.config['weight']*self.config['L']**2)
        self.config['I'] = np.array([[ixx,0,0],[0,iyy,0],[0,0,izz]])
        self.config['invI'] = np.linalg.inv(self.config['I'])
        self.run = True

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

    def update(self, dt):
        if self.lock is not None:
            self.lock.acquire()
            self.ode.set_initial_value(self.state,0).set_f_params(self.key)
            self.state = self.ode.integrate(self.ode.t + dt)
            self.lock.release()
        else:
            self.ode.set_initial_value(self.state,0).set_f_params(self.state)
            self.state = self.ode.integrate(self.ode.t + dt)
        self.state[6:9] = self.wrap_angle(self.state[6:9])
        self.state[2] = max(0,self.state[2])

    def set_motor_speeds(self,speeds):
        self.props['m1'].set_speed(speeds[0])
        self.props['m2'].set_speed(speeds[1])
        self.props['m3'].set_speed(speeds[2])
        self.props['m4'].set_speed(speeds[3])

    def get_position(self):
        return self.state[0:3]

    def get_linear_rate(self):
        return self.state[3:6]

    def get_orientation(self):
        return self.state[6:9]

    def get_angular_rate(self):
        return self.state[9:12]

    def get_state(self):
        return self.state

    def set_position(self,position):
        self.state[0:3] = position

    def set_orientation(self,orientation):
        self.state[6:9] = orientation

    def get_time(self):
        return self.time


    def thread_run(self,dt,time_scaling):
        rate = time_scaling*dt
        last_update = self.time
        while(self.run==True):
            time.sleep(0)
            self.time = datetime.datetime.now()
            if (self.time-last_update).total_seconds() > rate:
                self.update(dt)
                last_update = self.time

    def start_thread(self,dt=0.002,time_scaling=1, lock=None):
        self.lock = lock
        self.thread_object = threading.Thread(target=self.thread_run,args=(dt,time_scaling), daemon=True)
        self.thread_object.start()
        

    def stop_thread(self):
        self.run = False



