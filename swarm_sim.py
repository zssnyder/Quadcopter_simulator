import swarm,swarm_gui,controller
import signal
import sys
import argparse
import json
import logging

# Constants
TIME_SCALING = 1.0 # Any positive number(Smaller is faster). 1.0->Real Time, 0.0->Run as fast as possible
QUAD_DYNAMICS_UPDATE = 0.002 # seconds
CONTROLLER_DYNAMICS_UPDATE = 0.005 # seconds
run = True

logging.basicConfig(level=logging.DEBUG, format='%(relativeCreated)6d %(threadName)s %(message)s')

def Swarm_Formations():
    # Set goals to go to
    FORMATIONS = json.load(open('quad_sim/formations.json'))
    # Define the quadcopters
    QUADCOPTERS=json.load(open('quad_sim/quadcopters.json'))
    # Controller parameters
    CONTROLLER_PARAMETERS = {'Motor_limits':[4000,9000],
                        'Tilt_limits':[-10,10],
                        'Yaw_Control_Limits':[-900,900],
                        'Z_XY_offset':500,
                        'Linear_PID':{'P':[300,300,7000],'I':[0.04,0.04,4.5],'D':[450,450,5000]},
                        'Linear_To_Angular_Scaler':[1,1,0],
                        'Yaw_Rate_Scaler':0.18,
                        'Angular_PID':{'P':[22000,22000,1500],'I':[0,0,1.2],'D':[12000,12000,0]},
                        }
    # Create controller parameters for each quadcopter
    ctrl_params = [CONTROLLER_PARAMETERS.copy() for q in QUADCOPTERS]
    # Catch Ctrl+C to stop threads
    signal.signal(signal.SIGINT, signal_handler)
    # Make objects for quadcopter, gui and controllers
    gui_object = swarm_gui.GUI(drones=QUADCOPTERS)
    s = swarm.Swarm(drones=QUADCOPTERS)
    controllers = [
        controller.Controller_PID_Point2Point( \
            s.get_state, \
            s.get_time, \
            s.set_motor_speeds, \
            params=param, \
            quad_identifier=q) \
        for q,param in zip(QUADCOPTERS, ctrl_params)
    ]
    # ctrl1 = controller.Controller_PID_Point2Point(s.get_state,s.get_time,s.set_motor_speeds,params=CONTROLLER_1_PARAMETERS,quad_identifier='q1')
    # ctrl2 = controller.Controller_PID_Point2Point(s.get_state,s.get_time,s.set_motor_speeds,params=CONTROLLER_2_PARAMETERS,quad_identifier='q2')
    # Start the threads
    s.start_threads(dt=QUAD_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    for ctrl in controllers:
        ctrl.start_thread(update_rate=CONTROLLER_DYNAMICS_UPDATE,time_scaling=TIME_SCALING)
    # Update the GUI while switching between destination poitions
    while(run==True):
        key = input("Which formation would you like to make?\noptions: {}\n:".format([str(key) for key in FORMATIONS.keys()]))
        while key not in FORMATIONS.keys():
            key = input("Did not recognize input. Please select an option below...\noptions: {}\n:".format([str(key) for key in FORMATIONS.keys()]))
        
        # TODO: Write a smart algorithm to set targets of nearest drones
        for q,ctrl in zip(QUADCOPTERS,controllers):
            ctrl.update_target(tuple(FORMATIONS[key][q]))
        
        # TODO: Rewrite gui updates to make them more smooth
        for i in range(300):
            for key in QUADCOPTERS:
                gui_object.drones[key]['position'] = s.get_position(key)
                gui_object.drones[key]['orientation'] = s.get_orientation(key)
            gui_object.update()
    s.stop_threads()
    for ctrl in controllers:
        ctrl.stop_thread()

def parse_args():
    parser = argparse.ArgumentParser(description="Quadcopter Simulator")
    parser.add_argument("--sim", help='single_p2p, multi_p2p or single_velocity', default='formations')
    parser.add_argument("--time_scale", type=float, default=-1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.0, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    parser.add_argument("--controller_update_time", type=float, default=0.0, help='delta time for controller update(seconds), ex: --controller_update_time 0.005')
    return parser.parse_args()

def signal_handler(signal, frame):
    global run
    run = False
    print('Stopping')
    sys.exit(0)

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    if args.controller_update_time>0: CONTROLLER_DYNAMICS_UPDATE = args.controller_update_time
    if args.sim == 'formations':
        Swarm_Formations()