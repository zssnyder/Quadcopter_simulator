import drone
import scipy.integrate

logging.basicConfig(level=logging.DEBUG, format='%(relativeCreated)6d %(threadName)s %(message)s')

def parse_args():
    parser = argparse.ArgumentParser(description="Drone Process")
    parser.add_argument("--id", help='Drone id, a unique string identifier', default='q')
    parser.add_argument("--config", help='Drone config, path to a json file defining drone characteristics', default='quad_sim/quadcopters.json')
    parser.add_argument("--time_scale", type=float, default=1.0, help='Time scaling factor. 0.0:fastest,1.0:realtime,>1:slow, ex: --time_scale 0.1')
    parser.add_argument("--quad_update_time", type=float, default=0.002, help='delta time for quadcopter dynamics update(seconds), ex: --quad_update_time 0.002')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    if args.time_scale>=0: TIME_SCALING = args.time_scale
    if args.quad_update_time>0: QUAD_DYNAMICS_UPDATE = args.quad_update_time
    config = json.load(args.config)
    drone = Drone(args.id, config)
    drone.run(QUAD_DYNAMICS_UPDATE, TIME_SCALING)
    drone.wait()
    drone.stop()