import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as Axes3D
import sys
import time

class GUI():
    # 'quad_list' is a dictionary of format: quad_list = {'quad_1_name':{'position':quad_1_position,'orientation':quad_1_orientation,'arm_span':quad_1_arm_span}, ...}
    def __init__(self, drones, swarm):
        self.drones = {key: {} for key in drones}
        self.swarm = swarm
        self.fig = None
        self.anim = None
        self.ax = None

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

    def init_plot(self):
        for key in self.drones:
            self.drones[key]['l1'], = self.ax.plot([],[],[],color='blue',linewidth=3,antialiased=False)
            self.drones[key]['l2'], = self.ax.plot([],[],[],color='red',linewidth=3,antialiased=False)
            self.drones[key]['hub'], = self.ax.plot([],[],[],marker='o',color='green', markersize=6,antialiased=False)
        plot_items = []
        plot_items.extend([self.drones[key]['l1'] for key in self.drones])
        plot_items.extend([self.drones[key]['l2'] for key in self.drones])
        plot_items.extend([self.drones[key]['hub'] for key in self.drones])
        return plot_items

    def update(self, i):
        for key in self.drones:
            R = self.rotation_matrix(self.swarm.get_orientation(key))
            L = self.swarm.drones[key].config['L']
            points = np.array([ [-L,0,0], [L,0,0], [0,-L,0], [0,L,0], [0,0,0], [0,0,0] ]).T
            points = np.dot(R,points)
            points[0,:] += self.swarm.get_position(key)[0]
            points[1,:] += self.swarm.get_position(key)[1]
            points[2,:] += self.swarm.get_position(key)[2]
            self.drones[key]['l1'].set_data(points[0,0:2],points[1,0:2])
            self.drones[key]['l1'].set_3d_properties(points[2,0:2])
            self.drones[key]['l2'].set_data(points[0,2:4],points[1,2:4])
            self.drones[key]['l2'].set_3d_properties(points[2,2:4])
            self.drones[key]['hub'].set_data(points[0,5],points[1,5])
            self.drones[key]['hub'].set_3d_properties(points[2,5])
        plot_items = []
        plot_items.extend([self.drones[key]['l1'] for key in self.drones])
        plot_items.extend([self.drones[key]['l2'] for key in self.drones])
        plot_items.extend([self.drones[key]['hub'] for key in self.drones])
        return plot_items

    def animate(self, frames=200, interval=20):
        self.fig = plt.figure()
        self.ax = Axes3D.Axes3D(self.fig)
        self.ax.set_xlim3d([-2.0, 2.0])
        self.ax.set_xlabel('X')
        self.ax.set_ylim3d([-2.0, 2.0])
        self.ax.set_ylabel('Y')
        self.ax.set_zlim3d([0, 5.0])
        self.ax.set_zlabel('Z')
        self.ax.set_title('Quadcopter Simulation')
        self.fig.canvas.mpl_connect('key_press_event', self.keypress_routine)
        self.anim = animation.FuncAnimation(self.fig, self.update, init_func=self.init_plot,frames=frames,interval=interval, repeat=False)
        plt.show()

    def keypress_routine(self,event):
        sys.stdout.flush()
        if event.key == 'x':
            y = list(self.ax.get_ylim3d())
            y[0] += 0.2
            y[1] += 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'w':
            y = list(self.ax.get_ylim3d())
            y[0] -= 0.2
            y[1] -= 0.2
            self.ax.set_ylim3d(y)
        elif event.key == 'd':
            x = list(self.ax.get_xlim3d())
            x[0] += 0.2
            x[1] += 0.2
            self.ax.set_xlim3d(x)
        elif event.key == 'a':
            x = list(self.ax.get_xlim3d())
            x[0] -= 0.2
            x[1] -= 0.2
            self.ax.set_xlim3d(x)
