#!/usr/bin/env python

import traceback
import collections
import rospy
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from tnex_driver.msg import CruiseControlData


rospy.init_node('mctrl_ccviewer')
matplotlib.use('TkAgg')

class CruiseControlViewer:
    def __init__(self):
        rospy.Subscriber('cruise_control_data', CruiseControlData, self.get_cc_data)

        fig = plt.figure(figsize=(12, 8))
        fig.canvas.manager.set_window_title('Cruise Control Viewer')
        self.ax = plt.subplot(2, 1, 1) # vehicle speed plot
        self.ax2 = plt.subplot(2, 1, 2) # vehicle throttle + brake plot
        self.ax.set_ylim([-1, 15])
        self.ax2.set_ylim([-1.5, 1.5])
        anim_interval = 100 # ms
        self.anim = animation.FuncAnimation(fig, self.animate_plots, repeat=False, init_func=self.init_plots, blit=True, interval=anim_interval)
        num_plot_points = int(1000 / anim_interval * 60) # we want 60 seconds worth of points
        self.speeds = collections.deque(np.zeros(num_plot_points))
        self.throttle_brake_vals = collections.deque(np.zeros(num_plot_points))
        self.target_speed = 0

    def get_cc_data(self, cc_data):
        self.speeds.popleft()
        self.speeds.append(cc_data.speed)
        self.throttle_brake_vals.popleft()
        self.throttle_brake_vals.append(cc_data.throttle_brake)
        self.target_speed = cc_data.target_speed

    def create(self):
        plt.show()

    def destroy(self):
        self.anim.event_source.stop()
        plt.close('all')

    def get_plots(self):
        speed_plot = self.ax.plot(self.speeds, color='r', label='Speed (m/s)')
        target_speed_plot = self.ax.axhline(y=self.target_speed, color='g', label='Target Speed')
        throttle_brake_plot = self.ax2.plot(self.throttle_brake_vals, color='b', label='Throttle + Brake')
        throttle_brake_boundary_plot = self.ax2.axhline(y=0, color='k')
        self.show_legend(self.ax)
        self.show_legend(self.ax2)

        plots = [*speed_plot, target_speed_plot, *throttle_brake_plot, throttle_brake_boundary_plot]
        return plots

    def init_plots(self):
        return self.get_plots()

    def animate_plots(self, i):
        return self.get_plots()

    def show_legend(self, ax):
        # Prevent dup labels in legend
        # See https://stackoverflow.com/a/56253636/6293466
        handles, labels = ax.get_legend_handles_labels()
        unique = [(h, l) for i, (h, l) in enumerate(zip(handles, labels)) if l not in labels[:i]]
        ax.legend(*zip(*unique))


try:
    ccv = CruiseControlViewer()
    rospy.on_shutdown(ccv.destroy)
    ccv.create()
except Exception:
    rospy.logerr(traceback.format_exc())
