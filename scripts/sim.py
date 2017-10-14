from time import time
from student_pid_class import PID
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

import yaml



class VerticalDrone:
    def __init__(self, axes, histaxes, pid_params, 
        step_size=0, latency=0, drag_coeff=0, mass=460, sensor_noise=0):

        self.axes = axes
        self.histaxes = histaxes
        self.x = 0
        self.xsp = 0
        self.g = -9.81
        self.step_size = step_size/100.
        self.latency = latency
        self.drag_coeff = drag_coeff
        self.mass = mass
        self.sensor_noise = sensor_noise/100.
        self.pid = PID(pid_params)
        self.start = time()
        self.reset()

    def init_animation(self):
        self.linesp, = self.axes.plot([], [], 'x-', lw=2, c='red')
        self.line, = self.axes.plot([], [], 'o-', lw=2)
        self.history, = self.histaxes.plot([0], [0], '-', lw=2)
        self.text = self.axes.text(0.05, 0.8, '', transform=self.axes.transAxes)
        return self.linesp, self.line, self.text
        
    def step(self, t):
        dt = t - self.lastt     # update time
        self.lastt = t

        self.error = self.z - self.setpoint     # update error
        self.interror += self.error
        self.deriverror = self.lasterror - self.error
        self.lasterror = self.error

        noise = np.random.normal(scale=self.sensor_noise) if self.sensor_noise > 0 else 0
        pwm = self.pid.step(self.error + noise, t)     # calc forces

        self.latent_thrusts.append(self.pwm_to_thrust(pwm))
        thrust = self.latent_thrusts.pop(0)
        drag = - np.sign(self.vz) * self.vz ** 2 * self.drag_coeff
        # TODO - add ground effect 
        
        self.az = self.g + (drag + thrust) / self.mass    # update drone
        self.vz += + self.az * dt
        self.z += self.vz * dt
        if self.z <= 0:
            self.z = 0
            self.vz = 0

    def animate(self, fig):
        t = time() - self.start
        self.step(t)

        self.linesp.set_data([self.xsp], [self.setpoint])
        self.line.set_data([self.x], [self.z])
        self.text.set_text("z: %.3f\nvz: %.3f\naz: %.3f\nerror: %.3f\nintegrated error: %.3f\nderiv of error: %.3f" % 
                           (self.z, self.vz, self.az, self.error, self.interror, self.deriverror))
        self.times.append(t)
        self.errors.append(self.error)
        self.history.set_data(self.times, self.errors)
        return self.linesp, self.line, self.text, self.history

    def press(self, event):
        if event.key == 'up':
            self.setpoint += self.step_size
        elif event.key == 'down':
            if self.setpoint > 0: self.setpoint -= self.step_size
        elif event.key == 'r':
            self.reset()

    def reset(self):
        self.lastt = time() - self.start
        self.times = []
        self.errors = []
        self.latent_thrusts = [1100] * self.latency
        self.z = 0
        self.vz = 0
        self.az = 0
        self.setpoint = 0
        self.interror = 0
        self.lasterror = 0
        self.pid.reset()

    def pwm_to_thrust(self, pwm):
        max_thrust = 420 * 4 * 9.81 # max thrust in newtons
        pwm_min = 1100.
        pwm_max = 1900.
        pwm = max(pwm_min, min(pwm, pwm_max))   # bound the pwm between 1100 and 1900
        throttle_fraction = (pwm - pwm_min) / (pwm_max - pwm_min)   # rescale between 0 and 1
        return throttle_fraction * max_thrust

def main():
    parser = argparse.ArgumentParser(description='''
        This PID simulator simulates gravity acting on a hovering drone.
        Drag, latency, and sensor noise are disabled by default, but can be
        enabled with the flags below.

        For a more realistic sim of a snap-setpoint takeoff to 30cm try 
        
        python sim.py -l 2 -n 0.5 -d 0.02 -s 30
        ''')
    parser.add_argument('-s', '--step', type=int, default=1,
        help='the size of a command step in centimeters')
    parser.add_argument('-d', '--drag', type=float, default=0,
        help='the amount of drag in the simulation')
    parser.add_argument('-m', '--mass', type=int, default=460,
        help='the mass of the simulated drone')
    parser.add_argument('-n', '--noise', type=float, default=0)
    parser.add_argument('-l', '--latency', type=int, default=0,
        help='the number of timesteps of latency (in 40ms increments)')

    args = parser.parse_args()

    with open("pid_terms.yaml", 'r') as stream:
        try:
            yaml_data = yaml.safe_load(stream)
            print yaml_data['z']
        except yaml.YAMLError as exc:
            print exc
            print 'Failed to load PID terms! Exiting.'
            sys.exit(1)

    fig = plt.figure()

    ax = fig.add_subplot(121, autoscale_on=False, xlim=(-10, 10), ylim=(-0, 1))
    plt.title("1D Drone")
    ax2 = fig.add_subplot(122, autoscale_on=False, xlim=(0, 100), ylim=(-1, 1))
    plt.title("Error history")

    sim = VerticalDrone(ax, ax2,
        pid_params=yaml_data['z'],
        step_size=max(args.step, 0),
        latency=max(args.latency, 0),
        drag_coeff=max(args.drag, 0),
        mass=max(args.mass, 0),
        sensor_noise=max(args.noise, 0))

    fig.canvas.mpl_connect('key_press_event', sim.press)
    ani = animation.FuncAnimation(fig, sim.animate,
                                  interval=25, blit=True, init_func=sim.init_animation)
    plt.show()


if __name__ == "__main__":
    main()
