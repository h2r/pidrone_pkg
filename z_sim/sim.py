from time import time
from pid_class import PID
from numpy import sign

class VerticalDrone:
    def __init__(self, axes, histaxes):
        self.axes = axes
        self.histaxes = histaxes
        self.x = 0

        self.setpoint = 0
        self.drag_coeff = 0.05
        self.g = -9.81
        self.mass = 460
        self.pid = PID(50.,40.,100.)
        self.reset()

    def init_animation(self):
        self.axes.plot([0], [0], 'x-', lw=2, c='red')
        self.line, = self.axes.plot([], [], 'o-', lw=2)
        self.history, = self.histaxes.plot([0], [0], '-', lw=2)
        self.text = self.axes.text(0.05, 0.8, '', transform=self.axes.transAxes)
        return self.line, self.text
        
    def step(self, t):
        dt = t - self.lastt     # update time
        self.lastt = t

        self.error = self.z - self.setpoint     # update error
        self.interror += self.error
        self.deriverror = self.lasterror - self.error
        self.lasterror = self.error

        pwm = self.pid.step(self.error, t)     # calc forces
        thrust = self.pwm_to_thrust(pwm)
        drag = - sign(self.vz) * self.vz ** 2 * self.drag_coeff

        self.az = self.g + (drag + thrust) / self.mass    # update drone
        self.vz += + self.az * dt
        self.z += self.vz * dt
        if self.z < 0:
            self.z = 0
            self.vz = 0

    def animate(self, fig):
        t = time() - self.start
        self.step(t)

        self.line.set_data([self.x], [self.z])
        self.text.set_text("z: %.3f\nvz: %.3f\naz: %.3f\nerror: %.3f\nintegrated error: %.3f\nderiv of error: %.3f" % 
                           (self.z, self.vz, self.az, self.error, self.interror, self.deriverror))
        self.times.append(t)
        self.errors.append(self.error)
        self.history.set_data(self.times, self.errors)
        return self.line, self.text, self.history

    def press(self, event):
        if event.key == 'up':
            self.setpoint += 0.05
        elif event.key == 'down':
            if self.setpoint > 0: self.setpoint -= 0.05
        elif event.key == 'r':
            self.reset()

    def reset(self):
        self.start = time()
        self.lastt = 0
        self.times = []
        self.errors = []
        self.vz = 0
        self.az = 0
        self.z = 1
        self.interror = 0
        self.lasterror = 0
        self.pid.reset()

    def pwm_to_thrust(self, pwm):
        max_thrust = 10000. # max thrust in newtons
        pwm_min = 1100.
        pwm_max = 1900.
        pwm = max(pwm_min, min(pwm, pwm_max))   # bound the pwm between 1100 and 1900
        throttle_fraction = (pwm - pwm_min) / (pwm_max - pwm_min)   # rescale between 0 and 1
        return throttle_fraction * max_thrust

def main():

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    fig = plt.figure()

    
    ax = fig.add_subplot(121, autoscale_on=False, xlim=(-10, 10), ylim=(-0, 2))
    plt.title("1D Drone")

    ax2 = fig.add_subplot(122, autoscale_on=False, xlim=(0, 100), ylim=(-2, 2))
    plt.title("Error history")

    arm = VerticalDrone(ax, ax2)

    fig.canvas.mpl_connect('key_press_event', arm.press)

    ani = animation.FuncAnimation(fig, arm.animate,
                                  interval=25, blit=True, init_func=arm.init_animation)
    print ani
    plt.show()


if __name__ == "__main__":
    main()
