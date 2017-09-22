from time import time
from pid import PIDController

class VerticalDrone:
    def __init__(self, axes, histaxes):
        self.axes = axes
        self.histaxes = histaxes

        self.x = 0
        self.setpoint = 0
        self.g = 9.8
        self.reset()
        self.controller = PIDController()

    def init_animation(self):
        self.axes.plot([0], [0], 'x-', lw=2, c='red')

        self.line, = self.axes.plot([], [], 'o-', lw=2)
        self.history, = self.histaxes.plot([0], [0], '-', lw=2)

        self.text = self.axes.text(0.05, 0.8, '', 
                                   transform=self.axes.transAxes)
        return self.line, self.text
        
    def step(self, t):
        elapsed = t - self.lastt 
        self.vz = self.vz + elapsed * (self.az - self.g)
        self.z = self.z + self.vz * elapsed
        self.lastt = t
        self.error = self.z - self.setpoint
        self.interror += self.error
        self.az = az = self.controller.step(t, self.error)
        self.deriv = self.lasterror - self.error
        self.lasterror = self.error

    def animate(self, fig):
        t = time()
        self.step(t)

        self.line.set_data([self.x], [self.z])
        self.text.set_text("z: %.3f\nvz: %.3f\naz: %.3f\nerror: %.3f\nintegrated error: %.3f\nderiv of error: %.3f" % 
                           (self.z, self.vz, self.az, self.error, self.interror, self.deriv))
        self.times.append(t - self.start)
        self.errors.append(self.error)
        self.history.set_data(self.times, self.errors)
        return self.line, self.text, self.history

    def press(self, event):
        if event.key == 'up':
            self.az += 0.1
        elif event.key == 'down':
            self.az -= 0.1
        elif event.key == 'r':
            self.reset()

    def reset(self):
        self.start = time()
        self.lastt = time()
        self.times = []
        self.errors = []
        self.vz = 0
        self.az = 0
        self.z = 1
        self.interror = 0
        self.lasterror = 0

def main():

    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    fig = plt.figure()

    
    ax = fig.add_subplot(121, autoscale_on=False, xlim=(-10, 10), ylim=(-10, 10))
    plt.title("1D Drone")

    ax2 = fig.add_subplot(122, autoscale_on=False, xlim=(0, 100), ylim=(-10, 10))
    plt.title("Error history")

    arm = VerticalDrone(ax, ax2)

    fig.canvas.mpl_connect('key_press_event', arm.press)

    ani = animation.FuncAnimation(fig, arm.animate,
                                  interval=25, blit=True, init_func=arm.init_animation)
    print ani
    plt.show()


if __name__ == "__main__":
    main()
