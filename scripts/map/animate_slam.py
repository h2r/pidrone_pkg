import matplotlib.pyplot as plt
import matplotlib.animation as animation
import ast


def process_slam_output(path):
    """
    reads the output of student slam implementation formatted as the following:
    line1: particle pose list
    line2: landmark poses list
    line3: current features poses list
    ...
    repeated for every camera frame

    returns a list of x data and y data, formatted like:
    x_data = [[[particle poses], [landmark poses], [feature poses]],
              ...,
              [[particle], [lm], [feat]]]]    <-  one time step
    y_ data is the same
    """
    x_data = []
    y_data = []
    frame_data_x = [[], [], []]
    frame_data_y = [[], [], []]
    count = 0
    try:
        fp = open(path, 'r')
        while True:
            line = fp.readline()
            if not line:
                break
            # this line has current features
            if count == 2:
                feature_list = ast.literal_eval(line)
                frame_data_x[2] = [pair[0] for pair in feature_list]
                frame_data_y[2] = [pair[1] for pair in feature_list]
                # append this frame's data and reset frame_data
                x_data.append(frame_data_x)
                y_data.append(frame_data_y)
                frame_data_x = [[], [], []]
                frame_data_y = [[], [], []]
                count = 0
            # this line has landmark poses
            elif count == 1:
                landmark_list = ast.literal_eval(line)
                frame_data_x[1] = [pair[0] for pair in landmark_list]
                frame_data_y[1] = [pair[1] for pair in landmark_list]
                count += 1
            # this line has particle poses
            elif count == 0:
                particle_list = ast.literal_eval(line)
                frame_data_x[0] = [pair[0] for pair in particle_list]
                frame_data_y[0] = [pair[1] for pair in particle_list]
                count += 1
    finally:
        fp.close()
    return x_data, y_data


def animate(count):
    X = x_data[count]
    Y = y_data[count]
    labels = ['particles', 'landmarks', 'current features']
    colors = ['red', 'grey', 'blue']

    ax.clear()
    for i, (x, y) in enumerate(zip(X, Y)):
        ax.scatter(x, y, 5, label=labels[i], color=colors[i])
        plt.xlim(-0.25, 0.75)
        plt.ylim(0, 0.6)
        plt.title("Simultaneous Localization and Mapping on the PiDrone")

    ax.legend(fontsize='medium')


fig = plt.figure()
ax = fig.add_subplot(111)

x_data, y_data = process_slam_output('pose_data.txt')

ani = animation.FuncAnimation(fig, animate, frames=len(x_data), interval=10, blit=False, repeat=False)

ani.save('name2.mp4', fps=40, extra_args=['-vcodec', 'libx264'])

plt.show()

