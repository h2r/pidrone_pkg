import subprocess as sp
import numpy as np

class Camera:
    """ Interfaces with raspivid """
    def __init__(self, width = 320, height = 240):
        self.width = width
        self.height = height
        raspividcmd = ['raspivid', '-t', '0', '-w', str(self.width), '-h',
        str(self.height), '-fps', '40', '-md', '4', '-r', '-', '--raw-format', 'yuv', '-o', '/dev/null', '-n',
        '-pf', 'baseline', '-drc', 'off', '-ex', 'fixedfps', '-fl']
        self.stream = sp.Popen(raspividcmd, stdout = sp.PIPE, universal_newlines = True)

    # Generator function that gets the next image
    def getImage(self):
        while True:
            print self.width, self.height
            raw = self.stream.stdout.read(self.width * self.height + (self.width * self.height / 2))[0:self.width * self.height]
            img = np.fromstring(raw, dtype=np.uint8).reshape(self.height, self.width)
            yield img
