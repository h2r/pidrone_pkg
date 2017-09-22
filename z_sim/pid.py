class PIDController:
    def __init__(self):
        self.KP = 50
        self.KI = 0
        self.KD = 500
        self.reset()

    def reset(self):
        self.summed_error = 0.0
        self.last_error = None

    def step(self, t, et):
        self.summed_error += et
        if self.last_error != None:
            det = self.last_error - et
            print "det", det
            ut = (-1 * self.KP * et + 
                  -1 * self.KI * self.summed_error + 
                  1 * self.KD * det)
        else:
            ut = 0
        self.last_error = et

        return ut
    


def main():
    print "hello"



if __name__ == "__main__":
    main()
