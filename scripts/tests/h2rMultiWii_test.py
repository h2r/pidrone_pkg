import time
from ..h2rMultiWii import MultiWii

def main():
    board = MultiWii("/dev/ttyACM0")
    loop_start_time = time.time()
    loop_count = 0

    while time.time() - loop_start_time < 10.0:
        loop_count += 1
        loop_iteration_start_time = time.time()

        # board.getData(MultiWii.ATTITUDE)
        # print(board.attitude)
        # board.getData(MultiWii.RAW_IMU)

        # board.getData(MultiWii.MOTOR)
        # board.getData(MultiWii.ANALOG)
        board.send_raw_command(8,MultiWii.SET_RAW_RC, [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000])
        board.receiveDataPacket()

        loop_iteration_end_time = time.time()
        loop_iteration_duration = loop_iteration_end_time - loop_iteration_start_time
        loop_frequency = 1 / loop_iteration_duration

        print(f"Loop frequency: {loop_frequency} Hz")

    loop_end_time = time.time()
    total_loop_duration = loop_end_time - loop_start_time
    average_loop_frequency = loop_count / total_loop_duration

    print(f"Average loop frequency: {average_loop_frequency} Hz")

if __name__ == "__main__":
    main()
