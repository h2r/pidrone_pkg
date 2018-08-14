"""
Thread Queue

We will use this class to manage threads in slam_helper, we should be able to simply call add_thread on any
map_update thread we want to happen
"""


class ThreadQueue:
    """
    A class to manage threads so there is at most one at a time
    """
    def __init__(self):
        self.queue = []

    def add_thread(self, thread):
        """
        manages threads so that there is only a single thread running at a time

        :param thread: a thread to enqueue if there are no current threads
        """
        if len(self.queue) == 1:
            old_thread = self.queue[0]
            if not old_thread.isAlive():
                self.queue.remove(old_thread)
                self.queue.append(thread)
                thread.start()
        elif len(self.queue) == 0:
            self.queue.append(thread)
            thread.start()












