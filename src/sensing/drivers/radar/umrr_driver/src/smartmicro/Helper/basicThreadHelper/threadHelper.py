import threading
import logging
import time


class ThreadHelperException(Exception):
    pass


class ThreadHelper(threading.Thread):
    """
    The class provides a frame for the threads used in the framework.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self):
        """
        The function initializes all necessary variables and instances to deal with threads.
        """
        # init parent class
        threading.Thread.__init__(self)
        # init additional instances and variables
        self.mutex          = threading.Lock()
        self.eventWakeup    = threading.Event()
        self.running        = True
        self.shutDownBool   = False
        # bind thread to main process
        self.setDaemon(True)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: suspend                                                                                                #
    # ---------------------------------------------------------------------------------------------------------------- #
    def suspend(self):
        """
        Suspends the thread.
        """
        with self.mutex:
            self.running = False

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: resume                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def resume(self):
        """
        Resumes the thread.
        """
        with self.mutex:
            if self.running is not True:
                self.running = True
                self.eventWakeup.set()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: shutDown                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def shutDown(self):
        """
        Shut down the thread.
        """
        with self.mutex:
            self.shutDownBool = True
        if self.running is not True:
            self.resume()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: run                                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def run(self):
        """
        The default run function.
        """
        logging.debug("Start the default thread")

        while self.shutDownBool is not True:

            if self.running:
                # raise ThreadHelperException("The thread use the default run function. Implement a run function in the"
                #                            " derived class")
                logging.debug("Default thread executed")
                time.sleep(0.05)
            else:
                logging.debug('Default thread wait')
                self.eventWakeup.wait()
                logging.debug('Default thread resumed')

        logging.debug("Shut down the default thread")