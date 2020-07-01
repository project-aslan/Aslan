import os
import time
import logging
import datetime
from copy import deepcopy
from smartmicro.Helper.basicThreadHelper.threadHelper import ThreadHelper


class canReceive(ThreadHelper):
    """
    This is an internal class and should not be used by the user

    The user should use the communicationModule which instantiates a canReceive object if the communication module
    is configured to use can.

    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, canDevice, exportData=False):
        """
        The function initializes the basic components for receiving the can messages.

        Parameters
        ----------
        canDevice   : canDevice instance
            instance of the used can device
        exportData  : Bool
            assign recording of received data ( True = activate | False = deactivate )
        """
        # init parent class
        ThreadHelper.__init__(self)
        # set basic values
        self.queue          = []
        self.canDevice      = canDevice
        self.queueDict      = dict()
        self.exportData     = exportData
        self.recordingQueue = None
        # check export condition
        if self.exportData:
            # provide output
            self.__provideDataOutput()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __provideDataOutput                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __provideDataOutput(self):
        """
        The file provides an output file for received data.

        Returns
        -------

        """
        # get current date
        date = datetime.date.today()
        # get current time
        timestamp = datetime.datetime.now()
        # provide output file name
        fileName = 'CanRecv' + '_{}'.format(date) + '_{}-{}-{}'.format( timestamp.hour, timestamp.minute,
                                                                        timestamp.second ) + '.txt'
        # provide output file
        self.completeName = os.path.join('C:\\CAN\\', fileName)
        self.outputFile   = open(self.completeName, mode='x', encoding='utf-8')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: registerQueue                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def registerQueue(self, queue, canId, idRange=0):
        """
        The function provides the possibility to register queues for dedicated identifier. These identifier can be
        defined outside of the thread.

        Parameters
        ----------
        queue   : Queue
            queue used as instance for the can identifier
        canId   : integer
            start identifier for the current queue
        idRange : integer
            range of identifier for the queue

        Returns
        -------

        """
        with self.mutex:
            # register the queue for all identifier
            for currId in range(canId, canId+idRange + 1):
                # check if list of queues is available or not
                if str(currId) in self.queueDict.keys():
                    # register queue
                    self.queueDict[str(currId)].append(queue)
                else:
                    # provide list and register queue
                    self.queueDict[str(currId)] = list()
                    self.queueDict[str(currId)].append(queue)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: registerRecordingQueue                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def registerRecordingQueue(self, queue):
        """
        The function provides a queue to get all received data.

        Parameters
        ----------
        queue   : Queue
            queue with all received data

        Returns
        -------

        """
        self.recordingQueue = queue

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: run                                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def run(self):
        """
        The main function to receive data from the can device.

        Returns
        -------

        """
        # save log info
        logging.info("CanRecv Thread Start")
        # message data set
        msg = dict()
        # run thread until thread is closed
        while self.shutDownBool is not True:
            # check if thread is suspended
            if self.running:
                # get data from registered can device
                receivedData = self.canDevice.receiveData()
                # save data if necessary
                if self.exportData:
                    self.outputFile.writelines('{} CAN Message:{} \n' .format(datetime.datetime.now(), receivedData))
                # check if data available
                if receivedData is None:
                    time.sleep(0.005)
                else:
                    # extract data from received data set
                    [canId, dlc, canMessage, channelIdx] = receivedData
                    # write data
                    with self.mutex:
                        # write data to dictionary
                        msg['canID']        = canId
                        msg['dlc']          = dlc
                        msg['data']         = canMessage[:dlc]
                        msg['channelIndex'] = channelIdx
                        msg['clientId']     = 0
                        # check if identifier is registered
                        if str(canId) in self.queueDict:
                            # log can data info
                            logging.debug('Put the message in the queue:{}'.format(receivedData))
                            # copy data to queues
                            for listEntry in self.queueDict[str(canId)]:
                                listEntry.put(deepcopy(msg))
                        # check recording condition
                        if self.recordingQueue is not None:
                            # copy data to recording queue
                            self.recordingQueue.put(deepcopy(msg))
            else:
                # wait until thread is resumed
                logging.info("CanRecv Thread Suspended")
                self.eventWakeup.wait()
                logging.info("CanRecv Thread Resumed")
        # log thread shut down
        logging.info("CanRecv Thread Shut Down")
