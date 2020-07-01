import json
import queue
import struct

from copy import deepcopy
from smartmicro.Services.basicCanServices.canService import CanIDService


class CanIDServiceTargetList(CanIDService):
    """
    The current class inidicates the decoding of target messages in general. It provides the possibility to decode every
    target message from header to single frame messages.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, canSpecFile):
        """
        The current function initializes target can ID service. Therefore the decoding style of the message will be
        required and an nput queue will be configured.

        Parameters
        ----------
        canSpecFile : json-File
            The json file has to be configured to describe the setup of can message designed for targets
        """

        self.recvQueue = queue.Queue()

        with open(canSpecFile) as json_data:
            self.targetListSpec = json.load(json_data)


    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getTargetList                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getTargetList(self, timeout=None):
        """
        The function will be called fro the outside to get the target list of the current cycle.

        Returns
        -------
        targets : list of targets(dict)
            The return value is a list of targets. The single target will be designed as dictionary like the json file.
        """

        ## wait for a header message
        msg = self.recvQueue.get(block=True, timeout=timeout)

        while msg['canID'] != self.targetListSpec['header']['CanId']:
            msg = self.recvQueue.get()

        headerSpec = self.targetListSpec['header']['Elements']

        header = self.__decodeMsgElements(msg['data'], headerSpec)

        ## wait for the delivered number of
        targets = []
        for i in range(0, int(header['Number_Of_Objects'])):
            target = self.__decodeMultiFrameMsg(self.recvQueue.get()['data'])
            target.update(self.__decodeMultiFrameMsg(self.recvQueue.get()['data']))
            target.update(header)

            targets.append(deepcopy(target))

        return deepcopy(targets)


    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __decodeMultiFrameMsg                                                                                  #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __decodeMultiFrameMsg(self, msg):
        """
        The current function will deal with the multiframe state of the can message. It will call the correct decoding
        style.

        Parameters
        ----------
        msg : binary
            The message is the binary coded style of the data

        Returns
        -------
        target : dict
            The return value is the decoded target from the message
        """

        if ( msg[0] ) & 0x1 == 0:
            frame = 'frame0'
        else:
            frame = 'frame1'

        return self.__decodeMsgElements(msg, self.targetListSpec[frame]['Elements'])

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __decodeMsgElements                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __decodeMsgElements(self, msg, decodingDict):
        """
        The function will deal with the single decoding of every information listed in the json file for a given
        message.

        Parameters
        ----------
        msg             : binary
            The message will be a binary format information which has to be decoded

        decodingDict    : dict
            The decoding dict includes all rules to extract information from the current message

        Returns
        -------
        targetDict      : dict
            The target dictionary includes all decoded information
        """

        targetDict = dict()

        for decRule in decodingDict:
            targetDict[decRule] = self.decode(msg, decodingDict[decRule])

        return targetDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: clearQueue                                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def clearQueue(self):
        """
        Clear all targets in the queue.

        Returns
        -------
        None
        """
        while self.isEmpty() == False:
            self.getTargetList(0.01)
