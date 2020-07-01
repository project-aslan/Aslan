import queue
import struct

class CanIDService:
    """
    Can ID service interface class.

    This interface class has to be used to implement any can id based service that shall have be able to be
    registered with the communication module :class:`test_framework.communication.communication`
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self):
        """
        The function initializes the basic values of the class.
        """
        self.recvQueue = None

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getRXQueue                                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getRXQueue(self):
        """
        The function returns the memory information of the queue.

        Returns
        -------
            queue
        """
        return self.recvQueue

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: clearQueue                                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def clearQueue(self):
        """
        Flushes the recvQueue.

        Returns
        -------
        None
        """
        while self.isEmpty() is False:
            self.getRXQueue()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: isEmpty                                                                                                #
    # ---------------------------------------------------------------------------------------------------------------- #
    def isEmpty(self):
        """
        Checks if the queue is empty or not.

        Returns
        -------
        retValue    : bool
            True queue is empty. False queue is not empty.

        """
        retValue = self.recvQueue.empty()

        return retValue

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def decode(cls, msg, extractRule):
        """
        The function provides the basic rule to decode messages.

        Parameters
        ----------
        msg         : binary
            the message is a binary format of interested information
        extractRule : dict
            the dictionary is a set of information to decode the information from the message

        Returns
        -------
        retValue    : value
            the value represents the extracted information
        """

        data = cls._extractBytes(msg, extractRule)

        if 'resolution' in extractRule:
            resolution = extractRule['resolution']
        else:
            resolution = 1

        if 'offset' in extractRule:
            offset = extractRule['offset']
        else:
            offset = 0

        retValue = (data - offset) * resolution

        return retValue

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: _extractBytes                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def _extractBytes(cls, msg, extractRule):
        """
        The function extracts the requested bytes from the array.

        Parameters
        ----------
        msg         : bytearray
            msg with the necessary data
        extractRule : dictionary
            dictionary with the extraction rule

        Returns
        -------
        data        : integer
            extracted data
        """
        # get message length
        msgLength = len(msg)
        # append data to message
        if msgLength < 8:
            msg += bytearray(8 - msgLength)

        convertedData = struct.unpack("<Q", msg)[0]

        startBitShift = 8 * extractRule['start_byte'] + extractRule['start_bit']
        endBitShift   = 8 * (8 - extractRule['end_byte']) - (extractRule['end_bit'] + 1)

        referenceShiftHigh = 64 - endBitShift

        deactivationMaskTemp = convertedData >> referenceShiftHigh
        deactivationMaskHigh = deactivationMaskTemp << referenceShiftHigh

        data = (convertedData & ~deactivationMaskHigh) >> startBitShift

        return data

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encode(msg, packingRule, value):
        """
        The function provides the basic rule to encode data to a messages.

        Parameters
        ----------
        msg         : byte
            currently used message
        packingRule : dictionary
            rule to encode the given data
        value       : int or float
            value to be packed by the function

        Returns
        -------
        msg         : byte
            currently used message with added data

        """
        # decode already packed data
        packedData = struct.unpack("<Q", msg)[0]
        # calculate number of bits
        endBit   = packingRule['end_byte'] * 8 + packingRule['end_bit']
        startBit = packingRule['start_byte'] * 8 + packingRule['start_bit']

        lengthOfData = endBit - startBit + 1

        codingMask = 0
        # set coding mask
        for bitNumber in range (0, lengthOfData):
            codingMask <<= 1
            codingMask  |= 1

        # calculate number to pack
        packingValue = ( int( value / packingRule['resolution'] ) ) + packingRule['offset']
        # ensure that the value is not bigger than the coding mask
        packingValue &= codingMask

        # configure pre data mask
        preDataMask = 0
        # set coding mask
        for bitNumber in range (0, startBit):
            preDataMask <<= 1
            preDataMask  |= 1

        # configure pre data mask
        postDataMask = 0
        # set coding mask
        for bitNumber in range( 0, ( 64 - endBit ) ):
            postDataMask <<= 1
            postDataMask |= 1

        postDataMask <<= endBit
        # save existing data
        preDataSet  = packedData & preDataMask
        currDataSet = packingValue << startBit
        postDataSet = packedData & postDataMask

        packedData = postDataSet | currDataSet | preDataSet

        return packedData.to_bytes(8, byteorder='little')