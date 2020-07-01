from enum import Enum, unique


@unique
class CanFrameFormat(Enum):
    """
    This class describes the different can format.
    """
    std = 0     #: Standard frame format (11 bit identifier)
    xtd = 1     #: Extended frame format (28 bit identifier)
    fd  = 2     #: CAN FD (Flexible Data Rate)


class CanInterfaceError(Exception):
    pass


class CanInterface:
    """
    The can interface is used in any can adapter class.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getAdapter                                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def getDeviceList(path):
        """
        Returns all devices that are connected to the pc and are not already in use.

        Parameters
        ----------
        path    : str
            The path from the can dll file

        Returns
        -------

        """
        raise CanInterfaceError('getDeviceList function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initChannel                                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    def initChannel(self, bitRate, channelIndex=1):
        """
        Initializes the can driver and configures the can bus.

        Parameters
        ----------
        bitRate         : int
            The channel bit rate in kbit/s
        channelIndex    : int
            Specifies the channel index

        Returns
        -------

        """
        raise CanInterfaceError('initChannel function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendData                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendData(self, identifier, dlc, data, frameFormat=CanFrameFormat.std, channelIndex=1):
        """
        Send a can message.

        Parameters
        ----------
        identifier      : int
            Can message identifier
        dlc             : int
            Can message data length
        data            : list
            Can message data
        frameFormat     : instance of can frame format
            Specifies the can frame format (standard or extended or canFD
        channelIndex    : int
            Specifies the can channel number

        Returns
        -------

        """
        raise CanInterfaceError('sendData function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: receiveData                                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    def receiveData(self, channelIndex=1):
        """
        Returns the received can messages.

        Parameters
        ----------
        channelIndex    : int
            Specifies the can channel number

        Returns
        -------

        """
        raise CanInterfaceError('receiveData function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: status                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def status(self, channelIndex=1):
        """
        Print the can bus status.

        Parameters
        ----------
        channelIndex    : int
            Specifies the can channel number

        Returns
        -------

        """
        raise CanInterfaceError('status function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: changeBitRate                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def changeBitRate(self, bitRate, channelIndex=1):
        """
        This function will be change the channel bit rate.

        Parameters
        ----------
        bitRate         : int
            The channel bit rate in kbit/s
        channelIndex    : int
            Specifies the can channel number

        Returns
        -------

        """
        raise CanInterfaceError('changeBitRate function is not implemented')

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: close                                                                                                  #
    # ---------------------------------------------------------------------------------------------------------------- #
    def close(self):
        """
        Close the can driver.

        Returns
        -------

        """
        raise CanInterfaceError('close function is not implemented')
