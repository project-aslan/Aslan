from smartmicro.HardwareLayer.drivers.canDrivers.canSocket import CanSocket


class ComDeviceFactory:

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: _createCanSocket                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def _createCanSocket(socket):
        """

        Returns
        -------
        device        : instance of can socket
            The can socket handle.
        """
        device = CanSocket(socket)
        return device

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getDevice                                                                                              #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def getDevice(deviceName, socket):
        """
        This function initializes the communication interface and returns the device handle.

        Parameters
        ----------
        deviceName
            The deviceName specifies the communication interface.'CAN_SOCKET'.
        socket
            The socket to use

        Returns
        -------
        device  : instance of device
            The device handle.

        """
        device = None
        if deviceName == 'CAN_SOCKET':
            device = ComDeviceFactory._createCanSocket(socket)

        return device
