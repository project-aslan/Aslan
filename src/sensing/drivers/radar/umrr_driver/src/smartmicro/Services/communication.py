import threading

from enum import Enum, unique
from copy import deepcopy
from smartmicro.Helper.extendedThreadHelper.CanRecv import canReceive
# from smartmicro.HardwareLayer.comFactory.comDeviceFactory import ComDeviceFactory

@unique
class comDeviceTypes(Enum):
    """
    Enumeration of all communication device types that are supported by the communication module.
    """
    CAN      = 1
    CAN_FD   = 2

@unique
class deviceList(Enum):
    """
    Enumeration of available device list entries.
    """
    STATIC_DEVICE_LIST  = 1
    DYNAMIC_DEVICE_LIST = 2

class Communication:
    """
    The communication class provides the interface to the user to allow for the usage of
    all available communication interfaces.

    The communication module provides different interfaces.
    * a can message based interface

    The transmission of data is done in a synchronized way. Everything is transmitted before returning.
    The reception of data has to be implemented by the registration of services. The following services can be registered
    * canIdService

    The registration of a can ID service enables the reception of can messages for a given can ID range. The registered
    service receive each can message as a single object in it receive queue.

    """
    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, deviceDictList=None, clientId=0):
        """

        Parameters
        ----------
        deviceDictList  : list
            a list of dictionaries. [{'deviceType', 'device'}]
        clientId        : integer

        """
        # save input variables
        self.deviceDict                   = deviceDictList
        self.clientId                     = clientId

        self.receiveThreadCan             = dict()
        self.registeredCanServices        = []
        self.registeredPortServices       = []
        self.registeredDeviceList         = []
        self.staticDeviceList             = []
        self.send_mutex                   = threading.Lock()
        self.registered_response_services = []
        self.recordHelperQueue            = None
        self.smsTpVersion                 = 1
        # configure usage of device list
        self.deviceListUsageDict = {deviceList.STATIC_DEVICE_LIST : True,
                                    deviceList.DYNAMIC_DEVICE_LIST: True}
        #set segmentInd for port communication
        self.segIndex = 1

        if deviceDictList is not None:

            deviceTypeList = []
            # Creates a list with all deviceTypes from the deviceDictList
            for dev in deviceDictList:
                deviceTypeList.append(dev['deviceType'])
            # check if each key is only present once
            for devType in comDeviceTypes:
                if deviceTypeList.count(devType) > 1 :
                    raise ValueError('Communication Module: One com device type is more than once in the deviceDictList'
                                     '. Type:{} comDeviceList:{}' .format(devType, self.deviceDict))

            # Init for each deviceType a receive thread
            for dev in deviceDictList:
                if dev['deviceType'] is comDeviceTypes.CAN:
                    self.receiveThreadCan[comDeviceTypes.CAN] = canReceive(dev['device'])
                    self.receiveThreadCan[comDeviceTypes.CAN].start()
                else:
                    raise ValueError("Communication Module: unknown device")

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: enableDisableDeviceListUsage                                                                           #
    # ---------------------------------------------------------------------------------------------------------------- #
    def enableDisableDeviceListUsage(self, listType=deviceList.STATIC_DEVICE_LIST, enable=True):
        """
        The function enables the device list usage for static or dynamic list usage.

        Parameters
        ----------
        listType    : deviceList
            contains the used list type
        enable      : bool
            assign enabled or disabled list
        """

        self.deviceListUsageDict[listType] = enable

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: addSensorDeviceToList                                                                                  #
    # ---------------------------------------------------------------------------------------------------------------- #
    def addSensorDeviceToList(self, serialNr=0, deviceId=0, udpIp=None, udpPort=None, clientId=0, staticList=False):
        """
        The function adds a new device to the device list.

        Parameters
        ----------
        serialNr    : integer
            serial number of the device
        deviceId    : integer
            target device identifier
        udpIp       : string
            ip address of the target (e.g."192.168.11.1")
        udpPort     : integer
            used port
        clientId    : integer
            client identification number
        staticList  : bool
            assign static (True) or dynamic (False(default)) list entry
        """
        # check if ip address and port is given
        if udpIp is None or udpPort is None:
            # provide error
            raise ValueError("udp port or udp address missing")
        # provide dictionary with device data
        device = {"serialNr": serialNr,
                  "deviceId": deviceId,
                  "udpPort" :  udpPort,
                  "udpAddr" :    udpIp,
                  "clientId": clientId}
        # check if static or dynamic list is updated
        if staticList:
            # update static list
            self.staticDeviceList.append(deepcopy(device))
        else:
            # update dynamic list
            self.registeredDeviceList.append(deepcopy(device))

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: clearDeviceList                                                                                        #
    # ---------------------------------------------------------------------------------------------------------------- #
    def clearDeviceList(self, staticList=False):
        """
        The function deletes all entries in the device list.

        Parameters
        ----------
        staticList  : bool
            assign static (True) or dynamic (False(default)) list entry
        """
        # check if static or dynamic list is updated
        if staticList:
            # update static list
            self.staticDeviceList     = []
        else:
            # update dynamic list
            self.registeredDeviceList = []

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: enableSensorLogging                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def enableSensorLogging(self, deviceType):
        """


        Parameters
        ----------
            deviceType: (comDeviceTypes) specifies which communication device is used obtain the log messages from

        Returns
        -------
        None
        """

        if deviceType == comDeviceTypes.CAN:
            raise IOError("a can device does not support log message output")

        com_receive = self.receiveThreadCan[deviceType]
        com_receive.enableLogMessageOutput()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: disableSensorLogging                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    def disableSensorLogging(self, deviceType):
        """
        Parameters
        ----------
            deviceType: (comDeviceTypes) specifies which communication device is used obtain the log messages from

        Returns
        -------
        None
        """

        if deviceType == comDeviceTypes.CAN:
            raise IOError("a can device does not support log message output")

        com_receive = self.receiveThreadCan[deviceType]
        com_receive.disableLogMessageOutput()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: unregisterSensorLogQueue                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def unregisterSensorLogQueue(self, deviceType):
        """
        Parameters
        ----------
            deviceType: (comDeviceTypes) specifies which communication device is used obtain the log messages from

        Returns
        -------
        None
        """

        if deviceType == comDeviceTypes.CAN:
            raise IOError("a can device does not support log message output")

        com_receive = self.receiveThreadCan[deviceType]
        com_receive.unregisterLogMessageQueue()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: flushAllRegisterServices                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def flushAllRegisterServices(self):
        """
        This flushes all registers can services.

        Returns
        -------
        None

        """
        # flush all registers can services
        for service in self.registeredCanServices:
            service.clearQueue()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: registerCanIDService                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    def registerCanIDService(self, service, canID, canIDRange=0, deviceType=comDeviceTypes.CAN):
        """
        This registers a can id service which provides a receive queue. After the registration the communication
        module puts any received message with the given can ids into the queue that is owned by the specified service.

        :param service: can id services that provides as interface a receive queue
        :param canID: can id that shall be registered with the provided services
        :param canIDRange: registers the service for the can id: (canID : canID + canIDRange), default is 0
        :param deviceType: (comDeviceTypes) specifies which communication device is used to transfer the given can messages.
        """
        # add the service to the register service list
        self.registeredCanServices.append(service)
        # get service queue
        serviceQueue = service.getRXQueue()

        # register the service queue
        self.receiveThreadCan[deviceType].registerQueue(serviceQueue, canID, canIDRange)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: registerDebugStreamerService                                                                           #
    # ---------------------------------------------------------------------------------------------------------------- #
    def registerRecordHelper(self, recordHelper, deviceType=comDeviceTypes.CAN):
        """
        This registers a can recording helper which provides a receive queue. After the registration the communication
        module puts any received message into the queue that is owned by the specified helper.


        :param recordHelper: canRecordingHelper. instance of can recording helper class
        :param deviceType: comDeviceTypes
        """
        # get recording queue
        self.recordHelperQueue = recordHelper.getRecordQueue()
        # register recordQueue
        self.receiveThreadCan[deviceType].registerRecordingQueue(self.recordHelperQueue)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: suspendCommunicationDevice                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def suspendCommunicationDevice(self, deviceType):
        """
        Stops putting data into the registered service queues for the specified communication device type

        Parameters
        ----------
            deviceType

        """
        # Suspends the receiveThread
        self.receiveThreadCan[deviceType].suspend()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: suspendAllCommunicationDevices                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def suspendAllCommunicationDevices(self):
        """
        Stops putting data into the registered service queues for alll registered communication device type
        """
        for key in self.receiveThreadCan:
            self.receiveThreadCan[key].suspend()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: resumeCommunicationDevice                                                                              #
    # ---------------------------------------------------------------------------------------------------------------- #
    def resumeCommunicationDevice(self, deviceType):
        """
        Resumes putting data from the specified communication device type into the registered service queues

        :param deviceType:
        :return:
        """
        # Resumes the receiveThread
        self.receiveThreadCan[deviceType].resume()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: shutDownCommunicationDevice                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    def shutDownCommunicationDevice(self, deviceType):
        """
        Shut down putting data from the specified communication device type into the registered service queues

        :param deviceType:
        :return:
        """
        # Shut down the received Thread
        self.receiveThreadCan[deviceType].shutDown()
        self.receiveThreadCan[deviceType].join()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: shutDownAllCommunicationDevices                                                                        #
    # ---------------------------------------------------------------------------------------------------------------- #
    def shutDownAllCommunicationDevices(self):
        """
           Shut down putting data from all registered communication device type into the registered service queues
        """
        for key in self.receiveThreadCan:
            self.receiveThreadCan[key].shutDown()
            self.receiveThreadCan[key].join()

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __getComDeviceByType                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __getComDeviceByType(self, deviceType):
        """

        :param deviceType:
        """
        dictDev = next(item for item in self.deviceDict if item['deviceType'] == deviceType)
        comDevice = dictDev['device']

        return comDevice

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: close                                                                                                  #
    # ---------------------------------------------------------------------------------------------------------------- #
    def close(self):
        """
        Closes all communication devices that where registered at instantiation time

        Returns
        -------
        errorList   : list

        """
        errorList = list()
        for dev in self.deviceDict:
            try:
                dev['device'].close()
            except Exception as e:
                # add the error information to the list
                errorList.append(e)

        return errorList

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendCanMessages                                                                                        #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendCanMessages(self, canMessageList, deviceType=comDeviceTypes.CAN, channelIndex=1, destId=0):
        """
        This is a function to send a list of can messages at once. The device type parameter decides which registered
        device. This interface is a can message interface only. The lower levels are dealing how to transfer these
        type of message over the chosen communication channel (deviceType)

        Parameters
        ----------
        canMessageList  : list
            A list of can messages
        deviceType      : comDeviceTypes
            selected device to be used to determine the package size
        channelIndex    : int
            index of the used can channel
        destId          : integer
            client id of the corresponding device

        Returns
        -------
        """
        # get transmission functions
        txFunc = self.__getCanMsgTransmissionFunctions(deviceType)
        # handle the msg recording of send messages
        self.__handleRecording(canMessageList, channelIndex)
        # protect sending process
        with self.send_mutex:
            # select device to send
            txDevice = self.__selectTransmissionDevice(deviceType)
            # transmit data
            txFunc(canMessageList, txDevice, channelIndex, destId)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __getCanMsgTransmissionFunctions                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __getCanMsgTransmissionFunctions(self, deviceType):
        """
        The function holds all transmission functions and provides the corresponding function depending on the given
        device type.

        Parameters
        ----------
        deviceType  : comDeviceTypes
            device type to be used

        Returns
        -------
        txFunc      : instance
            instance of the transmission function
        """
        # provide transmission function map
        txFuncMap = dict()
        # add function for each interface
        txFuncMap[comDeviceTypes.CAN]      = self.__sendViaCanDevice

        return txFuncMap[deviceType]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __sendViaCanDevice                                                                                     #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __sendViaCanDevice(self, canMessageList, txDevice, channelIndex, destId):
        """
        The function handles the main process of sending CAN messages over can interface.

        Parameters
        ----------
        canMessageList  : dict
            list of message dictionaries
        txDevice        : instance
            instance of the transmission device
        channelIndex    : integer
            channel number of the selected can device
        destId          : integer
            client id of the corresponding device

        Returns
        -------

        """
        # send the messageList
        for msg in canMessageList:
            # throw an exception if the data is too long
            if len(msg['data']) > 8:
                raise ValueError("Can data size must not exceed 8 bytes")
            # send the can message
            txDevice.canDevice.sendData(msg['canId'], len(msg['data']), msg['data'], channelIndex=channelIndex)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __handleRecording                                                                                      #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __handleRecording(self, canMessageList, channelIndex=1):
        """
        The function handles the recording of data within the *.dr2 file. Therefore all messages will be extend with dlc
        and channel information.

        Parameters
        ----------
        canMessageList  : list
            list of messages to be send as dictionary
        channelIndex    : integer
            channel to be used

        Returns
        -------
        """
        # verify that recording is necessary
        if self.recordHelperQueue is not None:
            # run through every single message
            for msg in canMessageList:
                # update the msg dictionary
                msg["channelIndex"] = channelIndex
                msg["canID"]        = msg["canId"]
                msg["dlc"]          = len(msg['data'])
                # throw an exception if the data is too long
                if len(msg['data']) > 8:
                    raise ValueError("Can data size must not exceed 8 bytes")
                # store message to be send
                self.recordHelperQueue.put(deepcopy(msg))

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __selectTransmissionDevice                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __selectTransmissionDevice(self, deviceType):
        """
        The function selects a transmission device depending on the given device type.

        Parameters
        ----------
        deviceType  : comDeviceTypes
            device type to be used

        Returns
        -------
        txDevice    : instance
            device instance if available if not set to be None
        """
        # define default transmission device as None
        txDevice = None
        # run through all registered devices
        for device in self.receiveThreadCan.keys():
            # check if device type is matched
            if device is deviceType:
                # store selected device
                txDevice = self.receiveThreadCan[device]
        # throw an exception if no device was found
        if txDevice is None:
            raise LookupError("Specified communication device not found: {}". format(deviceType.name))

        return txDevice

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __extractTransmitAddress                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __extractTransmitAddress(self, destId):
        """
        The function extracts the destination address of a registered device.

        Parameters
        ----------
        destId  : integer
            client id of the target

        Returns
        -------
        txAddress   : dict
            address data of the target
        """
        # set default device list to None
        txAddress = None
        # check if dynamic list is enabled
        if self.deviceListUsageDict[deviceList.DYNAMIC_DEVICE_LIST]:
            # check registered device list
            for device in self.registeredDeviceList:
                # verify that target exists
                if destId == device["clientId"]:
                    # fill in the destination address
                    txAddress = device
        # check if static list is enabled
        if self.deviceListUsageDict[deviceList.DYNAMIC_DEVICE_LIST]:
            # check if destination address is set
            if txAddress is None:
                # check static device list
                for device in self.staticDeviceList:
                    # verify that target exists
                    if destId == device["clientId"]:
                        # fill in the destination address
                        txAddress = device
        # check if destination address is available
        if txAddress is None:
            # provide error
            raise LookupError("Transmission Target does not exist for client ID: {}".format(destId))

        return txAddress

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __handleCanPackages                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __handleCanPackages(self, canMessageList, deviceType, destId):
        """
        The function cuts the can messages in several parts to handle them within the sensor.

        Parameters
        ----------
        canMessageList  : dict
            list of can message dictionaries
        deviceType      : comDeviceTypes
            selected device to be used to determine the package size
        destId          : integer
            client id of the corresponding device

        Returns
        -------
        outputDataList  : list
            list of message dictionaries
        """
        # reset message counter
        self.messageCounter = 0
        # define output message list
        outputDataList = list()
        # run as long as messages are available
        while len(canMessageList) > 0:
            # define current message list
            curMessageList = list()
            while len(canMessageList) > 0 and len(curMessageList) < self.nofMessagesDict[deviceType]:
                # append first message to current list
                curMessageList.append(canMessageList[0])
                # remove first element
                canMessageList.pop(0)
            # define output data list
            outputDataList.append(self.__provideStreamPackage(curMessageList, deviceType, self.messageCounter, destId))
            # increment message counter
            self.messageCounter += 1

        return outputDataList