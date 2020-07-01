import logging
from smartmicro.Protocols.cmdStatParam.cspCommon import cspCommon, errorFunctionID
from smartmicro.Protocols.uat.uatMain import DataFormat, MessageType
from smartmicro.Services.communication import comDeviceTypes


class Command:

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, _comModule, commandFileList=None, rootDir=None, uatVersion=4, _comDeviceType=comDeviceTypes.CAN):
        """
        The init requires a communication module instance since it requires a means to transfer commands

        Parameters
        ----------
        _comModule          : instance of communication module
            internal used instance to send messages
        commandFileList : list
            contains a file that contains a list of command files. This parameter overrules the root dir parameter
        rootDir         : str
            this is the start directory where the command module searches for .command files
        uatVersion          : integer
            version of used uat version, default version 4
        _comDeviceType      : comDeviceTypes
            specifies with which device the status is requested

        Raises
        ------
        ValueError
            if rootDir and commandFileList are set

        """
        self.comDeviceType = _comDeviceType
        # register CAN module
        self.comModule      = _comModule
        # set uat version for command module
        self.uatVersion     = uatVersion
        # define default error function dictionary
        self.errorFuncDict  = dict()
        # define default error function dictionary
        self.txConfig       = dict()
        # create empty command dictionary
        self.commandValues  = dict()

        if rootDir is not None and commandFileList is not None:
            logging.error("Either use root Dir or a command file list, FIXME")
        # update send data
        self.configureTxParameter(_comDeviceType)

        cspCommon.initializeValueDict(self.commandValues, commandFileList, rootDir, fileExt="command")

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: configureErrorFunction                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def configureErrorFunction(self, errorIndex=errorFunctionID.PRE_ENCODE_ERROR, errorFunction=None):
        """
        The function provides the possibility to configure error injection functions. Therefore the following two
        possibilities are provided:

        * pre-encode  : set a function directly before the encoding starts
        * post-encode : set a function directly after the encoding

        Both possibilities ensure the manipulation of the send data. The function itself needs the following structure:

        >>> def errorInjectFunction(self, data):
        >>>     pass

        The inserted data diverge depending on the expected execution time:

        +-------------------+---------------------+
        | errorIndex        | input               |
        +===================+=====================+
        | PRE_ENCODE_ERROR  | dictionary of data  |
        +-------------------+---------------------+
        | POST_ENCODE_ERROR | list of bytearray   |
        +-------------------+---------------------+

        The dictionary of data for the PRE_ENCODE_ERROR injection has the following structure for uat version 4:

        +----------------------+--------------+----------------------------------+
        | key                  | property     | comment                          |
        +======================+==============+==================================+
        | UAT-ID               | integer      | currently used parameter section |
        +----------------------+--------------+----------------------------------+
        | uatVersion           | integer      | currently used uat version       |
        +----------------------+--------------+----------------------------------+
        | deviceId             | integer      | currently requested device       |
        +----------------------+--------------+----------------------------------+
        | numberOfInstructions | integer      | number of instructions           |
        +----------------------+--------------+----------------------------------+
        | instructionsList     | list of dict | instructions to be handled       |
        +----------------------+--------------+----------------------------------+

        The mentioned instruction list has the following structure:

        +----------------------+--------------+----------------------------------+
        | key                  | property     | comment                          |
        +======================+==============+==================================+
        | section              | integer      | currently used parameter section |
        +----------------------+--------------+----------------------------------+
        | dataFormat           | enum         | currently used type of value :   |
        |                      |              | integer or float derived from    |
        |                      |              | 'DataFormat'                     |
        +----------------------+--------------+----------------------------------+
        | messageType          | enum         | currently used type of msg  like |
        |                      |              | read, write or write & read      |
        |                      |              | derived from 'MessageType'       |
        +----------------------+--------------+----------------------------------+
        | id                   | integer      | current used parameter number    |
        +----------------------+--------------+----------------------------------+
        | value                | int\float    | value to be transmitted          |
        +----------------------+--------------+----------------------------------+
        | dim0                 | integer      | first dimension value            |
        +----------------------+--------------+----------------------------------+
        | dim1                 | integer      | second dimension value           |
        +----------------------+--------------+----------------------------------+
        | instructionIdx       | integer      | instruction idx counter          |
        +----------------------+--------------+----------------------------------+

        If the selected index is 'POST_ENCODE_ERROR' the input is a list of byte arrays

        Parameters
        ----------
        errorIndex      : enumerate
            current used position of the function ( PRE_ENCODE_ERROR | POST_ENCODE_ERROR )
        errorFunction   : function
            instance of a function

        Returns
        -------

        """

        # assign error function to error function dict
        self.errorFuncDict[errorIndex] = errorFunction

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: configureTxParameter                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    def configureTxParameter(self, deviceType=comDeviceTypes.CAN, channelId=1, canId=0x3fb, deviceId=0, clientId=0x0):
        """
        The function allows to reconfigure the requested target as well as to configure the used interface and depending
        on the used device the selected can channel (can device) and the client identifier (ethernet) of the target.

        Parameters
        ----------
        deviceType  : comDeviceTypes
            select device type of sending
        channelId   : integer
            if multi channel support is available select the channel number (start: 1)
        canId       : integer
            select the used can identifier (default: 0x3fb)
        deviceId    : integer
            select the used device identifier (multi sensor on can)
        clientId    : integer
            contains the client identifier to search the target address within the com module
        """
        self.txConfig["deviceType"]   = deviceType
        self.txConfig["channelIndex"] = channelId
        self.txConfig["canId"]        = canId
        self.txConfig["deviceId"]     = deviceId
        self.txConfig["clientId"]     = clientId

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendCommand                                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendCommand(self):
        raise BaseException ("Not implemented currently!")

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getCommandDict                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getCommandDict(self):
        """
        The function returns the possible command dictionary.

        Returns
        -------
        commandValues  : list of dictionaries
            the dictionary corresponding to the module
        """
        return self.commandValues

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendRawCommand                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendRawCommand(self, section, id, value, deviceId=0):
        """

        Parameters
        ----------
        section
        id
        value
        deviceId    : integer
            sets the device id for the request

        Returns
        -------

        """
        instruction = dict()
        instruction["messageType"]     = MessageType.COMMAND
        instruction["dataFormat"]      = DataFormat.INTEGER
        instruction["id"] = id
        instruction["value"]           = value
        instruction["dim0"]            = 0
        instruction["dim1"]            = 0
        instruction["instructionIdx"]  = 0
        instruction['section']          = section
        commandModuleParam = dict()
        commandModuleParam['UAT-ID'] = section
        commandModuleParam['uatVersion'] = self.uatVersion
        commandModuleParam['deviceId'] = deviceId
        # set default number of instructions
        commandModuleParam['numberOfInstructions'] = 1
        # provide default instruction list
        commandModuleParam['instructionsList'] = [instruction]

        # send message
        logging.info("CommandModule: Send Raw Command: {}" .format(commandModuleParam))
        self.__sendRawCommand(commandModuleParam, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __sendRawCommand                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __sendRawCommand(self, param, errorHandler=None):
        """
        The function calls the function to send the messages.

        Parameters
        ----------
        param       : dict
            dictionary to be send
        errorHandler    : dict
            generates defined errors at a value not equal to zero

        Returns
        -------

        """
        cspCommon.sendRawParameter(param, comModule=self.comModule, txConfig=self.txConfig, errorHandler=errorHandler)