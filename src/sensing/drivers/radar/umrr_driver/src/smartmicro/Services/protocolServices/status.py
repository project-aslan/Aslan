import logging
from smartmicro.Protocols.cmdStatParam.cspCommon import cspCommon, errorFunctionID
# from smartmicro.Protocols.cmdStatParam.cspSendVersion1 import cspSendVersion1
from smartmicro.Protocols.cmdStatParam.cspSendVersion2 import cspSendVersion2
from smartmicro.Protocols.uat.uatMain import MessageType, DataFormat, ParameterType
from smartmicro.Services.communication import comDeviceTypes


class Status:
    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __init__                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, _comModule, statusFileList=None, rootDir=None, uatVersion=4, _comDeviceType=comDeviceTypes.CAN):


        """
        The init requires a communication module instance since it requires a means to read status values

        Status
        ----------
        _comModule          : instance of communication module
            internal used instance to send messages
        statusFileList      : string
            contains a file that contains a list of status files.
        rootDir             : string
            this is the start directory where the status module searches for .status files
        uatVersion          : integer
            version of used uat version, default version 4
        _comDeviceType      : comDeviceTypes
            specifies with which device the status is requested

        Raises
        ------
        ValueError
            if rootDir and statusFileList are set
        """
        self.comDeviceType = _comDeviceType
        # copy instance of communication module
        self.comModule     = _comModule
        # set empty dictionary for status list
        self.statusValues  = dict()
        # define default error function dictionary
        self.errorFuncDict   = dict()
        # define default error function dictionary
        self.txConfig      = dict()
        # set uat version
        self.uatVersion    = uatVersion
        # check if file with status are available
        if rootDir is not None and statusFileList is not None:
            raise ValueError("Either use root Dir or a status file list")
        # update send data
        self.configureTxParameter(_comDeviceType)
        # derive status structure from given file
        cspCommon.initializeValueDict(self.statusValues, statusFileList, rootDir, fileExt="status")

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
    # function: readStatus                                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def readStatus(self, sectionName, paramName, dimension=None, element=None):
        """
        The function provides all necessary steps to encode a status request as well as the function to send the
        message. The usage of the status function depends on the used uat version. The uat versions 1-4 support a
        function call which provides a single message.

        Additional the uat version 4 supports a call with a list of dictionaries within the "paramName" variable. The
        structure of the dictionary has to be the following one:

        +----------------+----------------------+-----------------------------------+
        | key            | property             | comment                           |
        +================+======================+===================================+
        | name           | string               | used parameter of the section     |
        +----------------+----------------------+-----------------------------------+

        Parameters
        ----------
        sectionName : string
            name of the used parameter section
        paramName   : string or list of dictionaries
            name of the used parameter
        dimension   : integer
            array dimension
        element     : integer
            element position in the array
        """
        # if self.uatVersion is 1 or self.uatVersion is 2 or self.uatVersion is 3:
        #     # build parameter dictionary
        #     statusReq = cspSendVersion1.buildMessageDict(sectionName, paramName, self.statusValues,
        #                                                  self.uatVersion, 0, write=False, read=True)
        #     # build parameter array dimension version 3 - part 1
        #     statusReq.update(cspSendVersion1.addMessageExtension1(element))
        #     # build parameter array dimension version 3 - part 2
        #     statusReq.update(cspSendVersion1.addMessageExtension2(dimension, element))
        if self.uatVersion is 4:
            statusReq = cspSendVersion2.buildMessageDict(sectionName, paramName, self.statusValues, value=0,
                                                         element=element, msgType=MessageType.STATUS,
                                                         deviceId=self.txConfig["deviceId"])
        else:
            raise ValueError('The uat version {} is not supported' .format(self.uatVersion))

        # send message
        logging.info("StatusModule: Read Status: {}" .format(statusReq))
        self.__sendRawStatusRequest(statusReq, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getStatusDict                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getStatusDict(self):
        """
        The function returns the possible status dictionary.

        Returns
        -------
        statusValues  : list of dictionaries
            the dictionary corresponding to the module
        """
        return self.statusValues

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __sendRawStatusRequest                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __sendRawStatusRequest(self, param, errorHandler=None):
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

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendRawStatus                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendRawStatus(self, section, parameterNr, dimension=[], devId=0, statusType="integer"):
        """
        The function sets all relative values to send the raw status request.

        Parameters
        ----------
        section     : integer
            current raw section index
        parameterNr : integer
            current raw parameter index
        dimension   : list()
            list of dimension entries ( e.g. [0, 1, 0] (3d) )
        devId       : integer
            device id of the requested parameter
        statusType  : string
            assign "float" or "integer" value type

        Returns
        -------

        """
        # set default status to None
        status = None
        # pack all status requests from version 1...3
        # if 0 < self.uatVersion < 4:
        #     # provide parameter dictionary
        #     status = self.__provideStatusDictVersion1(section, parameterNr, dimension, devId, statusType)
        # pack all status requests from version 4
        if self.uatVersion == 4:
            # provide parameter dictionary
            status = self.__provideStatusDictVersion4(section, parameterNr, dimension, devId, statusType)
        # send status request
        self.__sendRawStatusRequest(status, errorHandler=self.errorFuncDict)

    # # ---------------------------------------------------------------------------------------------------------------- #
    # # function: __provideStatusDictVersion1                                                                            #
    # # ---------------------------------------------------------------------------------------------------------------- #
    # def __provideStatusDictVersion1(self, section, parameterNr, dimension, devId, statusType):
    #     """
    #     The function provides a dictionary to request a status with uat version 1.
    #
    #     Parameters
    #     ----------
    #     section     : integer
    #         current raw section index
    #     parameterNr : integer
    #         current raw parameter index
    #     dimension   : list
    #         list of dimension entries ( e.g. [0, 1, 0] (3d) )
    #     devId       : integer
    #         device id of the requested parameter
    #     statusType  : string
    #         assign "float" or "integer" value type
    #
    #     Returns
    #     -------
    #
    #     """
    #     # derive parameter type
    #     if statusType == "float":
    #         statusType = ParameterType.IEEE_FLOAT_RW
    #     else:
    #         statusType = ParameterType.INTEGER_RW
    #
    #     # create dictionary
    #     param = dict()
    #     # fill parameter dictionary
    #     param["section"]          = section
    #     param["uatVersion"]      = 1
    #     param["id"] = parameterNr
    #     param["parameterType"]   = statusType
    #     param["value"]           = 0
    #     param["deviceId"]        = devId
    #     param["dim0"]            = 0
    #     param["dim1"]            = 0
    #     param["dim2"]            = 0
    #     param["dim3"]            = 0
    #     param["dim4"]            = 0
    #     param["dim5"]            = 0
    #     param["dim6"]            = 0
    #     param["dim7"]            = 0
    #
    #     counter = 0
    #
    #     for element in dimension:
    #
    #         param["dim" + str(counter)] = element
    #
    #         counter += 1
    #
    #     return param

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __provideStatusDictVersion4                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __provideStatusDictVersion4(self, section, parameterNr, dimension, devId, statusType):
        """
        The function provides a dictionary to request a status with uat version 4.

        Parameters
        ----------
        section     : integer
            current raw section index
        parameterNr : integer
            current raw parameter index
        dimension   : list()
            list of dimension entries ( e.g. [0, 1, 0] (3d) )
        devId       : integer
            device id of the requested parameter
        statusType  : string
            assign "float" or "integer" value type

        Returns
        -------

        """
        # derive parameter type
        if statusType == "float":
            statusType = DataFormat.FLOAT_IEEE
        else:
            statusType = DataFormat.INTEGER
        # create parameter dictionary
        status = dict()
        # fill base information
        status['UAT-ID']                 = section
        status['uatVersion']             = 4
        status['deviceId']               = devId
        status['numberOfInstructions']   = 1
        # fill parameter instruction
        status['instructionsList']                       = [dict()] * status['numberOfInstructions']
        status['instructionsList'][0]['section']          = section
        status['instructionsList'][0]['dataFormat']      = statusType
        status['instructionsList'][0]['messageType']     = MessageType.STATUS
        status['instructionsList'][0]['id'] = parameterNr
        status['instructionsList'][0]['value']           = 0
        status['instructionsList'][0]['instructionIdx']  = 0

        try:
            status['instructionsList'][0]['dim0'] = dimension[0]
        except LookupError:
            status['instructionsList'][0]['dim0'] = 0

        try:
            status['instructionsList'][0]['dim1'] = dimension[1]
        except LookupError:
            status['instructionsList'][0]['dim1'] = 0

        return status