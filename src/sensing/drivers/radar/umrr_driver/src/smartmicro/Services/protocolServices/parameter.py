import logging
from smartmicro.Protocols.cmdStatParam.cspCommon import cspCommon, errorFunctionID
# from smartmicro.Protocols.cmdStatParam.cspSendVersion1 import cspSendVersion1
from smartmicro.Protocols.cmdStatParam.cspSendVersion2 import cspSendVersion2
from smartmicro.Protocols.uat.uatMain import MessageType, ParameterType, DataFormat
# from smartmicro.Protocols.udt.udtUatResponseMain import UAT_ErrorHandler
from smartmicro.Services.communication import comDeviceTypes


class Parameter:
    """
    The parameter module provides the functionality to write Parameter via a communication link to a destination.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __init__                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, _comModule, parameterFileList=None, rootDir=None, uatVersion=4,
                 _comDeviceType=comDeviceTypes.CAN):
        """
        The init requires a communication module instance since it requires a means to transfer parameters.

        Parameters
        ----------
        _comModule          : instance of communication module
            internal used instance to send messages
        parameterFileList   : string
            contains a file that contains a list of parameter files. This parameter overrules the root dir parameter
        rootDir             : string
            relative direction of param file
        uatVersion          : integer
            version of used uat version, default version 4
        _comDeviceType      : comDeviceTypes
            specifies with which device the status is requested
        """
        self.com_device_type = _comDeviceType
        # copy instance of communication module
        self.comModule       = _comModule
        # set empty dictionary for parameter list
        self.parameters      = dict()
        # define default error function dictionary
        self.errorFuncDict   = dict()
        # define default error function dictionary
        self.txConfig        = dict()
        # set uat version
        self.uatVersion      = uatVersion
        # check whether root direction or param list file are available
        if rootDir is not None and parameterFileList is not None:
            raise ValueError("Either use root Dir or a parameterFile list")
        # update send data
        self.configureTxParameter(_comDeviceType)
        # derive parameter structure from given file
        cspCommon.initializeValueDict(self.parameters, parameterFileList, rootDir, fileExt="param")

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
    # function: writeParameter                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def writeParameter(self, sectionName, paramName, value=None, dimension=None, element=None):
        """
        The function encodes the given parameter and send them to the sensor to write the parameter. In different
        versions the call may deviate. In uat version 4 it's possible to use a list of dictionaries instead of the
        paramName. This will improve the readability. The setup of the dictionary is the following one:

        +----------------+----------------------+-----------------------------------+
        | key            | property             | comment                           |
        +================+======================+===================================+
        | name           | string               | used parameter of the section     |
        +----------------+----------------------+-----------------------------------+
        | value          | integer/float        | value to be set                   |
        +----------------+----------------------+-----------------------------------+
        | dim0           | integer              | dimensional data first dimension  |
        +----------------+----------------------+-----------------------------------+
        | dim1           | integer              | dimensional data second dimension |
        +----------------+----------------------+-----------------------------------+


        Parameters
        ----------
        sectionName : string
            name of the used parameter section
        paramName   : string or list of dictionaries
            name of the used parameter
        value       : integer/float
            value to be manipulated
        dimension   : integer
            array dimension
        element     : integer
            element position in the array
        """
        # if self.uatVersion is 1 or self.uatVersion is 2 or self.uatVersion is 3:
        #     # build parameter dictionary
        #     param = cspSendVersion1.buildMessageDict(sectionName, paramName, self.parameters, self.uatVersion, value,
        #                                              write=True, read=False)
        #     # build parameter array dimension version 3 - part 1
        #     param.update(cspSendVersion1.addMessageExtension1(element))
        #     # build parameter array dimension version 3 - part 2
        #     param.update(cspSendVersion1.addMessageExtension2(dimension, element))
        if self.uatVersion is 4:
            # set the message type to parameter write (normal mode)
            messageType = MessageType.PARAMETER_WRITE

            param = cspSendVersion2.buildMessageDict(sectionName, paramName, self.parameters, value=value,
                                                     element=element, msgType=messageType, deviceId=self.txConfig["deviceId"])
        else:
            raise ValueError('The uat version is not supported. uat version: {}'.format(self.uatVersion))

        # send message
        logging.info("ParameterModule: Write Parameter: {}".format(param))
        self._sendRawParameter(param, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: readParameter                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def readParameter(self, sectionName, paramName, dimension=None, element=None):
        """
        The function encodes the given parameter and send them to the sensor to read the parameter. In different
        versions the call may deviate. In uat version 4 it's possible to use a list of dictionaries instead of the
        paramName. This will improve the readability. The setup of the dictionary is the following one:

        +----------------+----------------------+-----------------------------------+
        | key            | property             | comment                           |
        +================+======================+===================================+
        | parameterName  | string               | used parameter of the section     |
        +----------------+----------------------+-----------------------------------+
        | value          | integer/float        | value to be set                   |
        +----------------+----------------------+-----------------------------------+
        | dim0           | integer              | dimensional data first dimension  |
        +----------------+----------------------+-----------------------------------+
        | dim1           | integer              | dimensional data second dimension |
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
        #     param = cspSendVersion1.buildMessageDict(sectionName, paramName, self.parameters, self.uatVersion,
        #                                              write=False, read=True)
        #     # build parameter array dimension version 3 - part 1
        #     param.update(cspSendVersion1.addMessageExtension1(element))
        #     # build parameter array dimension version 3 - part 2
        #     param.update(cspSendVersion1.addMessageExtension2(dimension, element))
        if self.uatVersion is 4:
            # create a value list
            value = 0
            if type(paramName) is list:
                value = list()
                for idx in range(len(paramName)):
                    value.append(0)

            # set the message type to parameter read (normal mode)
            messageType = MessageType.PARAMETER_READ

            param = cspSendVersion2.buildMessageDict(sectionName, paramName, self.parameters, value=value,
                                                     element=element, msgType=messageType, deviceId=self.txConfig["deviceId"])
        else:
            raise ValueError('The uat version {} is not supported'.format(self.uatVersion))

        logging.info("ParameterModule: Read Parameter: {}".format(param))
        self._sendRawParameter(param, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: readWriteParameter                                                                                     #
    # ---------------------------------------------------------------------------------------------------------------- #
    def readWriteParameter(self, sectionName, paramName, value=None, dimension=None, element=None):
        """
        The function encodes the given parameter and send them to the sensor to write and read the parameter. In
        different versions the call may deviate. In uat version 4 it's possible to use a list of dictionaries instead of
        the paramName. This will improve the readability. The setup of the dictionary is the following one:

        +----------------+----------------------+-----------------------------------+
        | key            | property             | comment                           |
        +================+======================+===================================+
        | parameterName  | string               | used parameter of the section     |
        +----------------+----------------------+-----------------------------------+
        | value          | integer/float        | value to be set                   |
        +----------------+----------------------+-----------------------------------+
        | dim0           | integer              | dimensional data first dimension  |
        +----------------+----------------------+-----------------------------------+
        | dim1           | integer              | dimensional data second dimension |
        +----------------+----------------------+-----------------------------------+


        Parameters
        ----------
        sectionName : string
            name of the used parameter section
        paramName   : string or list of dictionaries
            name of the used parameter
        value       : integer/float
            value to be manipulated
        dimension   : integer
            array dimension
        element     : integer
            element position in the array
        """

        # if self.uatVersion is 1 or self.uatVersion is 2 or self.uatVersion is 3:
        #     # build parameter dictionary
        #     param = cspSendVersion1.buildMessageDict(sectionName, paramName, self.parameters, self.uatVersion, value,
        #                                              write=True, read=True)
        #     # build parameter array dimension version 3 - part 1
        #     param.update(cspSendVersion1.addMessageExtension1(element))
        #     # build parameter array dimension version 3 - part 2
        #     param.update(cspSendVersion1.addMessageExtension2(dimension, element))
        if self.uatVersion is 4:
            # set the message type to parameter write read (normal mode)
            messageType = MessageType.PARAMETER_WRITE_READ

            param = cspSendVersion2.buildMessageDict(sectionName, paramName, self.parameters, value=value,
                                                     element=element, msgType=messageType, deviceId=self.txConfig["deviceId"])
        else:
            raise ValueError('The uat version is not supported. uat version: {}'.format(self.uatVersion))

        logging.info("ParameterModule: read write Parameter: {}".format(param))
        self._sendRawParameter(param, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getParameterDict                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getParameterDict(self):
        """
        The function returns the possible parameter dictionary.

        Returns
        -------
        parameters  : list of dictionaries
            the dictionary corresponding to the module
        """
        return self.parameters

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: _sendRawParameter                                                                                      #
    # ---------------------------------------------------------------------------------------------------------------- #
    def _sendRawParameter(self, param, errorHandler=None):
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
    # function: sendRawParameter                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    def sendRawParameter(self, section, parameterNr, value=0, dimension=[], devId=0, paramType="integer", msgType="rw"):
        """
        The function sets all relative values to send the raw parameter.

        Parameters
        ----------
        section     : integer
            current raw section index
        parameterNr : integer
            current raw parameter index
        value       : integer
            current used value to be set
        dimension   : list of integer
            list of dimension entries ( e.g. [0, 1, 0] (3d) )
        devId       : integer
            device id of the requested parameter
        paramType   : string
            assign "float" or "integer" value type
        msgType     : string
            assign processing ("r": read | "w": write | "rw": write&read)

        Returns
        -------

        """
        param = None

        # if 0 < self.uatVersion < 4:
        #     # provide parameter dictionary
        #     param = self.__provideParamDictVersion1(section, parameterNr, value, dimension, devId, paramType, msgType)

        if self.uatVersion == 4:
            # provide parameter dictionary
            param = self.__provideParamDictVersion4(section, parameterNr, value, dimension, devId, paramType, msgType)

        self._sendRawParameter(param, errorHandler=self.errorFuncDict)

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __provideParamDictVersion1                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __provideParamDictVersion1(self, section, parameterNr, value, dimension, devId, paramType, msgType):
        """
        The function provides a dictionary to request a parameter with uat version 1.

        Parameters
        ----------
        section     : integer
            current raw section index
        parameterNr : integer
            current raw parameter index
        value       : integer
            current used value to be set
        dimension   : list()
            list of dimension entries ( e.g. [0, 1, 0] (3d) )
        devId       : integer
            device id of the requested parameter
        paramType   : string
            assign "float" or "integer" value type
        msgType     : string
            assign processing ("r": read | "w": write | "rw": write&read)

        Returns
        -------

        """
        if "rw" == msgType:
            # derive parameter type
            if paramType == "float":
                paramType = ParameterType.IEEE_FLOAT_RW
            else:
                paramType = ParameterType.INTEGER_RW
        elif "r" == msgType:
            # derive parameter type
            if paramType == "float":
                paramType = ParameterType.IEEE_FLOAT_READ
            else:
                paramType = ParameterType.INTEGER_READ
        elif "w" == msgType:
            # derive parameter type
            if paramType == "float":
                paramType = ParameterType.IEEE_FLOAT_WRITE
            else:
                paramType = ParameterType.INTEGER_WRITE
        else:
            raise BaseException("ERROR: Current action not possible!")

        # create dictionary
        param = dict()
        # fill parameter dictionary
        param["section"] = section
        param["uatVersion"] = 1
        param["id"] = parameterNr
        param["parameterType"] = paramType
        param["value"] = value
        param["deviceId"] = devId
        param["dim0"] = 0
        param["dim1"] = 0
        param["dim2"] = 0
        param["dim3"] = 0
        param["dim4"] = 0
        param["dim5"] = 0
        param["dim6"] = 0
        param["dim7"] = 0

        counter = 0

        for element in dimension:

            param["dim" + str(counter)] = element

            counter += 1

        return param

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __provideParamDictVersion4                                                                             #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __provideParamDictVersion4(self, section, parameterNr, value, dimension, devId, paramType, msgType):
        """
        The function provides a dictionary to request a parameter with uat version 4.

        Parameters
        ----------
        section     : integer
            current raw section index
        id          : integer
            current raw parameter index
        value       : integer
            current used value to be set
        dimension   : list()
            list of dimension entries ( e.g. [0, 1, 0] (3d) )
        devId       : integer
            device id of the requested parameter
        paramType   : string
            assign "float" or "integer" value type
        msgType     : string
            assign processing ("r": read | "w": write | "rw": write&read)

        Returns
        -------

        """
        # derive parameter type
        if paramType == "float":
            paramType = DataFormat.FLOAT_IEEE
        else:
            paramType = DataFormat.INTEGER

        if "rw" == msgType:
            msgAction = MessageType.PARAMETER_WRITE_READ
        elif "r" == msgType:
            msgAction = MessageType.PARAMETER_READ
        elif "w" == msgType:
            msgAction = MessageType.PARAMETER_WRITE
        else:
            raise BaseException("ERROR: Current action not possible!")

        # create parameter dictionary
        param = dict()
        # fill base information
        param['UAT-ID'] = section
        param['uatVersion'] = 4
        param['deviceId'] = devId
        param['numberOfInstructions'] = 1
        # fill parameter instruction
        param['instructionsList'] = [dict()] * param['numberOfInstructions']
        param['instructionsList'][0]['section']          = section
        param['instructionsList'][0]['dataFormat']      = paramType
        param['instructionsList'][0]['messageType']     = msgAction
        param['instructionsList'][0]['id'] = parameterNr
        param['instructionsList'][0]['value']           = value
        param['instructionsList'][0]['instructionIdx']  = 0

        try:
            param['instructionsList'][0]['dim0'] = dimension[0]
        except:
            param['instructionsList'][0]['dim0'] = 0

        try:
            param['instructionsList'][0]['dim1'] = dimension[1]
        except:
            param['instructionsList'][0]['dim1'] = 0

        return param






