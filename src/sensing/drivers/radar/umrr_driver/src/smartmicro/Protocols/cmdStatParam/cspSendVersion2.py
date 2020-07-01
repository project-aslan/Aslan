from smartmicro.Protocols.uat.uatMain import DataFormat, MessageType


class cspSendVersion2:

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: buildMessageDict                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def buildMessageDict(cls, sectionName, paramName, parameters, value, element, msgType, deviceId):
        """
        The function encodes the uat version 4.

        Parameters
        ----------
        sectionName : string
            section name of used parameter
        paramName   : string
            name of used parameter
        parameters  : dictionary
            dictionary with section and parameter information
        value       : integer or float
            current value to be set
        element     : integer/list of integer
            dimensional data
        msgType     : MessageType
            assign command/status/parameter message
        deviceId    : int
            device id

        Returns
        -------
        param       : dictionary
            dictionary with uat data
        """
        param = dict()
        # define uat version
        param["uatVersion"]             = 4
        # set section of parameter set
        param['UAT-ID']                 = parameters[sectionName]['section']
        # set default device ID
        param['deviceId']               = deviceId
        # set default number of instructions
        param['numberOfInstructions']   = 0
        # provide default instruction list
        param['instructionsList']       = list()

        # work on multi parameter tool
        if type(paramName) is list:

            for parameter in paramName:
                # check existence of value
                if "value" in parameter:
                    value = parameter["value"]
                else:
                    value = 0x0
                # check existence of dimension 0
                if "dim0" in parameter:
                    dim0 = parameter["dim0"]
                else:
                    dim0 = 0
                # check existence of dimension 1
                if "dim1" in parameter:
                    dim1 = parameter["dim1"]
                else:
                    dim1 = 0
                # get parameter dictionary
                paramDict = cls.extractSingleParamDict(parameter["name"], sectionName, parameters, msgType)
                # add instruction
                param['instructionsList'].append(cls.addInstruction(parameters[sectionName]['section'],paramDict, value,
                                                                    msgType, dim0, dim1, param['numberOfInstructions']))

                param['numberOfInstructions'] += 1

        # work on single parameter tool
        elif type(paramName) is not list:

                dim0 = 0
                dim1 = 0

                if type(element) is list:
                    dim0 = element[0]
                    dim1 = element[1]
                elif element is not None:
                    dim0 = element

                # get parameter dictionary
                paramDict = cls.extractSingleParamDict(paramName, sectionName, parameters, msgType)
                # add instruction
                param['instructionsList'].append(cls.addInstruction(parameters[sectionName]['section'], paramDict,
                                                                    value, msgType, dim0, dim1, 0))

                param['numberOfInstructions'] += 1

        else:
            raise BaseException("Error in current UAT configuration!")

        return param

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: addInstruction                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def addInstruction(cls, section, paramDict, value, msgType, dim0, dim1, instIdx):
        """
        The function provides a instruction set for uat version 4 messages.

        Parameters
        ----------
        section     : integer
            parameter section index
        paramDict   : dictionary
            parameter specific data
        value       : integer/float
            value to be set
        msgType     : MessageType
            assign parameter/status/command message
        dim0        : integer
            first dimension index
        dim1        : integer
            second dimension index
        instIdx     : integer
            running index of current instruction

        Returns
        -------
        instruction : dictionary
            set of uat msg instructions
        """
        # provide empty instruction dictionary
        instruction = dict()
        # fill instruction
        instruction["section"]          = section
        instruction["messageType"]      = msgType
        instruction["dataFormat"]       = cls.evaluateFormat(paramDict["type"])
        instruction["id"]               = paramDict["id"]
        instruction["value"]            = value
        instruction["dim0"]             = dim0
        instruction["dim1"]             = dim1
        instruction["instructionIdx"]   = instIdx

        return instruction
    # ---------------------------------------------------------------------------------------------------------------- #
    # function: evaluateFormat                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def evaluateFormat(cls, paramType):
        """
        The function evaluates float or integer parameter type.

        Parameters
        ----------
        paramType   : string
            string to assign float or integer

        Returns
        -------
        retType     : DataFormat
            assign float or integer
        """
        retType = DataFormat.INTEGER

        if paramType == "f32":
            retType = DataFormat.FLOAT_IEEE

        return retType

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: extractSingleParamDict                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def extractSingleParamDict(cls, paramName, sectionName, paramDict, msgType):
        """
        The function extracts data from the dictionary.

        Parameters
        ----------
        paramName       : string
            name of the parameter
        sectionName     : string
            name of the parameter section
        paramDict  : dictionary
            dictionary with section and parameter information
        msgType         : MessageType
            type of message (status/command/parameter)

        Returns
        -------
        paramDict       : dictionary
            dictionary with parameter information
        """
        pList = paramDict[sectionName][cls.dictEntry(msgType)]
        paramDictList = [d for d in pList if d.get('name') == paramName]
        if len(paramDictList) == 0:
            raise KeyError("Parameter with name {}.{} does not exist".format(sectionName, paramName))

        if len(paramDictList) > 1:
            raise ValueError("Parameter with doublicated name detected {}".format(paramDictList))
        paramDict = paramDictList[0]
        return paramDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: dictEntry                                                                                              #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def dictEntry(cls, msgType):
        """
        This function returns the message type as string.

        Parameters
        ----------
        msgType     : MessageType
            type of message (status/command/parameter)

        Returns
        -------
        name        : str or none
            returns none for message type command or the message type name

        """
        if msgType is MessageType.STATUS:
            name = "status"
        elif msgType is MessageType.COMMAND:
            name = None
        else:
            name = "parameters"

        return name