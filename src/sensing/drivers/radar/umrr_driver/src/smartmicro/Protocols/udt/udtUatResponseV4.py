import struct
import crc16

from copy import deepcopy

from smartmicro.Protocols.uat.uatMain import DataFormat, MessageType
from smartmicro.Protocols.udt.udtUatResponseMain import UAT_RespErrorCode


class UATv4Response:
    """
    The class provides encode and decode functions of UDT type 17000 version 5 which is an UAT Response Message for UAT
    message version 4. In UDT type 17000 version 5 several instructions can be sent in one block. Each instruction
    consists of 2 messages. Example: 3 instructions are to be sent in one block. The value of the number of instructions
    would thus be 3 and the following messages would be transmitted:
    * 1 x header + 3 * 3 (messages for the 3 instructions) = 10 messages
    * 1 x header + 3 * N. N = number of instructions.

    The structure of the single messages can be found in the following table:

    +--------------+------------------------+---------------------+---------------------+---------------------+
    | message      | header                 | instruction msg 1   | instruction msg 2   | instruction msg 3   |
    +==============+========================+=====================+=====================+=====================+
    | Byte 0       |                        |                     |                     |                     |
    +--------------+ UDT-Index              | UDT-Index           | UDT-Index           | UDT-Index           |
    | Byte 1       |                        |                     |                     |                     |
    +--------------+------------------------+---------------------+---------------------+---------------------+
    | Byte 2       |                        | Counter             | Counter             | Counter             |
    +--------------+ Version                |---------------------+---------------------+---------------------+
    | Byte 3       |                        | Message type        | Result              | Data format         |
    +--------------+------------------------+---------------------+---------------------+---------------------+
    | Byte 4       | Device ID              |                     |                     | Dim 0               |
    +--------------+------------------------+ UAT-ID              |                     |---------------------+
    | Byte 5       | Number of instructions |                     |                     | Dim 1               |
    +--------------+------------------------+---------------------+ Parameter value     |---------------------+
    | Byte 6       |                        |                     |                     | Not used            |
    +--------------+ crc16 value            | Parameter number    |                     |---------------------+
    | Byte 7       |                        |                     |                     | Not used            |
    +--------------+------------------------+---------------------+---------------------+---------------------+

    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __init__                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self):
        self.data = dict()
        self.data['UDT-Index'] = 17000
        self.data['version'] = 5
        self.data['deviceId'] = 0
        self.data['numberOfInstructions'] = 1
        self.data['instructionsList'] = [dict()] * self.data['numberOfInstructions']
        self.data['instructionsList'][0]['UAT-ID'] = 0
        self.data['instructionsList'][0]['result'] = UAT_RespErrorCode.SUCCESS
        self.data['instructionsList'][0]['dataFormat'] = DataFormat.DEFAULT
        self.data['instructionsList'][0]['messageType'] = MessageType.DEFAULT
        self.data['instructionsList'][0]['parameterNumber'] = 0
        self.data['instructionsList'][0]['counter'] = 0
        self.data['instructionsList'][0]['value'] = 0
        self.data['instructionsList'][0]['dim0'] = 0
        self.data['instructionsList'][0]['dim1'] = 0

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getDataPackage                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getDataPackage(self):
        """
        Returns a dict with default values.

        Returns
        -------
        data        : dict
            the dictionary with the data information
        """
        return self.data

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeHeaderMessage                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeHeaderMessage(data, crcCalcDataArray):
        """
        The function encodes the arguments to a byte array with 8 bytes.

        Parameters
        ----------
        data                : dict
            instruction header dictionary
        crcCalcDataArray        : list
            data for the calculation of the crc value

        Returns
        -------
        headerMessage       : bytearray
            encoded header message

        """
        # provide raw header message
        headerMessage = bytearray(8)
        # Encode the UDT-Index
        headerMessage[0:2] = struct.pack('<H', data['UDT-Index'])
        # Encode the version number
        headerMessage[2:4] = struct.pack('<H', data['version'])
        # Encode the device ID
        headerMessage[4] = data['deviceId']
        # Encode the number of instructions
        headerMessage[5] = data['numberOfInstructions']
        # Calculate the crc value. Over instruction message 1-3 for all instructions. Header is ignored for CRC
        # calculation.
        initCrc = 0xFFFF
        calcCRC = crc16.crc16xmodem(bytes(crcCalcDataArray), initCrc)
        # Encode the crc value
        headerMessage[6:8] = struct.pack('<H', calcCRC)

        return headerMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeHeaderMessage                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeHeaderMessage(headerMessage, crcCalcDataArray):
        """
        The function decodes the header message and check that the crc value is correct. Returns the UDT-Index, the
        version, the device ID and the number of instructions.

        Parameters
        ----------
        headerMessage           : bytearray
            encoded header message
        crcCalcDataArray        : list
            data for the calculation of the crc value

        Returns
        -------
        A list with the UDT-Index, the version, the device ID and the number of instructions.

        Raises
        ------
        ValueError:
            if the received crc and the calculated crc value mismatch.

        """
        # provide data dictionary
        dataDict = dict()
        # Decode the UDT-Index
        dataDict['UDT-Index']            = struct.unpack('<H', headerMessage[0:2])[0]
        # Decode the version number
        dataDict['version']              = struct.unpack('<H', headerMessage[2:4])[0]
        # Decode the device ID
        dataDict['deviceId']             = struct.unpack('<B', headerMessage[4:5])[0]
        # Decode the number of instructions
        dataDict['numberOfInstructions'] = struct.unpack('<B', headerMessage[5:6])[0]

        # Calculate the crc value
        calcCRC = crc16.crc16xmodem(bytes(crcCalcDataArray), 0xFFFF)
        # Decode the crc value
        recvCRC = struct.unpack('<H', headerMessage[6:8])[0]
        # check and compare received and
        if calcCRC != recvCRC:
            raise ValueError("UDT_UATv4Response decodeHeaderMessage: CRC error calc: {}, received {}"
                             .format(calcCRC, recvCRC))
        return dataDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeParameterMessage                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeParameterMessage(UDT_Index, singleInst):
        """
        The function encodes the arguments to a byte array with 8 bytes.

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        singleInst          : dict
            contains the currently used instruction set

        Returns
        -------
        parameterMessage    : bytearray
            encoded parameter message
        """
        # provide raw message
        parameterMessage = bytearray(8)
        # encode message
        parameterMessage[0:2] = struct.pack('<H', UDT_Index                      )
        parameterMessage[2:3] = struct.pack('<B', singleInst['counter']          )
        parameterMessage[3:4] = struct.pack('<B', singleInst['messageType'].value)
        parameterMessage[4:6] = struct.pack('<H', singleInst['UAT-ID']           )
        parameterMessage[6:8] = struct.pack('<H', singleInst['parameterNumber']  )

        return parameterMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeParameterMessage                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeParameterMessage(encMsg, singleInst):
        """
        The function decodes the parameter(instruction1) message. Returns a list with the UDT-Index, the counter, the
        message type, the uat-id and the parameter number.

        Parameters
        ----------
        encMsg      : bytearray
            encoded parameter message
        singleInst  : dict
            dictionary with instruction set

        Returns
        -------
        The list contains the UDT-Index, the counter, the message type, the uat-id and the parameter number.

        """
        # Decode the message type
        singleInst['messageType']     = MessageType(encMsg[3])
        # Decode the UAT-ID
        singleInst['UAT-ID']          = struct.unpack('<H', encMsg[4:6])[0]
        # Decode the parameter number
        singleInst['parameterNumber'] = struct.unpack('<H', encMsg[6:8])[0]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeParameterValueMessage                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeParameterValueMessage(UDT_Index, singleInst):
        """
        The function encodes the arguments to a byte array with 8 bytes.

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        singleInst          : dict
            contains the currently used instruction set

        Returns
        -------
        parameterValueMessages      : bytearray
            encoded parameter value message
        """
        # check rule set for decoding
        if singleInst['dataFormat'] == DataFormat.INTEGER:
            encRule = "<I"
        else:
            encRule = "<f"
        # provide raw message
        parameterValueMessage = bytearray(8)
        # encode message
        parameterValueMessage[0:2] = struct.pack(    '<H', UDT_Index                  )
        parameterValueMessage[2:3] = struct.pack(    '<B', singleInst['counter']      )
        parameterValueMessage[3:4] = struct.pack(    '<B', singleInst['result'].value )
        parameterValueMessage[4:8] = struct.pack( encRule, singleInst['value']        )

        return parameterValueMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeParameterValueMessage                                                                            #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeParameterValueMessage(encMsg, singleInst):
        """
        The function decodes the parameter value(instruction2) message. Returns a list with the UDT-Index, the counter,
        the result and the parameter value.

        Parameters
        ----------
        encMsg      : bytearray
            encoded parameter message
        singleInst  : dict
            dictionary with instruction set

        Returns
        -------
            A list contains the UDT-Index, the counter, the result and the parameter value

        Raises
        ------
        ValueError:
            if the data format does not specify int or float type.

        """
        # check rule set for decoding
        if singleInst['dataFormat'] == DataFormat.INTEGER:
            decRule = "<I"
        else:
            decRule = "<f"
        # Decode the result
        singleInst['counter'] = encMsg[2]
        # Decode the result
        singleInst['result']  = UAT_RespErrorCode(encMsg[3])
        # Decode the value
        singleInst['value']   = struct.unpack(decRule, encMsg[4:8])[0]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeDimMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeDimMessage(UDT_Index, singleInst):
        """
        The function encodes the arguments to a byte array with 8 bytes.

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        singleInst          : dict
            contains the currently used instruction set

        Returns
        -------
        dimMessage          : bytearray
            encoded dim message

        """
        # provide raw message
        dimMessage = bytearray(8)
        # encode message
        dimMessage[0:2] = struct.pack( '<H', UDT_Index                      )
        dimMessage[2:3] = struct.pack( '<B', singleInst['counter']          )
        dimMessage[3:4] = struct.pack( '<B', singleInst['dataFormat'].value )
        dimMessage[4:5] = struct.pack( '<B', singleInst['dim0']             )
        dimMessage[5:6] = struct.pack( '<B', singleInst['dim1']             )

        return dimMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeDimMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeDimMessage(encMsg, singleInst):
        """
        The function decodes the dim message.

        Parameters
        ----------
        encMsg      : bytearray
            encoded parameter message
        singleInst  : dict
            dictionary with instruction set

        Returns
        -------
        A list contains the UDT-Index, the counter, the data format, dim 0 and dim 1.

        """
        # Decode the data format
        singleInst['dataFormat'] = DataFormat(encMsg[3])
        # Decode the dim A
        singleInst['dim0']       = encMsg[4]
        # Encode the dim B
        singleInst['dim1']       = encMsg[5]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __headerValueCheck                                                                                     #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __headerValueCheck(cls, data):
        # provide check list and error messages
        checkList = [{"key": "version"  , "value":     5, "errorCode": "UDT: UAT response version error: "
                                                                       " 5 (expected) | {} (received)"},
                     {"key": "UDT-Index", "value": 17000, "errorCode": "UDT: UAT response index error: "
                                                                       " 17000 (expected) | {} (received)"}]
        # run through all checks
        for singleCheck in checkList:
            # execute check
            if data[singleCheck["key"]] != singleCheck["value"]:
                raise ValueError(singleCheck["errorCode"].format(data[singleCheck["key"]]))

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __headerTypeCheck                                                                                      #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __headerTypeCheck(cls, data):
        # provide check list and error messages
        checkList = [{"key": "UDT-Index", "type": int, "errorCode": "UDT: UAT response index type error: "
                                                                    " int (expected) | {} (received)"}]
        # run through all checks
        for singleCheck in checkList:
            # execute check
            if type(data[singleCheck["key"]]) is not singleCheck["type"]:
                raise ValueError(singleCheck["errorCode"].format(str(type(data[singleCheck["key"]]))))

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __instructionTypeCheck                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __instructionTypeCheck(cls, data):
        # provide check list and error messages
        checkList = [{"key": "parameterNumber", "type": int, "errorCode": "UDT: UAT response param number type error: "
                                                                          " int (expected) | {} (received)"},
                     {"key": "dim0"           , "type": int, "errorCode": "UDT: UAT response dimension 0 type error: "
                                                                          " int (expected) | {} (received)"},
                     {"key": "dim1"           , "type": int, "errorCode": "UDT: UAT response dimension 1 type error: "
                                                                          " int (expected) | {} (received)"},
                     {"key": "UAT-ID"         , "type": int, "errorCode": "UDT: UAT response uat index type error: "
                                                                          " int (expected) | {} (received)"}]
        # run through all checks
        for singleCheck in checkList:
            # execute check
            if type(data[singleCheck["key"]]) is not singleCheck["type"]:
                raise ValueError(singleCheck["errorCode"].format(str(type(data[singleCheck["key"]]))))

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def encode(cls, data):
        """
        Encodes a dictionary of UDT type 17000 version 5

        Parameters
        ----------
        data            : dict
            A dict with the keys: 'UDT-Index', 'version', 'UAT-ID', 'result', 'deviceId' 'parameterNumber',
            'messageType', 'dataFormat', 'counter', 'numberOfInstructions' 'dim0', 'dim1' and 'value'.

        Returns
        -------
        messageList     : list
            (1 + 3 * numberOfInstructions) byte objects representing the UAT Response Message for UAT message version 1

        Raises
        ------
        ValueError:
            if the UDT index does not match the expected one (17000)
            if the version is not equal 5
        TypeError:
            if the type from the udt-index, uat-index, parameterNumber, dim0 or dim1 is not int
            if the length form the data['instructionsList'] and the value from the data['numberOfInstructions'] mismatch

        """
        # execute header value check
        cls.__headerValueCheck(data)
        # execute header type check
        cls.__headerTypeCheck(data)
        # check size of list
        if len(data['instructionsList']) != data["numberOfInstructions"]:
            raise TypeError("number of instruction and length of instructions list mismatch")

        # provide list of messages
        messageList = []
        # run through all instructions
        for singleInst in data['instructionsList']:
            #
            cls.__instructionTypeCheck(singleInst)
            # Encode the instruction message 1 (parameter message)
            messageList.append(UATv4Response.encodeParameterMessage(data['UDT-Index'] + 16, singleInst))
            # Encode the instruction message 2 (parameter value message)
            messageList.append(UATv4Response.encodeParameterValueMessage(data['UDT-Index'] + 17, singleInst))
            # Encode the instruction message 3 (dim message)
            messageList.append(UATv4Response.encodeDimMessage(data['UDT-Index'] + 18, singleInst))
        # Encode the header and calculate the crc value
        # Calculate the crc value. Over instruction message for all instructions. Header is ignored for CRC
        crcCalcDataArray = []
        # Write all instruction message in an array to calculate the crc value
        for message in messageList:
            crcCalcDataArray.extend(message[0:8])
        # encode header
        header = UATv4Response.encodeHeaderMessage(data, crcCalcDataArray)
        # Add the header to the first position
        messageList.insert(0, header)

        return messageList

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def decode(cls, msgList):
        """
        Decodes a binary message of UDT type 17000 version 5 which is an UAT Response Message for UAT message version 4

        Parameters
        ----------
        msgList     : bytearray
            (1 + 3 * numberOfInstructions) byte objects representing the UAT Response Message for UAT message version 1

        Returns
        -------
        data        : dict
            A dict with the keys: 'UDT-Index', 'version', 'UAT-ID', 'result', 'deviceId' 'parameterNumber',
            'messageType', 'dataFormat', 'counter', 'numberOfInstructions' 'dim0', 'dim1' and 'value'.

        Raises
        ------
        ValueError:
            if the UDT index does not match the expected one (17000)
            if the version is not equal 5
            if the counters from the instruction message does not match.
        TypeError:
            if the length from each message is not equal 8.
            if the number of messages is not equal (1 + 3 * numberOfInstructions).
        """
        # Checks that the msgList is correct ( One header and n * 3 instructions messages )
        if len(msgList) % 3 != 1:
            # raise error for incomplete package
            raise TypeError('UDT_UATv4Response decode: The message list has not 1 + 2 * n messages. len(msgList):{}'
                            .format(len(msgList)))
        # provide empty header
        header           = bytearray(8)
        # provide array for all body messages for further crc calculation
        crcCalcDataArray = []
        # run through every received message
        for messageIdx in range(len(msgList)):
            # check if header message is active
            if messageIdx == 0:
                # store header message
                header = msgList[messageIdx]
            else:
                crcCalcDataArray.extend(msgList[messageIdx][0:8])
            # check that all messages have 8 byte length
            if len(msgList[messageIdx]) != 8:
                raise TypeError('UDT_UATv4Response decode: The input message list is corrupt. len(msgList):{}'
                                .format(len(msgList)))
        # decode header message
        data = UATv4Response.decodeHeaderMessage(bytearray(header), crcCalcDataArray)
        # check basic header data
        cls.__headerValueCheck(data)
        # remove header message
        msgList.pop(0)
        # provide instruction list
        data['instructionsList'] = list()
        # provide decoding rule
        decRuleDict = { 17016 : UATv4Response.decodeParameterMessage     ,
                        17017 : UATv4Response.decodeParameterValueMessage,
                        17018 : UATv4Response.decodeDimMessage           }
        # provide decoding order
        decOrderList = [17016, 17018, 17017]
        # run through all instructions
        for instructionIdx in range(data['numberOfInstructions']):
            # provide dict for current message set
            messageDict = dict()
            # extract 3 message block from the message list
            for loopIdx in range(3):
                # get message
                message  = msgList[instructionIdx * 3 + loopIdx]
                # decode udt index
                udtIndex = struct.unpack('<H', message[0:2])[0]
                # append data to messageDict
                messageDict[udtIndex] = deepcopy(message)
            # provide single instruction dictionary
            singleInst = dict()
            # run through decoding order
            for decIndex in decOrderList:
                # check if index is available in both data sets
                if decIndex in decRuleDict.keys() and \
                   decIndex in messageDict.keys():
                    # decode message
                    decRuleDict[decIndex](messageDict[decIndex], singleInst)
                else:
                    raise ValueError("UDT: uat response: missing udt index {} within the given data!".format(decIndex))
            # append new instruction to decoded message
            data['instructionsList'].append(deepcopy(singleInst))

        return data