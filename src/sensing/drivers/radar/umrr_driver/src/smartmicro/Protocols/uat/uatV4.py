import struct
import crc16

from copy import deepcopy

from smartmicro.Protocols.uat.uatMain import DataFormat, MessageType


class UATv4:
    """
    The class provides encode and decode functions of UAT messages for versions 4. The internal structure of the uat
    version 4 format is given in the beneath documentation.

    The uat version 4 requests include a subset of header and instruction messages. The number of header messages per
    request is limited to one at the beginning. The number of instructions can be extended to the number the device
    under test can handle. The structure of the uat version 4 requests is given in the below chart:

    +--------------+--------------------+--------------+--------------------+--------------------+
    | Byte index   | Header message     | Byte index   | instruction msg 1  | instruction msg 2  |
    +--------------+--------------------+--------------+--------------------+--------------------+
    | Byte 0       |                    | Byte 0       |                    |                    |
    +--------------+ UAT-ID             +--------------+ section            | section            |
    | Byte 1       |                    | Byte 1       |                    |                    |
    +--------------+--------------------+--------------+--------------------+--------------------+
    | Byte 2       | Message index      | Byte 2       | Command index      | Command index      |
    +--------------+--------------------+--------------+--------------------+--------------------+
    | Byte 3       | Format version     | Byte 3       | Message type       | Data format        |
    +--------------+--------------------+--------------+--------------------+--------------------+
    | Byte 4       | Device id          | Byte 4       |                    | Parameter value    |
    +--------------+--------------------+--------------+ Parameter number   |                    |
    | Byte 5       | Number of commands | Byte 5       |                    |                    |
    +--------------+--------------------+--------------+--------------------+                    |
    | Byte 6       |                    | Byte 6       | Dim 0              |                    |
    +--------------+ crc16 value        |--------------+--------------------+                    |
    | Byte 7       |                    | Byte 7       | Dim 1              |                    |
    +--------------+--------------------+--------------+--------------------+--------------------+

    The initial header message is a single message and contain basic information about the following requests. The
    request itself is split into two different messages. The first one contains basic request information for the
    current parameter like command type ( like status, parameter or command request ) as well as parameter index,
    section information or dimensional data. The second contains the parameter value as well as interpretation data.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeHeader                                                                                           #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeHeader(UAT_ID, formatVersion, deviceId, numberOfInstructions, crcCalcDataArray):
        """
        The function encodes the arguments to a byte array with 8 bytes.

        Parameters
        ----------
        UAT_ID                  : int
            UAT index
        formatVersion           : int
            UAT version number
        deviceId                : int
            target device ID
        numberOfInstructions    : int
            in UAT format version 4 several instructions can be sent in one block. The argument specifies the number of
            instructions.
        crcCalcDataArray        : list
            data for the calculation of the crc value

        Returns
        -------
        header      : bytearray
            encoded header message

        """
        # Creates the header message
        header = bytearray(8)
        ## Message 1 (Header) of UAT Format version 1
        header[0:2] = struct.pack('<H', UAT_ID) # UAT-ID
        header[2]   = 0x0 # Message index
        header[3]   = formatVersion
        header[4]   = deviceId
        header[5]   = numberOfInstructions

        # The crc input data
        crcCalcArray = header[0:6]
        crcCalcArray.extend(crcCalcDataArray)
        # Calculates the crc
        initCrc = 0xFFFF
        calcCRC = crc16.crc16xmodem(bytes(crcCalcArray),initCrc)
        # Write the crc value into the message 1 header
        header[6:8] = struct.pack('<H', calcCRC)
        return header

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getHeaderInfo                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def getHeaderInfo(msg):
        """
        The function encodes the UAT-ID, the number of instructions and the device id.
        Parameters
        ----------
        msg     : bytearray

        Returns
        -------
        data    : dict
            A dictionary with the UAT-ID, the number of instructions and the device id.

        """
        data                         = dict()
        data['UAT-ID']               = struct.unpack('<H', msg[0:2])[0]
        data['numberOfInstructions'] = msg[5]
        data['deviceId']             = msg[4]

        return data

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeCommandMessage                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeCommandMessage(instructionData, commandIndex):
        """
        The function encodes the command to two bytearray.

        Parameters
        ----------
        instructionData : dictionary
            instruction dictionary with parameter request information
        commandIndex    : integer
            index of the command

        Returns
        -------
        The encoded command messages

        Raises
        ------
        TypeError:
            if the data format and the type of the parameter value mismatch.
        ValueError:
            if the data format does not specify int or float type.

        """
        # evaluate parameter format : integer
        if instructionData['dataFormat'].value == DataFormat.INTEGER.value :
            # set encoding rule : integer
            encRule = "<I"
        # evaluate parameter format : float
        elif instructionData['dataFormat'].value == DataFormat.FLOAT_IEEE.value :
            # set encoding rule : float
            encRule = "<f"
        else:
            raise ValueError("UATv4 encodeCommandMessage: unknown Parameter Type")

        # provide bytearray for the first message
        message1 = bytearray(8)
        # encode command message : first message
        message1[0:2] = struct.pack('<H'   , instructionData['section']           ) # section id
        message1[2:3] = struct.pack('<B'   , commandIndex                        ) # message id
        message1[3:4] = struct.pack('<B'   , instructionData['messageType'].value) # msg type
        message1[4:6] = struct.pack('<H'   , instructionData['id']               ) # parameter id
        message1[6:7] = struct.pack('<B'   , instructionData['dim0']             ) # dimension 0 id
        message1[7:8] = struct.pack('<B'   , instructionData['dim1']             ) # dimension 1 id

        # provide bytearray for the second message
        message2 = bytearray(8)
        # encode command message : second message
        message2[0:2] = struct.pack('<H'   , instructionData['section']          ) # section id
        message2[2:3] = struct.pack('<B'   , commandIndex + 1                   ) # message id
        message2[3:4] = struct.pack('<B'   , instructionData['dataFormat'].value) # data format
        message2[4:]  = struct.pack(encRule, instructionData['value']           ) # parameter value

        return[message1, message2]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encode(data):
        """
        Encodes the provided data into (1 + 2 * numberOfInstructions) byte messages according to the UATv4 format

        Parameters
        ----------
        data            : dict
            A dict with the keys: 'UAT-ID', 'deviceId', 'dataFormat', 'numberOfInstructions', 'instructionsList',
            'id', 'messageType', 'value', 'din0', and 'dim1'.

        Returns
        -------
        (1 + 2 * numberOfInstructions byte objects representing the uat_v4 messages

        Raises
        ------
        ValueError:
            if the 'numberOfInstructions' value is negative
        TypeError:
            if the type from the uat-index, id, dim0 or dim1 is not int
            if the length form the data['instructionsList'] and the value from the data['numberOfInstructions'] mismatch.

        """
        # Check the UAT-ID type
        if type(data['UAT-ID']) is not int:
            raise TypeError("UATv4 encode: UAT-ID must be of type int")
        # Check the number of commands
        if data['numberOfInstructions'] < 0:
            raise ValueError("UATv4 encode: The number of Instructions is less then 0. NumberOfInstructions{}"
                             .format(data['numberOfInstructions']))
        # Check that the commandList is correct
        if len(data['instructionsList']) != data['numberOfInstructions']:
            raise TypeError("UATv4 encode: The instructions list does not have the same number as numberOfCommands. "
                             "len(data['instructionsList']):{} data['numberOfInstructions']{}" .format(len(data['instructionsList']),
                                                                                            data['numberOfInstructions']))

        numberOfInstructions = data['numberOfInstructions']
        messageList = []

        # Create and encode the two messages for each command
        for index in range (0, numberOfInstructions):
            # The command data
            instructionData  = data['instructionsList'][index]
            instructionIndex = 1 + ( 2 * instructionData["instructionIdx"] )
            # Check the parameter number type
            if type(instructionData['id']) is not int:
                raise TypeError("UATv4 encode: ParameterNumber must be of type int")
            if type(instructionData['dim0']) is not int or type(instructionData['dim1']) is not int:
                raise TypeError("UATv4 encode: Dimension must be of type int")

            # Encode the command
            instructionMessage = UATv4.encodeCommandMessage(instructionData, instructionIndex)
            messageList.append(instructionMessage[0])
            messageList.append(instructionMessage[1])

        # Create a array with all command data to calculate the crc value
        crcCalcArray = []
        for message in messageList:
            crcCalcArray.extend(message[0:8])

        header = UATv4.encodeHeader(data['UAT-ID'], data["uatVersion"], data['deviceId'], data['numberOfInstructions'],
                                    crcCalcArray)
        # Add the header to the first position
        messageList.insert(0, header)

        return messageList

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def decode(cls, msgList):
        """
        The function handles the decoding of the entire uat request.

        Parameters
        ----------
        msgList : list of bytearray
            current messages to be decoded

        Returns
        -------
        data    : dictionary
            current uat request dictionary
        """
        # check input list
        for msg in msgList:
            # check if message is 8 bytes long
            if len(msg) != 8:
                raise TypeError("UAT V4 : message length mismatch")
        # provide data dictionary for uat data
        data                     = dict()
        # decode header data
        checkSum, checkSumData   = cls.__decodeHeader( msgList[0], data )
        # check if number of messages is valid
        if len(msgList) != ( ( data['numberOfInstructions'] * 2 ) + 1 ):
            # raise an error
            raise TypeError("UATv4 decode: The message list has not 1 + 2 * n messages.")
        # provide list of instructions
        data['instructionsList'] = list()
        # run through the whole instruction list
        for msgIdx in range( 1, len( msgList ), 2 ):
            # decode instruction
            instruction = cls.__decodeInstruction( msgList[ msgIdx ], msgList[ msgIdx + 1 ], checkSumData )
            # append new instruction
            data['instructionsList'].append( deepcopy(instruction) )

        cls.__calcCheckSum( checkSumData, checkSum )

        return data



    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __decodeHeader                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __decodeHeader(cls, headerMsg=None, data=None):
        """
        The function decodes the header information.

        Parameters
        ----------
        headerMsg   : bytearray
            header message
        data        : dictionary
            header data dictionary

        Returns
        -------
        checkSum        : integer
            current decoded check sum
        checkSumData    : bytearray
            bytearray for further crc calculation
        """
        # decode header data
        data['UAT-ID']               = struct.unpack("<H", headerMsg[0:2])[0]
        data["uatVersion"]           = struct.unpack("<B", headerMsg[3:4])[0]
        data['deviceId']             = struct.unpack("<B", headerMsg[4:5])[0]
        data['numberOfInstructions'] = struct.unpack("<B", headerMsg[5:6])[0]
        # decode check sum
        checkSum                     = struct.unpack("<H", headerMsg[6:8])[0]
        # provide check sum array
        checkSumData                 = bytearray(0)
        # fill in relevant data
        checkSumData.extend(headerMsg[0:6])

        return checkSum, checkSumData

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __decodeInstruction                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __decodeInstruction(cls, message1, message2, checkSumData):
        """
        The function decodes the basic instruction data from the given messages.

        Parameters
        ----------
        message1        : bytearray
            first instruction message
        message2        : bytearray
            second instruction message
        checkSumData    : bytearray
            bytearray for further crc calculation

        Returns
        -------
        instructionData : dict
            dictionary with current data set
        """
        # get coding rule dict
        codingDict = cls.__getCodingDict()
        # provide dictionary for current instruction set
        instructionData = dict()

        # decode : first message
        instructionData['section']         = struct.unpack('<H', message1[0:2])[0]
        instructionData['instructionIdx']  = int        ( ( struct.unpack('<B', message1[2:3] )[0] - 1 ) / 2)
        instructionData['messageType']     = MessageType(   struct.unpack('<B', message1[3:4] )[0] )
        instructionData['id'] = struct.unpack('<H', message1[4:6])[0]
        instructionData['dim0']            = struct.unpack('<B', message1[6:7])[0]
        instructionData['dim1']            = struct.unpack('<B', message1[7:8])[0]

        # decode : second message
        instructionData['dataFormat']      = DataFormat (   struct.unpack('<B', message2[3:4] )[0] )
        # decode parameter value
        instructionData['value']           = struct.unpack(codingDict[instructionData['dataFormat']], message2[4:8])[0]

        # append instruction data
        checkSumData.extend(message1[0:8])
        checkSumData.extend(message2[0:8])

        return instructionData


    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __getCodingDict                                                                                        #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __getCodingDict(cls):
        """
        The function returns coding rules.

        Returns
        -------
        codingDict  : dictionary
            coding rule dictionary
        """
        # provide empty coding dictionary
        codingDict = dict()
        # fill in all coding rules
        codingDict[DataFormat.INTEGER]    = "<I"
        codingDict[DataFormat.FLOAT_IEEE] = "<f"

        return codingDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __calcCheckSum                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    @classmethod
    def __calcCheckSum(cls, dataBytes=None, refCheckSum=None):
        """
        The function calculates the crc of the given bytearray and contains the possibility to compare the crc with the
        given reference value.

        Parameters
        ----------
        dataBytes   : bytearray
            bytearray as base for crc
        refCheckSum : integer
            refereence value

        Returns
        -------
        crcCalc     : integer
            calculated check sum
        """
        # configure crc calculation module
        initCrc = 0xFFFF
        crcCalc = crc16.crc16xmodem(bytes(dataBytes),initCrc)
        # check if reference is available
        if refCheckSum is not None:
            # compare reference
            if refCheckSum != crcCalc:
                # raise an error
                raise ValueError( "UATv4 CRC error: crc calc : {}, crc ref : {}".format(crcCalc, refCheckSum) )

        return crcCalc
