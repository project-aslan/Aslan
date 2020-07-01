import struct
import crc16
from enum import Enum, unique
from smartmicro.Protocols.uat.uatMain import ParameterType


@unique
class UAT_RespErrorCode(Enum):
    SUCCESS = 0
    UNKNOWN_ERROR = 1
    REQUESTED_PARAM_INVALID = 2
    REQUESTED_ACTION_INVALID = 3
    REQUESTED_VERSION_INVALID = 4
    CRC_ERROR = 5
    UAT_INDEX_INVALID = 6
    INVALID_TYPE = 7
    INVALID_ID = 8
    INVALID_DIMENSION = 9
    RESERVED_UPPER_LAYER = 10
    INVALID_PARAM_VALUE_NAN = 11
    INVALID_PARAM_VALUE_TOO_SMALL = 12
    INVALID_PARAM_VALUE_TOO_BIG = 13
    INVALID_PARAM_VALUE_OUT_OF_BOUND = 14
    PARAM_IS_READ_PROTECTED = 15
    PARAM_IS_WRITE_PROTECTED = 16
    INCOMPLETE_PACKAGE = 17
    INVALID_DATA_TYPE = 18
    TIMEOUT = 19
    BUFFER_OVERRUN = 20
    PACKET_ERROR = 21
    INSUFFICIENT_ACCESS_LEVEL = 22
    COMMAND_EXEC_ERROR = 23
    UNSUPPORTED_CAN_IDENTIFIER =24
    CAN_DLC_ERROR = 25
    DEFAULT = None


@unique
class UAT_ErrorHandler(Enum):
    NO_ERROR = 0                    #: normal mode
    DELETE_FIRST_CAN_MESSAGE = 1    #: delete the first can message
    DELETE_LAST_CAN_MESSAGE = 2     #: delete the last can message
    DELETE_RANDOMLY_CAN_MESSAGE = 3 #: delete one can message from the can message list
    UATv4_INVALID_MESSAGE_TYPE = 4
    UATv4_INVALID_UAT_VERSION = 5
    UATv4_INVALID_CRC_VALUE = 6     #: manipulate the crc value
    UATv4_INVALID_INSTRUCTION_INDEX = 7     #: manipulate the instruction index
    UATv4_INVALID_PARAM_NAN = 8     #: E.g wrong float
    UATv4_INVALID_DATA_TYPE = 9


class UAT_ResponseMain:
    """
    The class provides encode and decode functions of UAT response messages for versions 1 to 3.
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __init__                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self):
        self.data                    = dict()
        self.data['UDT-Index']       = 17000
        self.data['deviceId']        = 0
        self.data['parameterType']   = ParameterType.DEFAULT
        self.data['parameterNumber'] = 0
        self.data['UAT-ID']          = 0
        self.data['value']           = 0
        self.data['version']         = 0
        self.data['result']          = UAT_RespErrorCode.DEFAULT
        self.data['counter']         = 0

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getDataPackage                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getDataPackage(self):
        """
        This function returns the dict data.

        Returns
        -------
        data    : dictionary
            the data package

        """
        return self.data

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeHeaderMessage                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeHeaderMessage(UDT_Index, version):
        """
        The function encodes the arguments to a byte array with 8 bytes. The header message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Version                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       | Not used                 |
        +--------------+--------------------------+
        | Byte 5       | Not used                 |
        +--------------+--------------------------+
        | Byte 6       | Not used                 |
        +--------------+--------------------------+
        | Byte 7       | Not used                 |
        +--------------+--------------------------+

        Parameters
        ----------
        UDT_Index   : int
            the udt index
        version     : int
            the "UAT Read Parameter" versions number

        Returns
        -------
        header      : bytearray
            the encoded header message

        """
        header = bytearray(8)
        # Encode the UDI Index
        header[0:2] = struct.pack('<H', UDT_Index)
        # Encode the version
        header[2:4] = struct.pack('<H', version)
        # Data byte 4-7 are unused
        return header

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeHeaderMessage                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeHeaderMessage(headerMessage):
        """
        The function decodes the header message. The header message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Version                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       | Not used                 |
        +--------------+--------------------------+
        | Byte 5       | Not used                 |
        +--------------+--------------------------+
        | Byte 6       | Not used                 |
        +--------------+--------------------------+
        | Byte 7       | Not used                 |
        +--------------+--------------------------+

        Parameters
        ----------
        headerMessage   : bytearray
            the encoded header message

        Returns
        -------
        A list with the UDT-Index and the version.

        """
        # Decode the UDI-Index
        UDI_Index = struct.unpack('<H', headerMessage[0:2])[0]
        # Decode the version
        version = struct.unpack('<H', headerMessage[2:4])[0]
        # Data byte 4-7 are unused
        return [UDI_Index, version]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeParameterMessage                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeParameterTypeMessage(UDT_Index, counter, result, deviceId, parameterType, parameterNumber):
        """
        The function encodes the arguments to a byte array with 8 bytes. The parameter(instruction1) message is defined:

        +--------------+-----------------------------+
        | Byte 0       |                             |
        +--------------+ UDT-Index                   |
        | Byte 1       |                             |
        +--------------+-----------------------------+
        | Byte 2       |                             |
        +--------------+ Counter                     |
        | Byte 3       |                             |
        +--------------+-----------------------------+
        | Byte 4       | Result                      |
        +--------------+-----------------------------+
        | Byte 5       | Device ID                   |
        +--------------+-----------------------------+
        | Byte 6       | - Bit 0-5: Parameter type   |
        |              | - Bit 6-7: Parameter number |
        +--------------+-----------------------------+
        | Byte 7       | Not used                    |
        +--------------+-----------------------------+

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        counter             : int
            number of parameters transmitted. Start at 0, increment by 1 for each additional parameter
        result              : int
            return value or error code. See the class UAT_RespErrorCode (udtUatResponseMain.py)
        deviceId            : int
            target device ID
        parameterType       : int
            specifies the format of the data
        parameterNumber     : int
            parameter number

        Returns
        -------
        parameterMessage    : bytearray
            the encoded parameter message

        """
        parameterMessage = bytearray(8)
        # Encode the UDT-Index
        parameterMessage[0:2] = struct.pack('<H', UDT_Index)
        # Encode the counter
        parameterMessage[2:4] = struct.pack('<H', counter)
        # Encode the result
        parameterMessage[4] = result
        # Encode the deviceId
        parameterMessage[5] = deviceId
        # Encode the parameter type (bit 0-5) and parameter number (bit 6-7)
        parameterMessage[6] = ((parameterNumber & 0x3) << 6) | (parameterType & 0x3f)
        parameterMessage[7] = (parameterNumber >> 2) & 0xFF
        return parameterMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeParameterMessage                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeParameterTypeMessage(parameterTypeMessage):
        """
        The function decodes the parameter type message. The parameter type message is defined:

        +--------------+-----------------------------+
        | Byte 0       |                             |
        +--------------+ UDT-Index                   |
        | Byte 1       |                             |
        +--------------+-----------------------------+
        | Byte 2       |                             |
        +--------------+ Counter                     |
        | Byte 3       |                             |
        +--------------+-----------------------------+
        | Byte 4       | Result                      |
        +--------------+-----------------------------+
        | Byte 5       | Device ID                   |
        +--------------+-----------------------------+
        | Byte 6       | - Bit 0-5: Parameter type   |
        |              | - Bit 6-7: Parameter number |
        +--------------+-----------------------------+
        | Byte 7       | Not used                    |
        +--------------+-----------------------------+

        Parameters
        ----------
        parameterTypeMessage        : bytearray
            the encoded parameter message

        Returns
        -------
        A list contains the UDT-Index, the counter, the result, the device ID, the parameter type and the parameter number.

        """
        # Decode the UDT-Index
        index = struct.unpack('<H', parameterTypeMessage[0:2])[0]
        # Decode the counter
        counter = struct.unpack('<H', parameterTypeMessage[2:4])[0]
        # Decode the result
        result = UAT_RespErrorCode(parameterTypeMessage[4])
        # Decode the deviceId
        deviceId = parameterTypeMessage[5]
        # The parameter type is the CAN data byte 6 (bit 0-5)
        parameterType = ParameterType(parameterTypeMessage[6] & 0x3f)
        # The parameter number is the CAN data byte 6 (bit 6-7) and data byte 7
        parameterNumber = (parameterTypeMessage[6] >> 6) | (parameterTypeMessage[7] << 2)

        return [index, counter, result, deviceId, parameterType, parameterNumber]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeCRCMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeCRCMessage(UDT_Index, counter, UAT_ID, crcCalcDataArrayBefore, crcCalcDataArrayAfter):
        """
        The function encodes the arguments to a byte array with 8 bytes. The crc message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+ CRC16                    |
        | Byte 5       |                          |
        +--------------+--------------------------+
        | Byte 6       |                          |
        +--------------+ UAT ID                   |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        counter             : int
            number of parameters transmitted. Start at 0, increment by 1 for each additional parameter
        UAT_ID              : int
            uat id
        crcCalcDataArrayBefore  : bytearray
            data for the calculation of the crc that are before the crc message
        crcCalcDataArrayAfter   : bytearray
            data for the calculation of the crc that are after the crc message

        Returns
        -------
        crcMessage  : bytearray
            the encoded crc message

        """
        crcMessage = bytearray(8)

        # Encode the UDT-Index
        crcMessage[0:2] = struct.pack('<H', UDT_Index)
        # Encode the counter
        crcMessage[2:4] = struct.pack('<H', counter)
        # Encode the UAT-ID
        crcMessage[6:8] = struct.pack('<H', UAT_ID)

        crcCalcArray = crcCalcDataArrayBefore
        crcCalcArray.extend(crcMessage[2:4])
        crcCalcArray.extend(crcMessage[6:8])
        crcCalcArray.extend(crcCalcDataArrayAfter)
        # Calculate the crc
        initCrc = 0xFFFF
        calcCRC = crc16.crc16xmodem(bytes(crcCalcArray),initCrc)
        # Encode the crc value
        crcMessage[4:6] = struct.pack('<H', calcCRC)
        return crcMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeCRCMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeCRCMessage(crcMessage, crcCalcDataArrayBefore, crcCalcDataArrayAfter):
        """
        The function decodes the crc message and check that the received crc value is correct. The crc message is
        defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+ CRC16                    |
        | Byte 5       |                          |
        +--------------+--------------------------+
        | Byte 6       |                          |
        +--------------+ UAT ID                   |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        crcMessage              : bytearray
            the encoded crc message
        crcCalcDataArrayBefore  : bytearray
            data for the calculation of the crc that are before the crc message
        crcCalcDataArrayAfter   : bytearray
            data for the calculation of the crc that are after the crc message

        Returns
        -------
        A list contains the UDT-Index, the counter and the UAT-ID.

        Raises
        ------
        ValueError:
            if the received crc value and the calculated crc vale does not match.

        """
        # Decode the UDT-Index
        index = struct.unpack('<H', crcMessage[0:2])[0]
        # Decode the counter
        counter = struct.unpack('<H', crcMessage[2:4])[0]

        crcCalcArray = bytearray(crcCalcDataArrayBefore)
        crcCalcArray.extend(crcMessage[2:4])
        crcCalcArray.extend(crcMessage[6:8])
        crcCalcArray.extend(crcCalcDataArrayAfter)
        # Calculate the crc value
        initCrc = 0xFFFF
        calcCRC = crc16.crc16xmodem(bytes(crcCalcArray),initCrc)
        # Decode the crc value
        recvCRC = struct.unpack('<H', crcMessage[4:6])[0]
        # Check the received and calculate crc
        if calcCRC != recvCRC:
            raise ValueError("UAT_ResponseMain decodeCRCMessage: CRC error calc: {}, received {}".format(calcCRC, recvCRC))

        # Decode the UAT-Id
        UAT_Id = struct.unpack('<H', crcMessage[6:8])[0]
        return [index, counter, UAT_Id]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeValueMessage                                                                                     #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeValueMessage(UDT_Index, counter, parameterType, value):
        """
        The function encodes the arguments to a byte array with 8 bytes. The value message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+                          |
        | Byte 5       |                          |
        +--------------+ Value                    |
        | Byte 6       |                          |
        +--------------+                          |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        counter             : int
            number of parameters transmitted. Start at 0, increment by 1 for each additional parameter
        parameterType       : ParameterType
            specifies the format of the data
        value               : int
            parameter value

        Returns
        -------
        valueMessage    : bytearray
            the encoded value message

        Raises
        ------
        ValueError:
            if the parameter type does not specify int or float type.
        TypeError:
            if the parameter type and the type of the parameter value mismatch.

        """
        valueMessage = bytearray(8)
        # Encode the UDT-Index
        valueMessage[0:2] = struct.pack('<H', UDT_Index)
        # Encode the counter
        valueMessage[2:4] = struct.pack('<H', counter)

        # Encode the parameter value
        # int value
        if parameterType == ParameterType.INTEGER_WRITE or parameterType == ParameterType.INTEGER_READ \
            or parameterType == ParameterType.INTEGER_RW:
            valueMessage[4:8] = struct.pack('<I', value)
        # float value
        elif parameterType == ParameterType.IEEE_FLOAT_WRITE or parameterType == ParameterType.IEEE_FLOAT_READ \
            or parameterType == ParameterType.IEEE_FLOAT_RW:
            valueMessage[4:8] = struct.pack('<f', value)
        else:
            raise ValueError("UAT_ResponseMain encodeValueMessage: Unsupported parameter Type")

        return valueMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeValueMessage                                                                                     #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeValueMessage(valueMessage, parameterType):
        """
        The function decodes the parameter value message. The parameter value message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+                          |
        | Byte 5       |                          |
        +--------------+ Value                    |
        | Byte 6       |                          |
        +--------------+                          |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        valueMessage        : bytearray
            the encoded value message
        parameterType       : ParameterType
            specifies the format of the data

        Returns
        -------
        A list contains the UDT-Index, the counter and the parameter value

        Raises
        ------
        ValueError:
            if the data format does not specify int or float type.

        """
        # Decode the UDT-Index
        index = struct.unpack('<H', valueMessage[0:2])[0]
        # Decode the counter
        counter = struct.unpack('<H', valueMessage[2:4])[0]
        # Decode the parameter value
        # int value
        if parameterType == ParameterType.INTEGER_WRITE or parameterType == ParameterType.INTEGER_READ \
            or parameterType == ParameterType.INTEGER_RW:
            value = struct.unpack('<I', valueMessage[4:8])[0]
        # float value
        elif parameterType == ParameterType.IEEE_FLOAT_WRITE or parameterType == ParameterType.IEEE_FLOAT_READ \
             or parameterType == ParameterType.IEEE_FLOAT_RW:
            value = struct.unpack('<f', valueMessage[4:8])[0]
        else:
            raise TypeError("UAT_ResponseMain decodeValueMessage: unsupported parameter Type")
        return [index, counter, value]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeDimMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encodeDimMessage(UDT_Index, counter, dim0, dim1):
        """
        The function encodes the arguments to a byte array with 8 bytes. The dim message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+ Dim 0                    |
        | Byte 5       |                          |
        +--------------+--------------------------+
        | Byte 6       |                          |
        +--------------+ Dim 1                    |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        UDT_Index           : int
            the udt index
        counter             : int
            number of parameters transmitted. Start at 0, increment by 1 for each additional parameter
        dim0                : int
            for arrays in parameter structures, specifies which indexes should be used in which dimension.
        dim1                : int
            for arrays in parameter structures, specifies which indexes should be used in which dimension.

        Returns
        -------
        dimMessage          : bytearray
            the encoded message

        """
        dimMessage = bytearray(8)
        # Encode the UDT-Index
        dimMessage[0:2] = struct.pack('<H', UDT_Index)
        # Encode the counter
        dimMessage[2:4] = struct.pack('<H', counter)
        # Encode the dim0
        dimMessage[4:6] = struct.pack('<H', dim0)
        # Encode the dim1
        dimMessage[6:8] = struct.pack('<H', dim1)
        return dimMessage

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decodeDimMessage                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decodeDimMessage(dimMessage):
        """
        The function decodes the dim message. The dim message is defined:

        +--------------+--------------------------+
        | Byte 0       |                          |
        +--------------+ UDT-Index                |
        | Byte 1       |                          |
        +--------------+--------------------------+
        | Byte 2       |                          |
        +--------------+ Counter                  |
        | Byte 3       |                          |
        +--------------+--------------------------+
        | Byte 4       |                          |
        +--------------+ Dim 0                    |
        | Byte 5       |                          |
        +--------------+--------------------------+
        | Byte 6       |                          |
        +--------------+ Dim 1                    |
        | Byte 7       |                          |
        +--------------+--------------------------+

        Parameters
        ----------
        dimMessage          : bytearray
            the encoded message

        Returns
        -------
        A list contains the UDT-Index, the counter, dim 0 and dim 1.

        """
        # Decode the UDT-Index
        index = struct.unpack('<H', dimMessage[0:2])[0]
        # Decode the counter
        counter = struct.unpack('<H', dimMessage[2:4])[0]
        # Decode dim0
        dim0 = struct.unpack('<H', dimMessage[4:6])[0]
        # Decode dim1
        dim1 = struct.unpack('<H', dimMessage[6:8])[0]
        return [index, counter, dim0, dim1]