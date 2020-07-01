import struct
from enum import Enum, unique
import crc16


@unique
class ParameterType(Enum):
    DEFAULT = None
    INTEGER_WRITE = 0
    FLOAT_SMS_WRITE = 1
    INTEGER_READ = 2
    FLOAT_SMS_READ = 3
    INTEGER_RW = 4
    FLOAT_SMS_RW = 5
    IEEE_FLOAT_WRITE = 10
    IEEE_FLOAT_READ = 11
    IEEE_FLOAT_RW = 12


@unique
class MessageType(Enum):
    DEFAULT = None
    COMMAND = 0
    STATUS = 1
    PARAMETER_WRITE = 2
    PARAMETER_READ = 3
    PARAMETER_WRITE_READ = 4
    TEST_INVALID_MESSAGE_TYPE = 255


@unique
class DataFormat(Enum):
    DEFAULT = None
    INTEGER = 0
    FLOAT_SMS = 1
    FLOAT_IEEE = 2
    TEST_INVALID_FORMAT = 255


# class UATMainV1_V2_V3:
#     """
#     The class provides encode and decode functions of UAT messages for versions 1 to 3.
#
#     """
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: encodeHeader                                                                                           #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def encodeHeader(UAT_ID, formatVersion, parameterType, parameterNumber, crcCalcDataArray):
#         """
#         The function encodes the arguments to a byte array with 8 bytes. The header message is defined:
#
#         +--------------+------------------------------+
#         | Byte 0       |                              |
#         +--------------+ UAT-ID                       |
#         | Byte 1       |                              |
#         +--------------+------------------------------+
#         | Byte 2       | Message index                |
#         +--------------+------------------------------+
#         | Byte 3       | Format version               |
#         +--------------+------------------------------+
#         | Byte 4       | - Bit 0-5: Parameter number  |
#         |              | - Bit 6-7: Parameter type    |
#         +--------------+------------------------------+
#         | Byte 5       | Parameter type               |
#         +--------------+------------------------------+
#         | Byte 6       |                              |
#         +--------------+ crc16 value                  |
#         | Byte 7       |                              |
#         +--------------+------------------------------+
#
#         Parameters
#         ----------
#         UAT_ID              : int
#             the UAT index.
#         formatVersion       : int
#             the UAT version number
#         parameterType       : int
#             specifies the format of the data
#         parameterNumber     : int
#             data for the calculation of the crc value
#         crcCalcDataArray    : bytearray
#
#         Returns
#         -------
#         header              : bytearray
#             the encoded header message
#
#         """
#         # create the bytearray for the header message
#         header      = bytearray(8)
#         ## message 1 (Header) of UAT Format version 1
#         header[0:2] = struct.pack('<H', UAT_ID) # UAT-ID
#         header[2]   = 0x0 # Message index
#         header[3]   = formatVersion # Format version
#         # the first 6 bits is the parameter type and the last two bits is for the parameter Number
#         header[4]   = ((parameterNumber & 0x3) << 6) | (parameterType & 0x3f)
#         header[5]   = (parameterNumber >> 2) & 0xFF # Parameter number bits 2-10
#
#         # the crc input data
#         crcCalcArray = header[0:6]
#         crcCalcArray.extend(crcCalcDataArray)
#         # calculates the crc
#         initCrc = 0xFFFF
#         calcCRC = crc16.crc16xmodem(bytes(crcCalcArray),initCrc)
#         # encode the crc value into the message 1 header
#         header[6:8] = struct.pack('<H', calcCRC)
#         return header
#
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: decodeHeader                                                                                           #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def decodeHeader(headerMessage, crcCalcDataArray):
#         """
#         The function decodes the header message and check that the crc value is correct. Returns the parameter number
#         and type. The header message is defined:
#
#         +--------------+------------------------------+
#         | Byte 0       |                              |
#         +--------------+ UAT-ID                       |
#         | Byte 1       |                              |
#         +--------------+------------------------------+
#         | Byte 2       | Message index                |
#         +--------------+------------------------------+
#         | Byte 3       | Format version               |
#         +--------------+------------------------------+
#         | Byte 4       | - Bit 0-5: Parameter number  |
#         |              | - Bit 6-7: Parameter type    |
#         +--------------+------------------------------+
#         | Byte 5       | Parameter type               |
#         +--------------+------------------------------+
#         | Byte 6       |                              |
#         +--------------+ crc16 value                  |
#         | Byte 7       |                              |
#         +--------------+------------------------------+
#
#         Parameters
#         ----------
#         headerMessage       : bytearray
#             the encoded header message
#         crcCalcDataArray    : bytearray
#             data for the calculation of the crc value
#
#         Returns
#         -------
#         A list with the parameter number and parameter type and uatVersion
#
#         Raises:
#         -------
#         ValueError:
#             If the received crc value and the calculated crc vale does not match
#
#         """
#         # check that the crc value is correct
#         crcCalcArray = bytearray(headerMessage[0:6])
#         crcCalcArray.extend(crcCalcDataArray)
#         # calculate the crc value
#         initCrc = 0xFFFF
#         crcCalc = crc16.crc16xmodem(bytes(crcCalcArray),initCrc)
#         # the received crc value
#         crcRecv = struct.unpack('<H', headerMessage[6:8])[0]
#         # checks that the calculated and received crc value are correct
#         if crcCalc != crcRecv:
#             raise ValueError("UATMainV1_V2_V3 decodeHeader: CRC Mismatch CRC Calc:{}, crc recv:{}".format(crcCalc, crcRecv))
#
#         # the parameter number has a 10 bit length. The last two bits from the data byte 4 and the data byte 5.
#         parameterNumber = (headerMessage[4] >> 6) | (headerMessage[5] << 2)
#         # the parameter type has a 6 bit length from the data byte 4
#         parameterType = ParameterType(headerMessage[4] & 0x3f)
#         # decode version
#         uatVersion = headerMessage[3]
#
#         return [parameterNumber, parameterType, uatVersion]
#
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: encodeParameterMessage                                                                                 #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def encodeParameterMessage(UAT_ID, deviceId, parameterType, value):
#         """
#         The function encodes the arguments to a byte array with 8 bytes. The parameter message is defined:
#
#         +--------------+--------------------------+
#         | Byte 0       |                          |
#         +--------------+ UAT-ID                   |
#         | Byte 1       |                          |
#         +--------------+--------------------------+
#         | Byte 2       | Message index            |
#         +--------------+--------------------------+
#         | Byte 3       | Device ID                |
#         +--------------+--------------------------+
#         | Byte 4       | Parameter value          |
#         +--------------+                          |
#         | Byte 5       |                          |
#         +--------------+                          |
#         | Byte 6       |                          |
#         +--------------+                          |
#         | Byte 7       |                          |
#         +--------------+--------------------------+
#
#         Parameters
#         ----------
#         UAT_ID              : int
#             the UAT index
#         deviceId            : int
#             the target device ID
#         parameterType       : ParameterType
#             specifies the format of the data
#         value               : int
#             parameter value
#
#         Returns
#         -------
#         parameterMessage    : bytearray
#             the encoded parameter message
#
#         Raises
#         -------
#         ValueError:
#             If the parameter type and the type of the parameter value mismatch.
#         TypeError:
#             If the parameter type does not specify int or float type.
#
#         """
#         parameterMessage      = bytearray(8)
#         ## encode the message
#         parameterMessage[0:2] = struct.pack('<H', UAT_ID)
#         parameterMessage[2]   = 0x1 # Message index
#         parameterMessage[3]   = deviceId
#
#         # encode the parameter value
#         # int value
#         if parameterType == ParameterType.INTEGER_READ or parameterType == ParameterType.IEEE_FLOAT_READ:
#             parameterMessage[4:] = struct.pack('<I', 0)
#         elif parameterType == ParameterType.INTEGER_WRITE or parameterType == ParameterType.INTEGER_RW:
#             parameterMessage[4:] = struct.pack('<I', value)
#         # float value
#         elif parameterType == ParameterType.IEEE_FLOAT_WRITE or parameterType == ParameterType.IEEE_FLOAT_RW:
#             parameterMessage[4:] = struct.pack('<f', value)
#         else:
#             raise ValueError("UATv1-3 encodeParameterMessage: unknown Parameter Type")
#         return parameterMessage
#
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: decodeParameterMessage                                                                                 #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def decodeParameterMessage(parameterType, parameterMessage):
#         """
#         The function decodes the parameter message. Returns a list with the device-id and the parameter value. The
#         parameter message is defined:
#
#         +--------------+--------------------------+
#         | Byte 0       |                          |
#         +--------------+ UAT-ID                   |
#         | Byte 1       |                          |
#         +--------------+--------------------------+
#         | Byte 2       | Message index            |
#         +--------------+--------------------------+
#         | Byte 3       | Device ID                |
#         +--------------+--------------------------+
#         | Byte 4       | Parameter value          |
#         +--------------+                          |
#         | Byte 5       |                          |
#         +--------------+                          |
#         | Byte 6       |                          |
#         +--------------+                          |
#         | Byte 7       |                          |
#         +--------------+--------------------------+
#
#         Parameters
#         ----------
#         parameterType       : int
#             specifies the format of the data
#         parameterMessage    : bytearray
#             the received parameter message
#
#         Returns
#         -------
#         A list with the device id and parameter value
#
#         Raises
#         -------
#         ValueError:
#             if the data format does not specify int or float type
#
#         """
#         # decode the parameter message
#         device_Id = parameterMessage[3]
#         # decode the value
#         if parameterType is ParameterType.INTEGER_RW or parameterType is ParameterType.INTEGER_READ \
#                 or parameterType is ParameterType.INTEGER_WRITE:
#             # parameter type is an integer type
#             value = struct.unpack('<I', parameterMessage[4:8])[0]
#         elif parameterType is ParameterType.IEEE_FLOAT_RW or parameterType is ParameterType.IEEE_FLOAT_READ \
#                 or parameterType is ParameterType.IEEE_FLOAT_WRITE:
#             # parameter type is a float type
#             value = struct.unpack('<f', parameterMessage[4:8])[0]
#         else:
#             raise ValueError("UATv1 decode: unknown Parameter Type")
#
#         return [device_Id, value]
#
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: encodeDimMessage                                                                                       #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def encodeDimMessage(UAT_ID, messageIndex, dim0, dim1, dim2, dim3):
#         """
#         The function encodes the arguments to a byte array with 8 bytes. The dim message is defined:
#
#         +--------------+--------------------------+
#         | Byte 0       |                          |
#         +--------------+ UAT-ID                   |
#         | Byte 1       |                          |
#         +--------------+--------------------------+
#         | Byte 2       | Message index            |
#         +--------------+--------------------------+
#         | Byte 3       | Dim 0                    |
#         +--------------+--------------------------+
#         | Byte 4       | - Bit 0-1: Dim 0         |
#         |              | - Bit 2-7: Dim 1         |
#         +--------------+--------------------------+
#         | Byte 5       | - Bit 0-3: Dim 1         |
#         |              | - Bit 4-7: Dim 2         |
#         +--------------+--------------------------+
#         | Byte 6       | - Bit 0-5: Dim 2         |
#         |              | - Bit 6-7: Dim 3         |
#         +--------------+--------------------------+
#         | Byte 7       | Dim 3                    |
#         +--------------+--------------------------+
#
#         Parameters
#         ----------
#         UAT_ID              : int
#             the UAT index
#         messageIndex        : byte
#             sequence number of command message. Start at 0, increment by 1 for each additional command message.
#         dim0                : byte
#             for arrays in parameter structures, specifies which indexes should be used in which dimension.
#         dim1                : byte
#             for arrays in parameter structures, specifies which indexes should be used in which dimension.
#         dim2                : byte
#             for arrays in parameter structures, specifies which indexes should be used in which dimension.
#         dim3                : byte
#             for arrays in parameter structures, specifies which indexes should be used in which dimension.
#
#         Returns
#         -------
#         dimMessage          : bytearray
#             the encoded dim message
#
#         """
#         dimMessage = bytearray(8)
#         # encode dim message
#         dimMessage[0:2] = struct.pack('<H', UAT_ID)
#         # message index
#         dimMessage[2] = messageIndex
#         # dim0 | dim1 | dim2 | dim3
#         dimMessage[3] = (dim0 & 0xff)
#         dimMessage[4] = ((dim1 & 0x3f) << 2) | ((dim0 & 0x300) >> 8)
#         dimMessage[5] = ((dim2 & 0x0f) << 4) | ((dim1 & 0x3c0) >> 6)
#         dimMessage[6] = ((dim3 & 0x3) << 6) | ((dim2 & 0x3f0) >> 4)
#         dimMessage[7] = (dim3 & 0x3fc) >> 2
#         return dimMessage
#
#     # ---------------------------------------------------------------------------------------------------------------- #
#     # function: decodeDimMessage                                                                                       #
#     # ---------------------------------------------------------------------------------------------------------------- #
#     @staticmethod
#     def decodeDimMessage(dimMessage):
#         """
#         The function decodes the dim message. Returns a list with the dim0, dim1, dim2 and dim3. The dim message is
#         defined:
#
#         +--------------+--------------------------+
#         | Byte 0       |                          |
#         +--------------+ UAT-ID                   |
#         | Byte 1       |                          |
#         +--------------+--------------------------+
#         | Byte 2       | Message index            |
#         +--------------+--------------------------+
#         | Byte 3       | Dim 0                    |
#         +--------------+--------------------------+
#         | Byte 4       | - Bit 0-1: Dim 0         |
#         |              | - Bit 2-7: Dim 1         |
#         +--------------+--------------------------+
#         | Byte 5       | - Bit 0-3: Dim 1         |
#         |              | - Bit 4-7: Dim 2         |
#         +--------------+--------------------------+
#         | Byte 6       | - Bit 0-5: Dim 2         |
#         |              | - Bit 6-7: Dim 3         |
#         +--------------+--------------------------+
#         | Byte 7       | Dim 3                    |
#         +--------------+--------------------------+
#
#
#         Parameters
#         ----------
#         dimMessage  : bytearray
#             the encoded dim message
#
#         Returns
#         -------
#         A list with the dim0, dim1, dim2 and dim3
#
#         """
#         dim0 = dimMessage[3] | ((dimMessage[4] & 0x3) << 8)
#         dim1 = ((dimMessage[5] & 0xf) << 6) | ((dimMessage[4] & 0xfc) >> 2)
#         dim2 = ((dimMessage[6] & 0x3f) << 4) | ((dimMessage[5] & 0xf0) >> 4)
#         dim3 = (dimMessage[7] << 2) | ((dimMessage[6] & 0xc0) >> 6)
#
#         return [dim0, dim1, dim2, dim3]