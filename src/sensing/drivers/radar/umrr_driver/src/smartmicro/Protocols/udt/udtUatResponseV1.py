from smartmicro.Protocols.uat.uatMain import ParameterType
from smartmicro.Protocols.udt.udtUatResponseMain import UAT_ResponseMain, UAT_RespErrorCode


class UATv1Response(UAT_ResponseMain):
    """
    The class encodes and decodes messages of UDT type 17000 version 2 which is an UAT Response Message for UAT message
    version 1
    """

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __init__                                                                                               #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self):
        super().__init__()
        self.data['version'] = 2
        self.data['parameterType'] = ParameterType.INTEGER_READ
        self.data['result'] = UAT_RespErrorCode.SUCCESS

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def encode(data):
        """
        Encodes a dictionary of UDT type 17000 version 2.

        Parameters
        ----------
        data        : dict
            A dict with the keys: 'UDT-Index', 'version', 'UAT-ID', 'result', 'deviceId' 'parameterNumber',
            'parameterType', 'counter' and 'value'.

        Returns
        -------
        five byte objects representing the UAT Response Message for UAT message version 1

        Raises
        ------
        TypeError:
            if the type from the udt-index, uat-index or parameterNumber is not int.
        ValueError:
            if the UDT index does not match the expected one (17000)
            if the version is not equal 2.

        """
        # Check that the parameterNumber, the UAT-ID are an integer
        if type(data['parameterNumber']) is not int:
            raise TypeError("UDT_UAT_v1_Response encode: ParameterNumber must be of type int")
        if type(data['UAT-ID']) is not int:
            raise TypeError("UDT_UAT_v1_Response encode: UAT-ID must be of type int")
        if type(data['UDT-Index']) is not int:
            raise TypeError("UDT_UAT_v1_Response encode: UDT-Index must be of type int")
        if data['UDT-Index'] != 17000:
            raise ValueError("UDT_UAT_v1_Response encode: UDT-Index value is not 17000")
        # Check the version number
        if data['version'] != 2:
            raise ValueError("UDT_UAT_v1_Response encode: The version number is false. Version 2 expected. Found: {}"
                             .format(data['version']))

        # Encode the header
        header = UAT_ResponseMain.encodeHeaderMessage(data['UDT-Index'], data['version'])

        # Encode the first message (parameter message)
        message1 = UAT_ResponseMain.encodeParameterTypeMessage(data['UDT-Index'] + 1, data['counter'], data['result'].value,
                                                               data['deviceId'], data['parameterType'].value,
                                                               data['parameterNumber'])

        # Encode the third message (value message)
        message3 = UAT_ResponseMain.encodeValueMessage(data['UDT-Index'] + 3, data['counter'], data['parameterType'],
                                                       data['value'])

        # Encode the second message and calculate the crc value
        crcCalcArrayBefore = message1[2:8]
        crcCalcArrayAfter = message3[2:8]
        message2 = UAT_ResponseMain.encodeCRCMessage(data['UDT-Index'] + 2, data['counter'], data['UAT-ID'],
                                                     crcCalcArrayBefore, crcCalcArrayAfter)

        return [header, message1, message2, message3, header]

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: decode                                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def decode(msgList):
        """
        Decodes a binary message of UDT type 17000 version 2 which is an UAT Response Message for UAT message version 1

        Parameters
        ----------
        msgList         : bytearray
            Contains five byte objects each being of length 8.

        Returns
        -------
        data            : dict
            A dict with the keys: 'UDT-Index', 'version', 'UAT-ID', 'result', 'deviceId' 'parameterNumber',
            'parameterType', 'counter' and 'value'.

        Raises
        ------
        TypeError:
            if the length from each message is not equal 8.
            if the number of messages is not equal 5.
        ValueError:
            if the UDT index does not match the expected one (17000)
            if the version is not equal 2.
            if the counters from the instruction message does not match.

        """
        # Check that the msgList is correct
        numberOfMessages = 5
        if len(msgList) != numberOfMessages:
            raise TypeError('UDT_UAT_v1_Response decode: The message list has not five messages. msgList:{}'
                            .format(msgList))
        # Check that the messages in the msgList is correct
        messagesLength = 8
        for msg in msgList:
            if len(msg) != messagesLength:
                raise TypeError('UDT_UAT_v1_Response decode: The input message list is corrupt. len(msg):{}. msgList:{}.'
                                .format(len(msg), msgList))

        # Create the data dict
        data = dict()
        header = msgList[0]
        message1 = msgList[1]
        message2 = msgList[2]
        message3= msgList[3]
        footer= msgList[4]

        # Decode the header
        [data['UDT-Index'], data['version']] = UAT_ResponseMain.decodeHeaderMessage(header)

        # Check the version
        if data['version'] != 2:
            raise ValueError("UDT_UAT_v1_Response decode: Tried to decode a UAT response with wrong UDT type 17000 version. "
                            "Version 2 expected. Found {}".format(data['version']))
        # Check the UDI-Index from header
        if data['UDT-Index'] != 17000:
            raise TypeError("UDT_UAT_v1_Response decode: header Decoded UDT index is not 17000 instead {} was decoded"
                            .format(data['UDT-Index']))

        [index, version] = UAT_ResponseMain.decodeHeaderMessage(footer)
        # Check that the header is equal to the footer
        if index != data['UDT-Index'] or version != data['version']:
            raise ValueError("UDT_UAT_v1_Response decode: The footer is not equal the header. Header:{} Footer{}"
                             .format(header, footer))

        # Decode the message 1 (parameter type message)
        [index, counter1, data['result'], data['deviceId'], data['parameterType'], data['parameterNumber']] = UAT_ResponseMain.decodeParameterTypeMessage(message1)

        # Check the UDI-Index from message 1
        if index != 17001:
            raise TypeError("UDT_UAT_v1_Response decode: Decoded message 1 UDT index is not 17001 instead {} was decoded"
                            .format(index))

        # decode message 2 (crc message)
        crcCalcDataArrayBefore = message1[2:8]
        crcCalcDataArrayAfter = message3 [2:8]
        [index, counter2, data['UAT-ID']] = UAT_ResponseMain.decodeCRCMessage(message2, crcCalcDataArrayBefore,
                                                                            crcCalcDataArrayAfter)
        # Check the UDI-Index from message 2
        if index != 17002:
            raise TypeError("UDT_UAT_v1_Response decode: Decoded message 2 UDT index is not 17002 instead {} was decoded"
                            .format(index))

        # Decode message 3 (parameter value message)
        [index, counter3, data['value']] = UAT_ResponseMain.decodeValueMessage(message3, data['parameterType'])

        # Check the UDI-Index from message 3
        if index != 17003:
            raise TypeError("UDT_UAT_v1_Response decode: Decoded message 3 UDT index is not 17003 instead {} was decoded"
                            .format(index))

        # Check that all three counter are the same
        if counter1 != counter2 or counter1 != counter3:
            raise ValueError('UDT_UAT_v1_Response decode: The three instruction message counters does not match. '
                             'Counter1:{} Counter2:{} Counter3:{}'
                             .format(counter1, counter2, counter3))
        data['counter'] = counter1
        return data