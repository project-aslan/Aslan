import json
import queue
import logging

from copy import deepcopy

from smartmicro.Services.basicCanServices.canService import CanIDService


class multiMessageService(CanIDService):
    """
    The service handles the decoding of header information within the given rules. The rule set itself needs the
    following structure:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | header                 | dict      | contains the header information     |
    +------------------------+-----------+-------------------------------------+
    | body                   | dict (opt)| contains the body information       |
    +------------------------+-----------+-------------------------------------+

    The "header" itself contains the following structure:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | canId                  | integer   | contains the can identifier of the  |
    |                        |           | header message                      |
    +------------------------+-----------+-------------------------------------+
    | modeBit                | dict      | contains the designated information |
    |                        |           | rule set of the given data, if no   |
    |                        |           | mode bit is used it contains: None  |
    +------------------------+-----------+-------------------------------------+
    | maximum                | integer   | contains the maximum number of      |
    |                        |           | header messages                     |
    +------------------------+-----------+-------------------------------------+
    | <mode>                 | dict      | contains the decoding rule set for  |
    |                        |           | the given data, the name itself     |
    |                        |           | depends on the message id e.g. "1", |
    |                        |           | but default is "0"                  |
    +------------------------+-----------+-------------------------------------+

    The "modeBit" dictionary contains this structure if not set to be None:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | start_byte             | integer   | contains start byte information     |
    +------------------------+-----------+-------------------------------------+
    | start_bit              | integer   | contains start bit information      |
    |                        |           | within the given start byte         |
    +------------------------+-----------+-------------------------------------+
    | end_byte               | integer   | contains end byte information       |
    +------------------------+-----------+-------------------------------------+
    | end_bit                | integer   | contains end bit information within |
    |                        |           | the given end byte                  |
    +------------------------+-----------+-------------------------------------+

    The "<mode>" itself needs to match the following information:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | <propertyName A>       | dict      | contains the decoding rule          |
    +------------------------+-----------+-------------------------------------+
    | <propertyName B>       | dict      | contains the decoding rule          |
    +------------------------+-----------+-------------------------------------+
    | ...                    | dict      | contains the decoding rule          |
    +------------------------+-----------+-------------------------------------+

    the property decoding rule is defined like this:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | offset                 | integer   | contains the numeric offset         |
    +------------------------+-----------+-------------------------------------+
    | resolution             | float/int | contains the resolution depending on|
    |                        |           | the given information               |
    +------------------------+-----------+-------------------------------------+
    | start_byte             | integer   | contains start byte information     |
    +------------------------+-----------+-------------------------------------+
    | start_bit              | integer   | contains start bit information      |
    |                        |           | within the given start byte         |
    +------------------------+-----------+-------------------------------------+
    | end_byte               | integer   | contains end byte information       |
    +------------------------+-----------+-------------------------------------+
    | end_bit                | integer   | contains end bit information within |
    |                        |           | the given end byte                  |
    +------------------------+-----------+-------------------------------------+

    Next to the mandatory "header" the body is optional and can be deleted if not necessary. The body itself contains
    the following structure:

    +------------------------+-----------+-------------------------------------+
    | key                    | property  | comment                             |
    +========================+===========+=====================================+
    | canId                  | integer   | contains the start can identifier   |
    |                        |           | of the body messages                |
    +------------------------+-----------+-------------------------------------+
    | rangeId                | integer   | contains the can identifier range of|
    |                        |           | the body message                    |
    +------------------------+-----------+-------------------------------------+
    | modeBit                | dict      | contains the designated information |
    |                        |           | rule set of the given data, if no   |
    |                        |           | mode bit is used it contains: None  |
    +------------------------+-----------+-------------------------------------+
    | maximum                | integer   | contains the maximum number of      |
    |                        |           | header messages                     |
    +------------------------+-----------+-------------------------------------+
    | <mode>                 | dict      | contains the decoding rule set for  |
    |                        |           | the given data, the name itself     |
    |                        |           | depends on the message id e.g. "1", |
    |                        |           | but default is "0"                  |
    +------------------------+-----------+-------------------------------------+

    The sub-dictionary information set is like the above defined structures.

    """
    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initialization                                                                                         #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __init__(self, canSpecFile=None):
        """
        The current function initializes target can ID service. Therefore the decoding style of the message will be
        required and an nput queue will be configured.

        Parameters
        ----------
        canSpecFile : json-File
            The json file has to be configured to describe the setup of can message designed for targets
        """
        # init parent class
        CanIDService.__init__(self)
        # provide queue
        self.recvQueue = queue.Queue()
        # check if CAN spec is available
        if canSpecFile is not None:
            # read can spec and save them
            with open(canSpecFile) as json_data:
                self.messageSpec = json.load(json_data)
            # extract header id
            self.headerId = self.messageSpec["header"]["canId"]
            # assign first message as None type
            self.nextMsg  = None
        else:
            # raise an error for missing can spec
            raise TypeError("ERROR : missing can spec!")

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: getMessageInformation                                                                                  #
    # ---------------------------------------------------------------------------------------------------------------- #
    def getMessageInformation(self, timeout=None):
        """
        The function returns a complete decoded can message.

        Returns
        -------
        decodedMessage  : dict
            dictionary with decoded data
        """
        # check if next message is available
        if self.nextMsg is None:
            # get message
            self.nextMsg = self.recvQueue.get(timeout=timeout)
        # get next header message
        while self.nextMsg["canID"] != self.headerId:
            # get message
            self.nextMsg = self.recvQueue.get(timeout=timeout)
        # handle header messages
        header = self.__handleHeaderMessages()
        # check if body is available
        if "body" in self.messageSpec:
            # handle body message
            retData = self.__handleBodyMessages(header)
        else:
            # set return data to current header information
            retData = header

        return retData

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __handleHeaderMessages                                                                                 #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __handleHeaderMessages(self):
        """
        The function handles the decoding of the header part and returns the resulting dictionary.

        Returns
        -------
        headerDict  : dictionary
            contains the decoded header information
        """
        # extract header rule set
        headerRuleSet = self.messageSpec["header"]
        # set number of decoded header messages to 0
        nofHeaderMsgs = 0
        # provide empty header messages
        headerDict    = dict()
        # run as long as possible for header messages
        while nofHeaderMsgs < headerRuleSet["maximum"]:

            # verify mode option
            if headerRuleSet["modeBit"] is not None:
                # extract requested data
                mode = self._extractBytes(self.nextMsg["data"], headerRuleSet["modeBit"])
                try:
                    # decode header
                    headerDict.update(self.__decodeMsgElements(self.nextMsg["data"], headerRuleSet[str(mode)]))
                except:
                    # log missing information for current mode
                    logging.info("MultiMsgService : current mode is not supported : {}".format(mode))
            else:
                # decode header
                headerDict.update(self.__decodeMsgElements(self.nextMsg["data"], headerRuleSet["0"]))
            # increment number of header messages
            nofHeaderMsgs += 1
            # check if further message is available
            if nofHeaderMsgs < headerRuleSet["maximum"]:
                # get next message
                self.nextMsg = self.recvQueue.get()
                # check if header msg is over
                if self.nextMsg["canID"] != headerRuleSet["canId"]:
                    # update number of header msgs to maximum
                    nofHeaderMsgs = headerRuleSet["maximum"]
            else:
                # set default message
                self.nextMsg = None

        return headerDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __handleBodyMessages                                                                                   #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __handleBodyMessages(self, header):
        """
        The function decodes the body part of multi message services. It appends the header information to each decoded
        part of the body and returns a list of available body information.
        Parameters
        ----------
        header  : dictionary
            contains the decoded header information

        Returns
        -------
        retList : list
            contains a list of body information
        """
        # extract header rule set
        bodyRuleSet   = self.messageSpec["body"]
        # set number of body messages
        nofBodyMsgs   = 0
        # generate return list
        retList       = list()
        # generate data dictionary
        dataDict      = dict()
        # update body data with header infos
        dataDict.update(header)
        # activate extended processing
        extProcessing = True
        # check if next message is available
        if self.nextMsg is None:
            # get message
            self.nextMsg = self.recvQueue.get()
        # save can identifier
        canIdentifier = self.nextMsg["canID"]
        # run through all data
        while extProcessing:

            # assign processing
            stopProcessing = False
            # check if next msg is in can id range
            if self.nextMsg["canID"] < ( bodyRuleSet["canId"] ) or \
               self.nextMsg["canID"] > ( bodyRuleSet["canId"] + bodyRuleSet["rangeId"] ):
                # stop processing
                stopProcessing = True
                # stop extended processing
                extProcessing = False
            # decode message
            if not stopProcessing:
                # verify mode option
                if bodyRuleSet["modeBit"] is not None:
                    # extract requested data
                    mode = self._extractBytes(self.nextMsg["data"], bodyRuleSet["modeBit"])
                    try:
                        # decode header
                        dataDict.update(self.__decodeMsgElements(self.nextMsg["data"], bodyRuleSet[str(mode)]))
                    except:
                        # log missing information for current mode
                        logging.info("MultiMsgService : current mode is not supported : {}".format(mode))
                else:
                    # decode header
                    dataDict.update(self.__decodeMsgElements(self.nextMsg["data"], bodyRuleSet["0"]))
                try:
                    # get next message
                    self.nextMsg = self.recvQueue.get(timeout=0.05)
                    # try to generate a new return dictionary
                    if canIdentifier != self.nextMsg["canID"]:
                        # increment number of body messages
                        nofBodyMsgs += 1
                        # append data to return list
                        retList.append(deepcopy(dataDict))
                        # generate data dictionary
                        dataDict = dict()
                        # update body data with header infos
                        dataDict.update(header)
                        canIdentifier = self.nextMsg["canID"]
                except:
                    # set message to default
                    self.nextMsg  = None
                    # set extended processing to false
                    extProcessing = False

        return retList

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: newMessageAvailable                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def newMessageAvailable(self, deleteMsg=True):
        """
        The function checks if a new message is available and deletes the old ones before if necessary.

        Parameters
        ----------
        deleteMsg   : bool
            assign messages to be deleted after new message check.
        Returns
        -------
        retValue    : bool
            true if message is received
        """
        # provide return message
        retValue = True
        # check receive queue
        if self.recvQueue.empty():
            # set retValue to false
            retValue = False
        # delete new message if necessary
        if deleteMsg == True and retValue == True:
            msg = self.recvQueue.get()

        return retValue

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: __decodeMsgElements                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    def __decodeMsgElements(self, msg, decodingDict):
        """
        The function will deal with the single decoding of every information listed in the json file for a given
        message.

        Parameters
        ----------
        msg             : binary
            The message will be a binary format information which has to be decoded

        decodingDict    : dict
            The decoding dict includes all rules to extract information from the current message

        Returns
        -------
        targetDict      : dict
            The target dictionary includes all decoded information
        """

        messageDict = dict()

        for decRule in decodingDict:
            messageDict[decRule] = self.decode(msg, decodingDict[decRule])

        return messageDict

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: encodeMessage                                                                                          #
    # ---------------------------------------------------------------------------------------------------------------- #
    def encodeMessage(self, information):
        """
        The function encodes the basic information with the given rule due to the coding rule.

        Parameters
        ----------
        information : dictionary
            the information is a dictionary that can hold all information given in the coding rule

        Returns
        -------
        msg         : dictionary
            the message is a dictionary with following keys "data" (data to send) and "canId"(used CAN Identifier)
        """
        keyList = self.messageSpec['Elements'].keys()

        data = int(0).to_bytes(8, byteorder='little')

        for key in keyList:
            if key in information.keys():
                data = self.encode(data, self.messageSpec['Elements'][key], information[key])
            else:
                data = self.encode(data, self.messageSpec['Elements'][key], 0)

        msg = dict()
        msg['canId'] = self.messageSpec["CanIdStart"]
        msg['data']  = data

        return msg



