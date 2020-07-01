import glob
import json
import logging
import os

from enum import Enum

# from smartmicro.Protocols.uat.uatV1 import UATv1
# from smartmicro.Protocols.uat.uatV2 import UATv2
# from smartmicro.Protocols.uat.uatV3 import UATv3
from smartmicro.Protocols.uat.uatV4 import UATv4

# from smartmicro.Services.communication import comDeviceTypes

class errorFunctionID(Enum):
    PRE_ENCODE_ERROR  = 0
    POST_ENCODE_ERROR = 1

class cspCommon:

    # ---------------------------------------------------------------------------------------------------------------- #
    # function: initializeValueDict                                                                                    #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def initializeValueDict(values, fileList, rootDir, fileExt):
        """
        The function reads the available status/command/parameter and creates a dictionary with the input.

        Parameters
        ----------
        values          : dictionary
            dictionary with keys and properties
        fileList       : list of strings / none
            list of used files for key analysis
        rootDir        : string / none
            relative path of file structure
        fileExt        : string
            file extension to assign kind of file

        """
        if fileList is not None:
            with open(fileList) as pFile_data:
                data  = json.load(pFile_data)
                files = data['relative']

                # absolute path from the start python file, e.g. runAllTests.py
                absolute_start_path = os.path.abspath(os.path.dirname(fileList))
                abs_path_to_files   = absolute_start_path + "/../"

                for i in range(0, len(files)):
                    # create a absolute path from the absolute start path and the relative path
                    files[i] = os.path.join(abs_path_to_files, files[i])
            # read all available status/command/parameter
            for file in files:
                logging.info("loading {} file: {}".format(fileExt, file))
                with open(file) as json_data:
                    d                 = json.load(json_data)
                    values[d['name']] = d

        if rootDir is not None:
            if os.name == 'nt':
                path_sep_ext = '\\**\\*.{}'.format(fileExt)
                # check if path is already absolute
                if not os.path.isabs(rootDir):
                    rootDir = os.getcwd() + "\\" + rootDir
            else:
                path_sep_ext = '/**/*.{}'.format(fileExt)
            fileList = glob.iglob(rootDir + path_sep_ext, recursive=True)
            # read all available status/command/parameter
            for file in fileList:
                logging.info("loading {} file: {}".format(fileExt, file))
                with open(file) as json_data:
                    d                 = json.load(json_data)
                    values[d['name']] = d


    # ---------------------------------------------------------------------------------------------------------------- #
    # function: sendRawParameter                                                                                       #
    # ---------------------------------------------------------------------------------------------------------------- #
    @staticmethod
    def sendRawParameter(param, comModule, txConfig=None, errorHandler=None):
        """
        This function expects a dictionary that describes the parameter as it is defined in the context of UAT
        format.
        The function encode the given dictionary into UAT format (v1 or v2) if the dictionary contains at least the
        dim0 as key it encodes the data as UAT v2. Otherwise it is encoded as v1

        Parameters
        ----------
        param           : dictionary
            dictionary with all information required to be UAT format compliant
        comModule       : instance of communication module
            communication module with can message interface to transfer the message
        txConfig        : dict
            contains all necessary information about the used transmission data (canId, ...)
        errorHandler    : dict
            generates errors (default: 0/ no error)

        Returns
        -------

        """
        # check if error handler is none type element
        if errorHandler is None:
            # set error to be a dictionary
            errorHandler = dict()
        # get current used uat version
        uatVersion = param["uatVersion"]
        # check if function is available
        if errorFunctionID.PRE_ENCODE_ERROR in errorHandler.keys():
            if errorHandler[errorFunctionID.PRE_ENCODE_ERROR] is not None:
                errorHandler[errorFunctionID.PRE_ENCODE_ERROR](param)

        if uatVersion is 4:
            canData = UATv4.encode(param)
        else:
            raise ValueError("Wrong UAT Version!")
        # elif uatVersion is 3:
        #     canData = UATv3.encode(param)
        # elif uatVersion is 2:
        #     canData = UATv2.encode(param)
        # else:
        #     canData = UATv1.encode(param)

        # check if function is available
        if errorFunctionID.POST_ENCODE_ERROR in errorHandler.keys():
            if errorHandler[errorFunctionID.POST_ENCODE_ERROR] is not None:
                errorHandler[errorFunctionID.POST_ENCODE_ERROR](canData)

        msgList = []
        for c in canData:
            msgList.append({'canId': txConfig["canId"], 'data': c})

        comModule.sendCanMessages(msgList, deviceType=txConfig["deviceType"], destId=txConfig["clientId"],
                                  channelIndex=txConfig["channelIndex"])
