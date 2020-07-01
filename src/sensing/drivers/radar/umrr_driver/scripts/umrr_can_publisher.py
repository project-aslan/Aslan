#!/home/bruns/.virtualenvs/ros_env/bin/python


import rospy
import sensor_msgs.point_cloud2 as pc2
import queue

from std_msgs.msg import Header

from smartmicro.HardwareLayer.comFactory.comDeviceFactory import ComDeviceFactory
from smartmicro.Services.basicCanServices.multiMessageService import multiMessageService
from smartmicro.Services.communication import comDeviceTypes, Communication
from smartmicro.Services.protocolServices.status import Status
from smartmicro.Services.protocolServices.parameter import Parameter
from smartmicro.Services.basicCanServices.uatResponseService import uatResponseService
from umrr_driver.srv import *


class umrr_publisher:
    """
    Class creates the ros node, provides steps to generate pc2 message
    from sensor target list and creates two services to communicate
    with the connected sensor through the status and parameter protocol-
    services
    """
    def __init__(self):
        # ask server whether legacy parameter is set
        if rospy.has_param("~legacy_mode"):
            # debug parameter is set, copy to local variable
            self.legacy_mode = rospy.get_param("~legacy_mode")
        else:
            rospy.logerr("No operating mode given!")
        # initialize publisher
        if rospy.has_param("~can_socket") and rospy.has_param("~can_spec"):
            self.spec = rospy.get_param("~can_spec")
            self.sock = rospy.get_param("~can_socket")
            self.pub = rospy.Publisher('radar_data', pc2.PointCloud2, queue_size=3)
            self.frame_id = rospy.get_param("~frame_id")
            # establish sensor communication
            err = self.__est_sens_connection()
            if err < 0:
                rospy.logerr("CanSpec or Socket not found!")
                self.pub.unregister()
                rospy.signal_shutdown("CanSpec not found.")
            else:
                self.setup_node()

        else:
            # error flag is set, unregister publisher
            self.pub.unregister()
            rospy.signal_shutdown("Can Socket and/or Can Spec not found.")

    def setup_node(self):
        """
        defines the running sequence within this node
        """
        # provide services for communication from outside
        self.__create_serv()
        # if in debug mode read in json file nevertheless
        self.__inform_log()
        if not self.legacy_mode:
            try:
                self.__read_sw_version()
                self.__setup_sensor()
                self.__get_used_atenna()
            except (TypeError, AttributeError):
                rospy.logerr("No communication with sensor possible! Either try legacy mode or inspect connection.")
        self.__run()

    def __run(self):
        """
        Function retreives messages from sensor and publishes pc2 clouds
        """
        # start spinning
        self.seq = 0
        rospy.loginfo("Initialization done. Entering running loop.")
        # receive_err = False
        while not rospy.is_shutdown():
            try:
                target_list = self.target_serv.getMessageInformation(timeout=0.1)
                # create pointcloud and...
                cloud_msg = self.__create_pcl_msg(target_list, self.seq, self.fields, self.field_names)
                # ... publish it
                self.pub.publish(cloud_msg)
                self.seq += 1
            except queue.Empty:
                pass

    def __create_serv(self):
        """
        Function creates necessary interfaces to request status and
        send parameters to the connected sensor from outside of the node
        """
        # services rely on communication with sensor, only run them when this is possible (not in legacy mode)
        if not self.legacy_mode:
            rospy.Service('sensor_status', sens_status, self.__status_req)
            rospy.Service('sensor_parameter', sens_param, self.__param_set)
            rospy.Service('sensor_parameter_request', sens_param_req, self.__param_req)

    def __inform_log(self):
        """
        Function prints some information about the connected Sensor like
        used software version, frame id and can spec
        """
        # create list for can messages
        self.__fields_from_spec()
        rospy.loginfo("Using frame_id: %s", self.frame_id)
        rospy.loginfo('The fields of the targets are: %s' % ''.join([name + ', ' for name in self.field_names]))

    def __fields_from_spec(self):
        """
        Function creates the message template from can spec
        """
        self.fields = []
        self.field_names = []
        self.byte_offset = 0
        # get fields from header message
        # first check whether modeBit is set
        if self.target_serv.messageSpec["header"]["modeBit"] is not None:
            # modeBit in header is set, retrieve length
            for i in range(0, self.target_serv.messageSpec["header"]["modeBit"]["end_bit"] + 1):
                elements = self.target_serv.messageSpec["header"][str(i)]
                for element in elements:
                    self.fields.append(pc2.PointField(element, self.byte_offset, pc2.PointField.FLOAT32, 1))
                    self.field_names.append(element)
                    self.byte_offset += 4
        else:
            # no modeBit is set, set field to default ("0")
            elements = self.target_serv.messageSpec["header"]["0"]
            for element in elements:
                self.fields.append(pc2.PointField(element, self.byte_offset, pc2.PointField.FLOAT32, 1))
                self.field_names.append(element)
                self.byte_offset += 4
        # retrieve length of body message from spec
        for i in range(0, self.target_serv.messageSpec["body"]["maximum"]+1):
            elements = self.target_serv.messageSpec["body"][str(i)]
            for element in elements:
                self.fields.append(pc2.PointField(element, self.byte_offset, pc2.PointField.FLOAT32, 1))
                self.field_names.append(element)
                self.byte_offset += 4

    def __setup_sensor(self):
        """
        If Parameters are set in config, function sets antenna mode and center frequency
        """
        # check if setup parameters are omitted:
        if rospy.has_param("~antenna_mode"):
            # set antenna mode
            ant_dict = rospy.get_param("~antenna_mode")
            # send config to sensor
            self.parameter.sendRawParameter(ant_dict["section"], ant_dict["parameter"],
                                                       ant_dict["value"], msgType="w")
        if rospy.has_param("~center_frequency"):
            freq_dict = rospy.get_param("~center_frequency")
            self.parameter.sendRawParameter(freq_dict["section"], freq_dict["parameter"],
                                                       freq_dict["value"], msgType="w")

    def __get_used_atenna(self):
        """
        Gets the used antenna mode upon startup
        :return: prints antenna mode to console
        """
        self.parameter.sendRawParameter(2010, 0, msgType="r")
        res = self.__get_resp_value()
        res = res["instructionsList"][0]['value']
        rospy.loginfo(''.join("Current Antenna Mode: " + str(res)))

    def __read_sw_version(self):
        """
        Function reads software and type information from sensor
        :return: prints software version and type to console
        """
        # create dictionary
        self.sw_dict = []
        for x in range(0, 14):
            self.status.sendRawStatus(2012, x)
            res = self.__get_resp_value()
            res = res["instructionsList"][0]['value']
            self.sw_dict.append(res)
        # check if sensor responded to query
        if len(self.sw_dict) == 0:
            raise ValueError()
        else:
            sens_sw = ''.join([str(i) + '.' for i in self.sw_dict[3:6]])
            sens_sw = sens_sw[0:len(sens_sw)-1]
            rospy.loginfo('Sensor Software Version: %s', sens_sw)
            sens_serial = hex(self.sw_dict[9])
            rospy.loginfo('Sensor Serial: %s', sens_serial)
            # send info to parameter Server
            if rospy.has_param('~software_vers'):
                rospy.set_param("~software_vers", sens_sw)
            if self.sw_dict[10] == 17:
                # connected to UMRR-11
                rospy.loginfo("Sensor Type: UMRR 11")
                # send info to parameter Server
                if rospy.has_param('~sensor_type'):
                    rospy.set_param("~sensor_type", "UMRR_11")
                self.sensor_type = 1

            else:
                # connected to UMRR-96
                rospy.loginfo("Sensor Type: UMRR 96")
                # send info to parameter Server
                if rospy.has_param('~sensor_type'):
                    rospy.set_param("~sensor_type", "UMRR_96")
                self.sensor_type = 0

    def __get_resp_value(self):
        """
        Function reads the response from sensor
        :return: dictionary with sensor response
        """
        try:
            resp = self.respserv.getMessage(timeout=2)
        except queue.Empty:
            rospy.logerr("Could not receive sensor response!")
            resp = 0
        return resp

    def __status_req(self, req):
        """
        Function handles status service
        :param req: expecting 2 int (section parameter), third int (data type) is optional
        :return: sensor response as string
        """
        # check if user omitted an empty string
        if req.type == "":
            self.status.sendRawStatus(req.section, req.param)
        else:
            # string is not empty, set to value
            self.status.sendRawStatus(req.section, req.param, statusType=req.type)
        # get response from queue
        resp = self.__get_resp_value()
        # retrieve error code from dict
        resp = resp["instructionsList"][0]['value']
        return ''.join("Sensor answered: " + str(resp))

    def __param_req(self, req):
        """
        Function handles parameter request service
        :param req: expecting 2 int (section parameter)
        :return: sensor response as string
        """
        # send parameter request to sensor (read mode)
        self.parameter.sendRawParameter(req.section, req.param, msgType="r")
        # get answer from queue
        res = self.__get_resp_value()
        resp = res["instructionsList"][0]['value']
        return ''.join("Sensor answered: " + str(resp))

    def __param_set(self, req):
        """
        Function handles parameter service
        :param req: expecting 3 int (section parameter value)
        :return: sensor response as string
        """
        if req.dim > 0:
            # build list of dimensions from received value (only supports one dimension!)
            dim = [req.dim]
        else:
            dim = [0, 0]
        self.parameter.sendRawParameter(req.section, req.param, value=req.value, dimension=dim)
        resp = self.__get_resp_value()
        resp = resp["instructionsList"][0]['result']
        return ''.join("Sensor answered: " + str(resp))

    def __create_pcl_msg(self, list_of_targets, seq, fields, field_names):
        """
        Function fills the predefined pc2 template with data coming from the sensor
        if debug mode is set, only the fields necessary for conversion to cartesian coordinates are checked
        :param list_of_targets: dictionary from target list server
        :param seq: consecutive integer for each iteration
        :param fields: predefined fields from canspec.json
        :param field_names: predefined names from canspec.json
        :return: ROS pc2 message containing information of measured targets
        """
        pointcloud_targets = []

        for target in list_of_targets:
            pointcloud_target = [None] * len(fields)
            for field in fields:
                # check if key is in list, else dismiss target
                try:
                    pointcloud_target[field_names.index(field.name)] = target[field.name]
                except KeyError:
                    # only exit loop if no cartesian coordinate can be build
                    if field.name != "Azimuth" and field.name != "Elevation" and field.name != "Range":
                        pointcloud_target[field_names.index(field.name)] = 0
                    else:
                        break
            else:
                # loop ended with no error, append target to list
                pointcloud_targets.append(pointcloud_target)

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.frame_id
        header.seq = seq

        cloud_msg = pc2.create_cloud(header, fields, pointcloud_targets)
        return cloud_msg

    def __est_sens_connection(self):
        """
        Function creates a communication instance,
        registers targetlist and response service and
        status and parameter module
        """

        try:
            device_inst = ComDeviceFactory.getDevice('CAN_SOCKET', self.sock)
            self.comMod = Communication([{'deviceType': comDeviceTypes.CAN, 'device': device_inst}])
            self.target_serv = multiMessageService(self.spec)
        except OSError:
            return -1

        self.respserv = uatResponseService()
        # get startId and endId from spec
        start_id = self.target_serv.messageSpec["header"]["canId"]
        id_range = self.target_serv.messageSpec["body"]["rangeId"]
        # register target serv with com module instance
        self.comMod.registerCanIDService(self.target_serv, start_id, id_range)
        self.comMod.registerCanIDService(self.respserv, 0x700)
        # register status and parameter module
        self.status = Status(self.comMod)
        self.parameter = Parameter(self.comMod)
        return 0


if __name__ == '__main__':
    # create node
    rospy.init_node('umrr_can_publisher.py')
    try:
        umrr_driver = umrr_publisher()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
