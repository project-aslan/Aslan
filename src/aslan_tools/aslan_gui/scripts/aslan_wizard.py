#! /usr/bin/env python
#
# Copyright 2020 Project Aslan
# Author: Efimia Panagiotaki <efimia@streetdrone.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


import wx
import wx.lib.buttons
import wx.lib.agw.customtreectrl as CT
import gettext
import os
import datetime
import sys
import fcntl
import threading
import Queue
import time
import shlex
import signal
import subprocess
import psutil
import yaml
import aslan_gui
import rospy
import std_msgs.msg
from std_msgs.msg import Bool
from decimal import Decimal

from types import MethodType

from aslan_msgs.msg import ConfigNDT
from aslan_msgs.msg import ConfigVelocitySet
from aslan_msgs.msg import ConfigNDTMapping
from aslan_msgs.msg import ConfigNDTMappingOutput
from aslan_msgs.msg import ConfigVoxelGridFilter
from aslan_msgs.msg import ConfigRayGroundFilter
from aslan_msgs.msg import ConfigWaypointLoader
from aslan_msgs.msg import ConfigWaypointFollower
from aslan_msgs.msg import ConfigLaneRule
from aslan_msgs.msg import ConfigLaneSelect
from aslan_msgs.msg import ConfigTwistFilter
from aslan_msgs.msg import ConfigLaneStop

SCHED_OTHER = 0
SCHED_FIFO = 1
SCHED_RR = 2


class MyFrame(aslan_gui.MyFrame):

    def __init__(self, *args, **kwds):
        aslan_gui.MyFrame.__init__(self, *args, **kwds)
        self.load_dic = self.load_yaml('param.yaml', def_ret={})
        self.params = []
        self.log_que_stdout = Queue.Queue()
        self.log_que_stderr = Queue.Queue()
        self.log_que = Queue.Queue()
        self.log_que_show = Queue.Queue()
        self.Bind(wx.EVT_CLOSE, self.OnClose)
        self.all_th_infs = []
        self.all_tabs = []
        self.all_cmd_dics = []
        self.config_dic = {}
        self.sensing_cmd = {}
        self.all_procs = []

        # ros
        #
        rospy.init_node('aslan_wizard', anonymous=True)
        # rospy.Subscriber('aslan_wizard', std_msgs.msg.String, self.ROSCb)
        # self.pub = rospy.Publisher('', std_msgs.msg.String, queue_size=10)

        self.bitmap_logo.Destroy()
        self.bitmap_logo = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap(src_dir()+ 'Aslan_Autonomy.png', wx.BITMAP_TYPE_ANY))

        self.bitmap_1.Destroy()
        self.bitmap_1 = wx.StaticBitmap(self.tab_simulation, wx.ID_ANY,
                                        wx.Bitmap(src_dir() + 'imgs/sd.jpg', wx.BITMAP_TYPE_ANY))

        self.bitmap_2.Destroy()
        self.bitmap_2 = wx.StaticBitmap(self.tab_simulation, wx.ID_ANY,
                                        wx.Bitmap(src_dir() + 'imgs/park.jpg', wx.BITMAP_TYPE_ANY))

        # Configuration tab (Vehicle, Vehicle Info, Vehicle Model, Sensors tree, TF)

        tab = self.tab_configuration
        self.all_tabs.append(tab)

        self.drv_probe_cmd = {}
        self.sensing_cmd = {}
        self.all_cmd_dics.append(self.sensing_cmd)
        dic = self.load_yaml('vehicle.yaml')

        self.add_params(dic.get('params', []))
        self.create_checkboxes(dic, self.panel_sensing, None, self.drv_probe_cmd, self.sensing_cmd,
                               self.OnSensingDriver)

        dic1 = self.load_yaml('cameras.yaml')

        self.add_params(dic1.get('params', []))
        self.create_checkboxes(dic1, self.panel_sensing1, None, self.drv_probe_cmd, self.sensing_cmd,
                               self.OnSensingDriver)

        dic2 = self.load_yaml('radar.yaml')

        self.add_params(dic2.get('params', []))
        self.create_checkboxes(dic2, self.panel_sensing2, None, self.drv_probe_cmd, self.sensing_cmd,
                               self.OnSensingDriver)

        self.setup_buttons(dic.get('buttons', {}), self.sensing_cmd)

        self.dlg_rosbag_record = MyDialogROSbagRecord(self, cmd_dic=self.sensing_cmd)
        buttons_color_hdr_setup(self.dlg_rosbag_record)

        # Perception tab (Map, TF, Filters clicks, Localization NDT Tree)

        tab = self.tab_perception
        self.all_tabs.append(tab)

        parent = self.tree_ctrl_0.GetParent()
        for i in range(2):
            self.obj_get('tree_ctrl_' + str(i)).Destroy()

        self.map_cmd = {}
        self.all_cmd_dics.append(self.map_cmd)
        self.map_dic = self.load_yaml('perception.yaml')
        items = self.map_dic

        self.add_params(self.map_dic.get('params', []))

        self.setup_buttons(self.map_dic.get('buttons', {}), self.map_cmd)

        self.tc_point_cloud = self.obj_to_varpanel_tc(self.button_pcd, 'pcd_file')

        self.label_point_cloud_bar.Destroy()
        self.label_point_cloud_bar = BarLabel(tab, '  Loading...  ')
        self.label_point_cloud_bar.Enable(False)

        def hook1G(args):
            for f in args.get('func')().split(','):
                sz = os.path.getsize(f)
                if sz > 1024 * 1024 * 1024:
                    wx.MessageBox("Over 1GB\n\n{}\n({:,})".format(f, sz), caption='Warning')

        args = {'func': self.tc_point_cloud.GetValue}
        hook_var = {'hook': hook1G, 'args': args, 'flags': ['every_time']}
        obj = self.button_pcd
        gdic_v = self.obj_to_gdic(obj, {}).get('pcd_file', {})
        gdic_v['hook_var'] = hook_var

        for i in range(2):
            tree_ctrl = self.create_tree(parent, items['subs'][i], None, None, self.map_cmd)
            tree_ctrl.ExpandAll()
            tree_ctrl.SetBackgroundColour(wx.NullColour)
            setattr(self, 'tree_ctrl_' + str(i), tree_ctrl)

        self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)

        # Planning tab (Waypoint maker (loader), lane planner (traffic_wp, lane_select), astar planner (obstacle avoid, vel set), wp follower (pp, twist f)

        tab = self.tab_planning
        self.all_tabs.append(tab)
        self.planning_cmd = {}
        self.all_cmd_dics.append(self.planning_cmd)

        parent = self.tree_ctrl_5.GetParent()
        self.obj_get('tree_ctrl_5').Destroy()
        self.obj_get('tree_ctrl_6').Destroy()
        self.obj_get('tree_ctrl_7').Destroy()
        self.obj_get('tree_ctrl_8').Destroy()

        items = self.load_yaml('planning.yaml')

        self.add_params(items.get('params', []))

        self.setup_buttons(items.get('buttons', {}), self.planning_cmd)

        for i in range(4):
            tree_ctrl = self.create_tree(parent, items['subs'][i], None, None, self.planning_cmd)
            tree_ctrl.ExpandAll()
            tree_ctrl.SetBackgroundColour(wx.NullColour)
            setattr(self, 'tree_ctrl_' + str(i + 5), tree_ctrl)

        self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)

        # Controls tab ( vehicle interface , radio boxes, supervisor, disclaimer )

        tab = self.tab_controls
        self.all_tabs.append(tab)

        self.controls_cmd = {}
        self.all_cmd_dics.append(self.controls_cmd)

        controls = self.load_yaml('controls.yaml')

        self.add_params(controls.get('params', []))

        self.setup_buttons(controls.get('buttons', {}), self.controls_cmd)

        self.label_rosbag_play_bar.Destroy()
        self.label_rosbag_play_bar = BarLabel(self.tab_tools, '  Playing...  ')
        self.label_rosbag_play_bar.Enable(False)

        # Simulation tab

        tab = self.tab_simulation
        self.all_tabs.append(tab)

        self.sim_cmd = {}
        self.all_cmd_dics.append(self.sim_cmd)

        sim = self.load_yaml('simulation.yaml')

        self.add_params(sim.get('params', []))

        self.setup_buttons(sim.get('buttons', {}), self.sim_cmd)

        parent = self.tree_ctrl_joystick.GetParent()
        self.obj_get('tree_ctrl_joystick').Destroy()

        tree_ctrl1 = self.create_tree(parent, sim['subs'], None, None, self.sim_cmd)
        tree_ctrl1.SetBackgroundColour(wx.NullColour)
        setattr(self, 'tree_ctrl_joystick', tree_ctrl1)

        self.Bind(CT.EVT_TREE_ITEM_CHECKED, self.OnTreeChecked)

        # General

        aslan_gui.MyFrame.__do_layout(self)

        cond = lambda s: s.startswith('tab_')
        self.tab_names = \
            [self.name_get_cond(tab, cond=cond, def_ret='').replace('tab_', '', 1) for tab in self.all_tabs]

        new_btn_grps = (lambda btn_names, tab_names=self.tab_names:
                        [[self.obj_get('button_{}_{}'.format(bn, tn)) for tn in tab_names] for bn in btn_names])

        self.alias_grps = new_btn_grps(('rosbag', 'rviz', 'rqt'))

        for grp in self.alias_grps:
            wx.CallAfter(self.alias_sync, get_top(grp))
            s = get_tooltip_obj(grp[0])
            if s:
                for obj in grp[1:]:
                    set_tooltip_str(obj, s)

        self.stat_dic = {}
        for k in ['pmap']:
            self.stat_dic[k] = False
            rospy.Subscriber('pmap_stat', std_msgs.msg.Bool, self.stat_callback, callback_args=k)

        # Tools tab (Rosbag play stop pause, rostopic list + refresh, echo, info, rviz, nodes graph)

        tab1 = self.tab_tools
        self.all_tabs.append(tab1)

        self.simulation_cmd = {}
        self.all_cmd_dics.append(self.simulation_cmd)
        dictools = self.load_yaml('tools.yaml')

        self.add_params(dictools.get('params', []))

        self.setup_buttons(dictools.get('buttons'), self.simulation_cmd)

        self.topics_dic = self.load_yaml('topics.yaml')
        self.topics_list = []
        self.topics_echo_curr_topic = None
        self.topics_echo_proc = None
        self.topics_echo_thinf = None

        self.topics_echo_que = Queue.Queue()
        self.topics_echo_sum = 0
        thinf = th_start(self.topics_echo_show_th)
        self.all_th_infs.append(thinf)

        self.refresh_topics_list()

        # logout thread
        interval = 0.01

        thinf = th_start(self.logout_th, {'que': self.log_que_stdout, 'interval': interval})
        self.all_th_infs.append(thinf)
        thinf = th_start(self.logout_th, {'que': self.log_que_stderr, 'interval': interval})
        self.all_th_infs.append(thinf)
        thinf = th_start(self.logout_th, {'que': self.log_que, 'interval': interval})
        self.all_th_infs.append(thinf)

        # Thumbnail

        bm = wx.Bitmap(src_dir() + 'Aslan_Icon.png')
        icon = wx.EmptyIcon()
        icon.CopyFromBitmap(bm)
        self.SetIcon(icon)

    def logout_th(self, que, interval, ev):
        if que == self.log_que_stdout or que == self.log_que_stderr:
            while not ev.wait(0):
                try:
                    s = que.get(timeout=1)
                except Queue.Empty:
                    continue
                self.log_que.put(s)

                if interval <= 0:
                    continue

        else:  # == self.log_que
            f = None

            while not ev.wait(0):
                try:
                    s = que.get(timeout=1)
                except Queue.Empty:
                    continue
                print s.strip()
                sys.stdout.flush()

                s = cut_esc(s)
                if f:
                    f.write(s)
                    f.flush()

            if f:
                f.close()

    def stat_callback(self, msg, k):
        self.stat_dic[k] = msg.data
        if k == 'pmap':
            v = self.stat_dic.get(k)
            wx.CallAfter(self.label_point_cloud_bar.SetLabel, 'OK' if v else '')

    def name_get_cond(self, obj, cond=(lambda s: True), def_ret=None):
        return next((nm for nm in self.__dict__ if cond(nm) and getattr(self, nm) is obj), def_ret)

    def __do_layout(self):
        pass

    def OnSetupVehicleInterface(self, event):
        obj = self.button_vehicle_interface
        (pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
        self.update_func(pdic, gdic, prm)

    #
    # for Topics
    #
    def OnRefreshTopics(self, event):
        self.refresh_topics_list()

    def refresh_topics_list(self):
        lst = subprocess.check_output(['rostopic', 'list']).strip().split('\n')
        panel = self.panel_topics_list
        szr = self.sizer_topics_list
        for obj in self.topics_list:
            szr.Remove(obj)
            obj.Destroy()
        self.topics_list = []
        for topic in lst:
            obj = wx.HyperlinkCtrl(panel, wx.ID_ANY, topic, '')
            self.Bind(wx.EVT_HYPERLINK, self.OnTopicLink, obj)
            szr.Add(obj, 0, wx.LEFT, 4)
            fix_link_color(obj)
            self.topics_list.append(obj)
        szr.Layout()
        panel.SetVirtualSize(szr.GetMinSize())

        # info clear
        lb = self.label_topics_info
        lb.SetLabel('')

        # echo clear
        self.topics_proc_th_end()

        # wait que clear
        while self.topics_echo_que.qsize() > 0:
            time.sleep(0.1)

        tc = self.text_ctrl_topics_echo
        tc.Enable(False)
        wx.CallAfter(tc.Clear)
        wx.CallAfter(tc.Enable, True)
        self.topics_echo_sum = 0
        self.topic_echo_curr_topic = None

    def OnEcho(self, event):
        if self.checkbox_topics_echo.GetValue() and self.topic_echo_curr_topic:
            self.topics_proc_th_start(self.topic_echo_curr_topic)
        else:
            self.topics_proc_th_end()

    def OnTopicLink(self, event):
        obj = event.GetEventObject()
        topic = obj.GetLabel()
        self.topic_echo_curr_topic = topic

        # info
        info = subprocess.check_output(['rostopic', 'info', topic]).strip()
        lb = self.label_topics_info
        lb.SetLabel(info)
        lb.GetParent().FitInside()

        # echo
        self.topics_proc_th_end()
        if self.checkbox_topics_echo.GetValue():
            self.topics_proc_th_start(topic)

    def topics_proc_th_start(self, topic):
        out = subprocess.PIPE
        err = subprocess.STDOUT
        self.topics_echo_proc = psutil.Popen(['rostopic', 'echo', topic], stdout=out, stderr=err)

        self.topics_echo_thinf = th_start(self.topics_echo_th)

    def topics_proc_th_end(self):
        thinf = self.topics_echo_thinf
        if thinf:
            th_end(thinf)
            self.topics_echo_thinf = None

        proc = self.topics_echo_proc
        if proc:
            terminate_children(proc)
            terminate(proc)
            # proc.wait()
            self.topics_echo_proc = None

    def topics_echo_th(self, ev):
        if not self.topics_echo_proc:
            return
        file = self.topics_echo_proc.stdout
        fl = fcntl.fcntl(file.fileno(), fcntl.F_GETFL)
        fcntl.fcntl(file.fileno(), fcntl.F_SETFL, fl | os.O_NONBLOCK)

        while not ev.wait(0):
            try:
                s = file.read(1)
            except:
                continue
            if not s:
                break
            if self.checkbox_topics_echo.GetValue():
                self.topics_echo_que.put(s)

        que_clear(self.topics_echo_que)

    def topics_echo_show_th(self, ev):
        que = self.topics_echo_que
        interval = self.topics_dic.get('gui_update_interval_ms', 100) * 0.001
        chars_limit = self.topics_dic.get('gui_chars_limit', 10000)
        tc = self.text_ctrl_topics_echo
        while not ev.wait(interval):
            qsz = que.qsize()
            if qsz <= 0:
                continue
            if qsz > chars_limit:
                over = qsz - chars_limit
                for i in range(over):
                    try:
                        que.get(timeout=1)
                    except Queue.Empty:
                        break
                qsz = chars_limit
            arr = []
            for i in range(qsz):
                try:
                    s = que.get(timeout=1)
                except Queue.Empty:
                    s = ''
                arr.append(s)
            s = ''.join(arr)

            self.topics_echo_sum += len(s)
            rm_chars = 0
            if self.topics_echo_sum > chars_limit:
                rm_chars = self.topics_echo_sum - chars_limit
                self.topics_echo_sum = chars_limit

            if self.checkbox_topics_echo.GetValue():
                wx.CallAfter(append_tc_limit, tc, s, rm_chars)

    # Configuration Tab

    def OnSensingDriver(self, event):
        self.OnChecked_obj(event.GetEventObject())

    def OnROSbagRecord(self, event):
        self.dlg_rosbag_record.show()
        obj = event.GetEventObject()
        set_val(obj, False)

    # Infos

    def OnInfoVM(self, event):
        message = "The Visualization button automatically unzips the .stl model of the vehicle of your choice and " \
                  "launches RVIZ.\n\n TIP: Try increasing the alpha parameter on RVIZ. \n\n " \
                  "For more info, visit: vehicle_configuration"
        self.dialog_info(message)

    def OnInfoSensors(self, event):
        message = "Select the Sensors Setup for your vehicle.\n\nThe camera drivers require additional packages " \
                  "to be installed, as described here: sensing/drivers/camera/README.md\n\n " \
                  "The UMRR Radar Driver, requires further configuration and interfacing, visit: sensing/drivers/radar/umrr_driver"
        self.dialog_info(message)

    def OnInfoTF1(self, event):
        message = "TF maintains the relationship between coordinate frames in a tree structure and lets the user " \
                  "transform points etc between any two coordinate frames.\n\nMeasure the distance between your " \
                  "localizer (lidar) sensor and the baselink of your vehicle (center of rear axis).\n\nFor " \
                  "more info, visit: http://wiki.ros.org/tf"
        self.dialog_info(message)

    def OnInfoPCD(self, event):
        message = "The package pcd_loader is loading a point cloud map. Choose the .pcd map you want to" \
                  " load, to update the launch file argument 'pcd_file'.\n\nFor more info, visit: mapping/pcd_loader"
        self.dialog_info(message)

    def OnInfoTF2(self, event):
        message = "The package map_tf_generator is applying a transformation from the map frame to the world frame. It requires " \
                  "the point cloud map to be loaded.\n\nTIP: Wait for the pcd_loader to load your map fully and then launch this package.\n\n" \
                  "For more info, visit: mapping/map_tf_generator"
        self.dialog_info(message)

    def OnInfoFilters(self, event):
        message = "These packages are filtering the output point cloud from the lidar or the radar sensor.\n\n" \
                  "Voxel Grid Filter is downsampling the point cloud generated using VoxelGrid filtering, " \
                  "for more info visit: http://pointclouds.org/documentation/tutorials/voxel_grid.php \n\n" \
                  "Ray Ground Filter is performing ground removal by masking the point cloud generated. Adapt the parameters based on the position of your sensors" \
                  "in the car\n\n" \
                  "TIP: From the config dialogs, you can select the point cloud topic you would like to filter.\n\n" \
                  "For more info, visit: src/sensing/filters "
        self.dialog_info(message)

    def OnInfoNDT(self, event):
        message = "NDT Mapping is extracting the 3D map from a recorded lidar scan of an area.\n" \
                  "NDT Matching is responsible for localizing the vehicle in the pre recorded .pcd map by using the " \
                  "Normal Distributions Transform technique.\n" \
                  "NDT Matching Monitor is performing a health monitoring and reinitialization for ndt_matching.\n\n" \
                  "For more info, visit: src/localization/packages/lidar_localizer\n\n" \
                  "For more info on NDT, visit: https://www.streetdrone.com/post/hello-world-localisation-ndt  "
        self.dialog_info(message)

    def OnInfoRoute1(self, event):
        message = "The Waypoints Loader is responsible for loading the path waypoints from a csv file.\n" \
                  "The Waypoints Saver is responsible for generating waypoints by recreating the path the vehicle followed while mapping.\n\n" \
                  "TIP1: To generate waypoints you need to replay a rosbag, " \
                  "load the pcd map generated from that rosbag, localize in the map " \
                  "and then start extracting the path.\n\n " \
                  "TIP2: To visualize the output on rviz, add the topic MarkerArray\n\n" \
                  "For more info, visit: src/planning/motion/waypoint_maker "
        self.dialog_info(message)

    def OnInfoRoute2(self, event):
        message = "Lane Selection is launching the traffic_waypoints and the lane_select nodes. It's responsible for" \
                  " drawing lanes between the waypoints for the vehicle to follow, by finding the closest waypoint to the vehicle's current position.\n\n" \
                  "For more info, visit: src/planning/mission/lane_planner"
        self.dialog_info(message)

    def OnInfoTraj1(self, event):
        message = "This package is based on the A* search algorithm for path planning. It is responsible for " \
                  "finding the optimal(shortest) path for the vehicle to follow in order to reach the next target waypoint.\n\n" \
                  "Obstacle Search: This is responsible for detecting obstacles within a specific (avoidance) distance and updating the path of the car.\n" \
                  "Obstacle Sim: This can be used to simulate obstacles on rviz. It allows the user to draw obstacles using the 2d Nav Goal tool.\n" \
                  "Velocity Request: Based on the distance of the vehicle and the obstacle in it's path, this node is responsible for accelerating, decelerating and stopping the vehicle.\n\n" \
                  "For more info, visit: src/planning/motion/astar_planner "
        self.dialog_info(message)

    def OnInfoTraj2(self, event):
        message = "Pure pursuit: This is responsible for calculating the twist command for controlling the vehicle, by " \
                  "fitting a curve between the vehicle and the next target waypoint, " \
                  "based on the lookahead distance.\n" \
                  "Low Pass filtering: This is responsible for filtering the twist_raw command by applying a low pass filter.\n\n" \
                  "For more info, visit: src/planning/motion/waypoint_follower"
        self.dialog_info(message)

    def OnInfoSup(self, event):
        message = "High level supervisor node, monitoring the health of the system and publishing diagnostic messages for each package.\n\n" \
                  "For more info, visit: src/supervisor/check_message_alive"
        self.dialog_info(message)

    def OnInfoVI(self, event):
        message = "The StreetDrone Vehicle Interface is the bridge between ROS and the StreetDrone embedded system (XCU) CAN input. " \
                  "Control loop feedback mechanisms are also implemented in the interface for vehicle speed and steer control.\n\n" \
                  "TIP: If you are using PEAK_USB, you need to click on Enable SocketCAN before launching the node.\n\n" \
                  "For more info, visit: src/vehicle_interface"
        self.dialog_info(message)

    def OnInfoSim(self, event):
        message = "Gazebo simulation packages for the StreetDrone Twizy vehicle. Choose the world you would like to use from the dropdown menu." \
                  " Also select whether you would like to launch rviz alongside, with a default configuration.\n\n" \
                  "For more info, visit: vehicle_simulation/gazebo/sd_twizy_model"
        self.dialog_info(message)

    def OnInfoControl(self, event):
        message = "The simulation can be controlled by the /sd_control topic, published by:\n " \
                  "1. Launching the StreetDrone Vehicle Interface, when the sd_simulation_mode param flag is set to true.\n" \
                  "2. Launching the Joystick controller. You can map specific buttons for your joystick using the parameters on config, visit http://wiki.ros.org/joy\n" \
                  "3. Launching the Keyboard controller node. Follow the instructions on the pop up terminal window.\n\n" \
                  "For more info, visit: vehicle_simulation/gazebo/sd_twizy_model"
        self.dialog_info(message)

    def dialog_info(self, message):
        dialog = wx.MessageDialog(self.notebook_1, message, caption=wx.MessageBoxCaptionStr,
                                  style=wx.OK | wx.CENTRE, pos=wx.DefaultPosition)
        dialog.ShowModal()
        dialog.Destroy()

    def create_checkboxes(self, dic, panel, sizer, probe_dic, run_dic, bind_handler):
        # if 'name' not in dic:
        #     return
        obj = None
        bdr_flg = wx.ALL
        if 'subs' in dic:
            lst = []
            for d in dic['subs']:
                self.create_checkboxes(d, panel, lst, probe_dic, run_dic, bind_handler)
            if dic['name']:
                obj = static_box_sizer(panel, dic.get('name'))
                set_tooltip(obj.GetStaticBox(), dic)
            else:
                obj = wx.BoxSizer(wx.VERTICAL)
            for (o, flg) in lst:
                obj.Add(o, 0, wx.EXPAND | flg, 0)
        else:
            obj = wx.CheckBox(panel, wx.ID_ANY, dic['name'])
            set_tooltip(obj, dic)
            self.Bind(wx.EVT_CHECKBOX, bind_handler, obj)
            bdr_flg = wx.LEFT | wx.RIGHT
            if 'probe' in dic:
                probe_dic[obj] = (dic['probe'], None)
            if 'run' in dic:
                run_dic[obj] = (dic['run'], None)
            else:
                gdic = self.gdic_get_1st(dic)
                self.add_cfg_info(obj, obj, dic.get('name'), None, gdic, False, None)
        if sizer is not None:
            sizer.append((obj, bdr_flg))
        else:
            panel.SetSizer(obj)

    def alias_grp_top_obj(self, obj):
        return get_top(self.alias_grp_get(obj), obj)

    def alias_grp_get(self, obj):
        return next((grp for grp in self.alias_grps if obj in grp), [])

    def name_get(self, obj):
        return next((nm for nm in self.__dict__ if getattr(self, nm) is obj), None)

    def is_toggle_button(self, obj):
        return self.name_get(obj).split('_')[0] == 'button' and getattr(obj, 'GetValue', None)

    def alias_sync(self, obj, v=None):
        en = None
        if getattr(obj, 'IsEnabled', None):
            (key, en) = enables_get_last(obj)
            if not key:
                en = obj.IsEnabled()
        grp = self.alias_grp_get(obj)
        if getattr(obj, 'GetValue', None):
            v = obj.GetValue()
        for o in grp:
            if o is obj:
                continue

            if en is not None and o.IsEnabled() != en and not self.is_toggle_button(o):
                # if key:
                #     enable_set(o, key, en)
                # else:
                o.Enable(en)
            if v is not None and getattr(o, 'SetValue', None):
                set_val(o, v)
                if getattr(o, 'SetInsertionPointEnd', None):
                    o.SetInsertionPointEnd()

    def create_tree(self, parent, items, tree, item, cmd_dic):
        name = items.get('name', '')
        if tree is None:
            style = wx.TR_HAS_BUTTONS | wx.TR_NO_LINES | wx.TR_HIDE_ROOT | wx.TR_DEFAULT_STYLE | wx.SUNKEN_BORDER
            tree = CT.CustomTreeCtrl(parent, wx.ID_ANY, agwStyle=style)

            # for disable wrong scrolling at checked
            tree.AcceptsFocus = MethodType(lambda self: False, tree, CT.CustomTreeCtrl)

            item = tree.AddRoot(name, data=tree)
            tree.Bind(wx.EVT_MOTION, self.OnTreeMotion)
        else:
            ct_type = 1 if 'cmd' in items else 0  # 1:checkbox type
            item = tree.AppendItem(item, name, ct_type=ct_type)
            if 'desc' in items:
                item.SetData(items.get('desc'))
            if 'cmd' in items:
                cmd_dic[item] = (items['cmd'], None)

                pdic = self.load_dic_pdic_setup(name, items)
                pnl = wx.Panel(tree, wx.ID_ANY)
                add_objs = []
                gdic = self.gdic_get_1st(items)
                if 'param' in items:
                    self.new_link(item, name, pdic, gdic, pnl, 'config', items.get('param'), add_objs)
                else:
                    self.add_cfg_info(item, item, name, None, gdic, False, None)
                szr = sizer_wrap(add_objs, wx.HORIZONTAL, flag=wx.ALIGN_CENTER_VERTICAL, parent=pnl)
                szr.Fit(pnl)
                tree.SetItemWindow(item, pnl)

        for sub in items.get('subs', []):
            self.create_tree(parent, sub, tree, item, cmd_dic)
        return tree

    def new_link(self, item, name, pdic, gdic, pnl, link_str, prm_name, add_objs):
        lkc = None
        if 'no_link' not in gdic.get('flags', []):
            lkc = wx.HyperlinkCtrl(pnl, wx.ID_ANY, link_str, "")
            fix_link_color(lkc)
            self.Bind(wx.EVT_HYPERLINK, self.OnHyperlinked, lkc)
            if len(add_objs) > 0:
                add_objs += [wx.StaticText(pnl, wx.ID_ANY, ' ')]
            add_objs += [wx.StaticText(pnl, wx.ID_ANY, '['), lkc, wx.StaticText(pnl, wx.ID_ANY, ']')]
        prm = self.get_param(prm_name)
        self.add_cfg_info(lkc if lkc else item, item, name, pdic, gdic, False, prm)

    def OnTreeMotion(self, event):
        tree = event.GetEventObject()
        pt = event.GetPosition()
        event.Skip()
        (item, flags) = tree.HitTest(pt)
        if flags & CT.TREE_HITTEST_ONITEMLABEL == 0:
            return
        text = item.GetData()
        if not text:
            return
        x = item.GetX()
        y = item.GetY()
        w = item.GetWidth()
        h = item.GetHeight()
        (x, y) = tree.CalcScrolledPosition(x, y)
        iw = tree.GetItemWindow(item)
        w -= iw.GetSize()[0] if iw else 0
        if not wx.Rect(x, y, w, h).Contains(pt):
            return
        (x, y) = tree.ClientToScreen((x, y))
        self.tip_info = (tree, text, wx.Rect(x, y, w, h))
        if getattr(self, 'tip_timer', None) is None:
            self.tip_timer = wx.Timer(self)
            self.Bind(wx.EVT_TIMER, self.OnTipTimer, self.tip_timer)
        self.tip_timer.Start(200, oneShot=True)

    def OnTipTimer(self, event):
        if getattr(self, 'tip_info', None):
            (tree, text, rect) = self.tip_info
            (w, h) = self.GetSize()
            wx.TipWindow(tree, text, maxLength=w, rectBound=rect)

    def OnTreeChecked(self, event):
        self.OnChecked_obj(event.GetItem())

    def OnChecked_obj(self, obj):
        self.OnLaunchKill_obj(obj)

    def OnHyperlinked(self, event):
        self.OnHyperlinked_obj(event.GetEventObject())

    def OnHyperlinked_obj(self, obj):
        (pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
        if pdic is None or prm is None:
            return
        dic_list_push(gdic, 'dialog_type', 'config')
        klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
        dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
        show_modal(dlg)
        dic_list_pop(gdic, 'dialog_type')

    def update_func(self, pdic, gdic, prm):
        for var in prm.get('vars', []):
            name = var.get('name')
            gdic_v = gdic.get(name, {})
            func = gdic_v.get('func')
            if func is None and name in pdic:
                continue
            v = var.get('v')
            if func is not None:
                v = eval(func) if type(func) is str else func()
            pdic[name] = v

            hook = gdic_v.get('update_hook')
            if hook:
                hook(v)
            hook_var = gdic_v.get('hook_var', {})
            every_time = 'every_time' in hook_var.get('flags', [])
            if var == gdic.get('update_func_arg_var') or every_time:
                hook = hook_var.get('hook')
                if hook:
                    hook(hook_var.get('args', {}))

        if 'pub' in prm:
            self.publish_param_topic(pdic, prm)
        self.rosparam_set(pdic, prm)
        self.update_depend_enable(pdic, gdic, prm)

    def update_depend_enable(self, pdic, gdic, prm):
        for var in prm.get('vars', []):
            name = var.get('name')
            gdic_v = gdic.get(name, {})
            depend = gdic_v.get('depend')
            if depend is None:
                continue
            vp = gdic_v.get('var')
            if vp is None:
                continue
            v = pdic.get(depend)
            if v is None:
                continue
            depend_bool = eval(gdic_v.get('depend_bool', 'lambda v : bool(v)'))
            v = depend_bool(v)
            enables_set(vp, 'depend', v)

    def publish_param_topic(self, pdic, prm):
        pub = prm['pub']
        klass_msg = globals()[prm['msg']]
        msg = klass_msg()

        for (name, v) in pdic.items():
            if prm.get('topic') == '/twist_cmd' and name == 'twist.angular.z':
                v = -v
            (obj, attr) = msg_path_to_obj_attr(msg, name)
            if obj and attr in obj.__slots__:
                type_str = obj._slot_types[obj.__slots__.index(attr)]
                setattr(obj, attr, str_to_rosval(v, type_str, v))

        if 'stamp' in prm.get('flags', []):
            (obj, attr) = msg_path_to_obj_attr(msg, 'header.stamp')
            setattr(obj, attr, rospy.get_rostime())

        pub.publish(msg)

    def obj_to_pdic_gdic_prm(self, obj, sys=False):
        info = self.config_dic.get(obj)
        if info is None:
            sys_prm = self.get_param('sys')
            prm_chk = lambda prm: prm is sys_prm if sys else prm is not sys_prm
            info = next((v for v in self.config_dic.values() if v.get('obj') is obj and prm_chk(v.get('param'))), None)
            if info is None:
                return (None, None, None)
        pdic = info.get('pdic')
        prm = info.get('param')
        gdic = info.get('gdic')
        return (pdic, gdic, prm)

    def obj_to_gdic(self, obj, def_ret=None):
        (_, gdic, _) = self.obj_to_pdic_gdic_prm(obj) if obj else (None, None, None)
        return gdic if gdic else def_ret

    def cfg_obj_dic(self, arg_dic, sys=False, def_ret=(None, {})):
        sys_prm = self.get_param('sys')
        prm_chk = {
            True: (lambda prm: prm is sys_prm),
            False: (lambda prm: prm is not sys_prm),
            None: (lambda prm: True)}.get(sys)
        arg_dic_chk = lambda dic: all([dic.get(k) == v for (k, v) in arg_dic.items()])
        return next(((cfg_obj, dic) for (cfg_obj, dic) in self.config_dic.items() \
                     if arg_dic_chk(dic) and prm_chk(dic.get('param'))), def_ret)

    def cfg_dic(self, arg_dic, sys=False, def_ret={}):
        (_, dic) = self.cfg_obj_dic(arg_dic, sys=sys, def_ret=(None, def_ret))
        return dic

    def cfg_prm_to_obj(self, arg_dic, sys=False):
        return self.cfg_dic(arg_dic, sys=sys).get('obj')

    def rosparam_set(self, pdic, prm):
        rosparams = None
        for var in prm.get('vars', []):
            name = var['name']
            if 'rosparam' not in var or name not in pdic:
                continue
            rosparam = var['rosparam']
            v = pdic.get(name)
            v = str(v)
            cvdic = {'True': 'true', 'False': 'false'}
            if v in cvdic:
                v = cvdic.get(v)
            if rosparams is None:
                cmd = ['rosparam', 'list']
                rosparams = subprocess.check_output(cmd).strip().split('\n')
            nm = rosparam
            nm = ('/' if len(nm) > 0 and nm[0] != '/' else '') + nm
            exist = nm in rosparams
            if exist:
                cmd = ['rosparam', 'get', rosparam]
                ov = subprocess.check_output(cmd).strip()
                if ov == v:
                    continue
            elif v == '':
                continue
            cmd = ['rosparam', 'set', rosparam, v] if v != '' else ['rosparam', 'delete', rosparam]
            print(cmd)
            subprocess.call(cmd)

    def obj_to_add_args(self, obj, msg_box=True):
        (pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
        if pdic is None or prm is None:
            return None

        if 'open_dialog' in gdic.get('flags', []) and msg_box:
            dic_list_push(gdic, 'dialog_type', 'open')
            klass_dlg = globals().get(gdic_dialog_name_get(gdic), MyDialogParam)
            dlg = klass_dlg(self, pdic=pdic, gdic=gdic, prm=prm)
            dlg_ret = show_modal(dlg)
            dic_list_pop(gdic, 'dialog_type')
            if dlg_ret != 0:
                return False

        self.update_func(pdic, gdic, prm)
        s = ''

        vars = []
        for var in prm.get('vars'):
            cmd_param = var.get('cmd_param')
            if cmd_param:
                vars.append(var)

        for var in vars[:]:  # copy
            cmd_param = var.get('cmd_param')
            if cmd_param.get('tail'):
                vars.remove(var)
                vars.append(var)

        for var in vars[:]:  # copy
            name = var.get('name')
            flags = gdic.get(name, {}).get('flags', [])
            if 'hide' in flags or 'disable' in flags:
                vars.remove(var)

        for var in vars:
            cmd_param = var.get('cmd_param')
            name = var.get('name')
            v = pdic.get(name)
            if (v is None or v == '') and 'default' in cmd_param:
                v = cmd_param.get('default')
            if dic_eval_if_str(self, cmd_param, 'must') and (v is None or v == ''):
                print 'cmd_param', name, 'is required'
                if msg_box:
                    wx.MessageBox('cmd_param ' + name + ' is required')
                return False
            if dic_eval_if_str(self, cmd_param, 'only_enable') and not v:
                continue
            if dic_eval_if_str(self, cmd_param, 'only_disable') and v:
                continue
            name = cmd_param.get('var_name', name)
            unpack = cmd_param.get('unpack')
            if unpack is not None:
                v = ' '.join(v.split(unpack))
            add = ''
            dash = cmd_param.get('dash')
            if dash is not None:
                add += dash + name
            delim = cmd_param.get('delim')
            if delim is not None:
                str_v = str(v)
                if var.get('kind') is None:
                    str_v = adjust_num_str(str_v)
                if var.get('kind') == 'path':
                    str_v = path_expand_cmd(str_v)
                    str_v = os.path.expandvars(os.path.expanduser(str_v))

                    relpath_from = var.get('relpath_from')
                    if relpath_from:
                        relpath_from = path_expand_cmd(relpath_from)
                        relpath_from = os.path.expandvars(os.path.expanduser(relpath_from))
                        str_v = os.path.relpath(str_v, relpath_from)
                add += delim + str_v
            if add != '':
                s += add + ' '
        return s.strip(' ').split(' ') if s != '' else None

    def OnLaunchKill(self, event):
        self.OnLaunchKill_obj(event.GetEventObject())

    def OnLaunchKill_obj(self, obj):
        self.alias_sync(obj)
        obj = self.alias_grp_top_obj(obj)
        v = obj.GetValue()  # true, false depends on selected or not
        add_args = self.obj_to_add_args(obj, msg_box=v)  # no open dialog at kill
        if add_args is False:
            set_val(obj, not v)
            return
        (cmd_dic, _, proc_bak) = self.obj_to_cmd_dic_cmd_proc(obj)
        self.launch_kill_proc(obj, cmd_dic, add_args=add_args)
        (_, _, proc) = self.obj_to_cmd_dic_cmd_proc(obj)
        if proc != proc_bak:
            self.toggle_enable_obj(obj)

    def OnROSbagPlay(self, event):
        obj = event.GetEventObject()

        play = self.button_play_rosbag_play
        stop = self.button_stop_rosbag_play
        pause = self.button_pause_rosbag_play

        (_, _, prm) = self.obj_to_pdic_gdic_prm(play)
        var = self.get_var(prm, 'sim_time', {})

        if obj == play:
            var['v'] = True
            self.OnLaunchKill_obj(play)
            button_color_change(play)
            set_val(stop, False)
            set_val(pause, False)
        elif obj == stop:
            set_val(stop, True)
            set_val(play, False)
            set_val(pause, False)
            var['v'] = False
            self.OnLaunchKill_obj(play)
            button_color_change(stop)
        elif obj == pause:
            (_, _, proc) = self.obj_to_cmd_dic_cmd_proc(play)
            if proc:
                proc.stdin.write(' ')

    def obj_to_cmd_dic(self, obj):
        return next((cmd_dic for cmd_dic in self.all_cmd_dics if obj in cmd_dic), None)

    def obj_to_cmd_dic_cmd_proc(self, obj):
        cmd_dic = self.obj_to_cmd_dic(obj)
        if cmd_dic is None:
            return (None, None, None)
        (cmd, proc) = cmd_dic.get(obj, (None, None))
        return cmd_dic, cmd, proc

    def toggle_enable_obj(self, obj):
        objs = []
        pfs = ['button_play_', 'button_stop_', 'button_pause_',
               'button_ref_', 'text_ctrl_']
        key = self.obj_key_get(obj, pfs)
        if key:
            objs += self.key_objs_get(pfs, key)

        gdic = self.obj_to_gdic(obj, {})
        objs += [eval_if_str(self, e) for e in gdic.get('ext_toggle_enables', [])]

        self.toggle_enables(objs)

    def toggle_enables(self, objs):
        for obj in objs:
            if getattr(obj, 'IsEnabled', None):
                en = enables_get(obj, 'toggle', obj.IsEnabled())
                enables_set(obj, 'toggle', not en)
                self.alias_sync(obj)

    def obj_key_get(self, obj, pfs):
        name = self.name_get(obj)
        if name is None:
            return None
        return next((name[len(pf):] for pf in pfs if name[:len(pf)] == pf), None)

    def key_objs_get(self, pfs, key):
        return [self.obj_get(pf + key) for pf in pfs if self.obj_get(pf + key)]

    def obj_to_varpanel(self, obj, var_name):
        gdic = self.obj_to_gdic(obj, {})
        return gdic.get(var_name, {}).get('var')

    def obj_to_varpanel_tc(self, obj, var_name):
        vp = self.obj_to_varpanel(obj, var_name)
        return vp.tc if vp and vp.tc else None

    def load_yaml(self, filename, def_ret=None):
        return load_yaml(filename, def_ret)

    def add_params(self, params):
        for prm in params:
            if 'topic' in prm and 'msg' in prm:
                klass_msg = globals()[prm['msg']]
                prm['pub'] = rospy.Publisher(prm['topic'], klass_msg, latch=True, queue_size=10)
        self.params += params

    def setup_buttons(self, d, run_dic):
        for (k, d2) in d.items():
            pfs = ['button_', 'checkbox_']
            obj = next((self.obj_get(pf + k) for pf in pfs if self.obj_get(pf + k)), None)
            if not obj:
                s = 'button_' + k
                obj = StrValObj(s, False)
                setattr(self, s, obj)
            if not d2 or type(d2) is not dict:
                continue

            if 'run' in d2:
                run_dic[obj] = (d2['run'], None)
            set_tooltip(obj, d2)
            gdic = self.gdic_get_1st(d2)
            if 'param' in d2:
                pdic = self.load_dic_pdic_setup(k, d2)
                prm = self.get_param(d2.get('param'))
                for var in prm.get('vars'):
                    name = var.get('name')
                    if name not in pdic and 'v' in var:
                        pdic[name] = var.get('v')

                for (name, v) in pdic.items():
                    restore = eval(gdic.get(name, {}).get('restore', 'lambda a : None'))
                    restore(v)

                self.add_cfg_info(obj, obj, k, pdic, gdic, False, prm)

                pnls = [gdic.get(var.get('name'), {}).get('panel') for var in prm.get('vars')]
                for pnl in [gdic.get('panel')] + pnls:
                    if pnl:
                        self.set_param_panel(obj, eval_if_str(self, pnl))
            else:
                self.add_cfg_info(obj, obj, k, None, gdic, False, None)

    def obj_get(self, name):
        return getattr(self, name, None)

    def log_th(self, file, que, ev):
        while not ev.wait(0):
            s = file.readline()
            if not s:
                break
            que.put(s)

    def proc_to_cmd_dic_obj(self, proc):
        for cmd_dic in self.all_cmd_dics:
            obj = next((obj for (obj, v) in cmd_dic.items() if proc in v), None)
            if obj:
                return (cmd_dic, obj)
        return (None, None)

    def OnClose(self, event):
        if self.quit_select() != 'quit':
            return

        # kill_all
        for proc in self.all_procs[:]:  # copy
            (_, obj) = self.proc_to_cmd_dic_obj(proc)
            self.launch_kill(False, 'dmy', proc, obj=obj)

        for thinf in self.all_th_infs:
            th_end(thinf)

        self.Destroy()

    def quit_select(self):
        lst = [
            ('Save and Quit', ['save', 'quit']),
            ('Save to param.yaml', ['save']),
            ('Quit without saving', ['quit'])
        ]
        choices = [s for (s, _) in lst]
        dlg = wx.SingleChoiceDialog(self, 'Select: ', '', choices)
        if dlg.ShowModal() != wx.ID_OK:
            return 'not quit'

        i = dlg.GetSelection()  # index of choices
        (_, f) = lst[i]
        if 'save' in f:
            self.save_param_yaml()
        return 'quit' if 'quit' in f else 'not quit'

    def save_param_yaml(self):
        save_dic = {}
        for (name, pdic) in self.load_dic.items():
            if pdic and pdic != {}:
                prm = self.cfg_dic({'name': name, 'pdic': pdic}).get('param', {})
                no_saves = prm.get('no_save_vars', [])
                pdic = pdic.copy()
                for k in pdic.keys():
                    if k in no_saves:
                        del pdic[k]
                save_dic[name] = pdic

        names = []
        for proc in self.all_procs:
            (_, obj) = self.proc_to_cmd_dic_obj(proc)
            name = self.cfg_dic({'obj': obj}).get('name')
            names.append(name)
        if 'booted_cmds' not in save_dic:
            save_dic['booted_cmds'] = {}
        save_dic.get('booted_cmds')['names'] = names

        if save_dic != {}:
            dir = src_dir()
            print('saving param.yaml')
            f = open(dir + 'param.yaml', 'w')
            s = yaml.dump(save_dic, default_flow_style=False)
            f.write(s)
            f.close()

    def launch_kill_proc(self, obj, cmd_dic, add_args=None):
        if obj not in cmd_dic:
            set_val(obj, False)
            print('not implemented.')
            return
        v = obj.GetValue()

        (cmd, proc) = cmd_dic[obj]
        if not cmd:
            set_val(obj, False)

        proc = self.launch_kill(v, cmd, proc, add_args, obj=obj)

        (cfg_obj, dic) = self.cfg_obj_dic({'obj': obj})
        if cfg_obj and dic.get('run_disable'):
            cfg_obj.Enable(not v)

        cmd_dic[obj] = (cmd, proc)

    def launch_kill(self, v, cmd, proc, add_args=None, sigint=None, obj=None, kill_children=None):
        msg = None
        msg = 'already launched.' if v and proc else msg
        msg = 'already terminated.' if not v and proc is None else msg
        msg = 'cmd not implemented.' if not cmd else msg
        if msg is not None:
            print(msg)
            return proc

        if v:
            args = shlex.split(cmd)
            if add_args:
                args += add_args
            print(args)  # for debug

            f = self.obj_to_gdic(obj, {}).get('stdout_func')
            f = eval_if_str(self, f)
            f = f if f else self.log_th

            out = subprocess.PIPE if f else None
            err = subprocess.STDOUT if f else None
            if f == self.log_th:
                err = subprocess.PIPE

            shell = (len(args) > 0 and args[0] == 'do_shell_exec')
            if shell:
                args = ' '.join(args[1:])
            proc = psutil.Popen(args, stdin=subprocess.PIPE, stdout=out, stderr=err, shell=shell)
            self.all_procs.append(proc)

            if f == self.log_th:
                thinf = th_start(f, {'file': proc.stdout, 'que': self.log_que_stdout})
                self.all_th_infs.append(thinf)
                thinf = th_start(f, {'file': proc.stderr, 'que': self.log_que_stderr})
                self.all_th_infs.append(thinf)
            elif f:
                thinf = th_start(f, {'file': proc.stdout})
                self.all_th_infs.append(thinf)
        else:
            flags = self.obj_to_gdic(obj, {}).get('flags', [])
            if sigint is None:
                sigint = 'SIGTERM' not in flags
            if kill_children is None:
                kill_children = 'kill_children' in flags
            if kill_children:
                terminate_children(proc, sigint)
            terminate(proc, sigint)
            enables_set(obj, 'proc_wait', False)
            th_start(proc_wait_thread, {'proc': proc, 'obj': obj})
            if proc in self.all_procs:
                self.all_procs.remove(proc)
            proc = None

        return proc

    def gdic_get_1st(self, dic):
        gdic = dic.get('gui', {})
        gdic['update_func'] = self.update_func
        return gdic

    def load_dic_pdic_setup(self, name, dic):
        name = dic.get('share_val', dic.get('name', name))
        pdic = self.load_dic.get(name, {})
        self.load_dic[name] = pdic
        return pdic

    def get_var(self, prm, var_name, def_ret=None):
        return next((var for var in prm.get('vars') if var.get('name') == var_name), def_ret)

    def get_param(self, prm_name):
        return next((prm for prm in self.params if prm['name'] == prm_name), None)

    def add_cfg_info(self, cfg_obj, obj, name, pdic, gdic, run_disable, prm):
        self.config_dic[cfg_obj] = {'obj': obj, 'name': name, 'pdic': pdic, 'gdic': gdic,
                                    'run_disable': run_disable, 'param': prm}

    def set_param_panel(self, obj, parent):
        (pdic, gdic, prm) = self.obj_to_pdic_gdic_prm(obj)
        panel = ParamPanel(parent, frame=self, pdic=pdic, gdic=gdic, prm=prm)
        sizer_wrap((panel,), wx.VERTICAL, 0, wx.EXPAND, 0, parent)
        k = 'ext_toggle_enables'
        gdic[k] = gdic.get(k, []) + [panel]

    def stdout_file_search(self, file, k):
        s = ''
        while True:
            c = file.read(1)
            if not c:
                return None
            if c != '\r' and c != '\n':
                s += c
                continue
            s = s.strip()
            if k in s:
                break
            s = ''
        i = s.find(k) + len(k)
        return s[i:]

    def point_cloud_progress_bar(self, file, ev):
        obj = self.button_pcd
        (pdic, _, _) = self.obj_to_pdic_gdic_prm(obj)
        n = len(pdic.get('pcd_file', '').split(','))
        if n == 0:
            return
        i = 0
        while not ev.wait(0):
            s = self.stdout_file_search(file, 'load ')
            if not s:
                break
            err_key = 'failed '
            if s[:len(err_key)] != err_key:
                i += 1
            else:
                i -= 1
                print s
            wx.CallAfter(self.label_point_cloud_bar.set, 100 * i / n)
        wx.CallAfter(self.label_point_cloud_bar.clear)

    def rosbag_play_progress_bar(self, file, ev):
        while not ev.wait(0):
            s = self.stdout_file_search(file, 'Duration:')
            if not s:
                break
            lst = s.split()
            pos = str_to_float(lst[0])
            # lst[1] is '/'
            total = str_to_float(lst[2])
            if total == 0:
                continue
            prg = int(100 * pos / total + 0.5)
            pos = str(int(pos))
            total = str(int(total))

            wx.CallAfter(self.label_rosbag_play_bar.set, prg)
        wx.CallAfter(self.label_rosbag_play_bar.clear)


def load_yaml(filename, def_ret=None):
    source_dir = src_dir()
    path = source_dir + filename
    if not os.path.isfile(path):
        return def_ret
    print('loading ' + filename)
    f = open(source_dir + filename, 'r')
    d = yaml.load(f)
    f.close()
    return d


def src_dir():
    return os.path.abspath(os.path.dirname(__file__)) + "/"


def path_expand_cmd(path):
    lst = path.split('/')
    s = lst[0]
    if s[:2] == '$(' and s[-1] == ')':
        cmd = s[2:-1].split(' ')
        lst[0] = subprocess.check_output(cmd).strip()
        path = '/'.join(lst)
    return path


def get_tooltip(dic):
    return dic.get('desc')


def get_tooltips(dic):
    return dic.get('descs', [])


def get_tooltip_obj(obj):
    if getattr(obj, 'GetToolTip', None):
        t = obj.GetToolTip()
        return t.GetTip() if t else None
    return None


def set_tooltip(obj, dic):
    set_tooltip_str(obj, get_tooltip(dic))


def set_tooltips(obj, dic):
    lst = get_tooltips(dic)
    if lst and getattr(obj, 'SetItemToolTip', None):
        for (ix, s) in enumerate(lst):
            obj.SetItemToolTip(ix, s)


def set_tooltip_str(obj, s):
    if s and getattr(obj, 'SetToolTipString', None):
        obj.SetToolTipString(s)


def static_box_sizer(parent, s, orient=wx.VERTICAL):
    sb = wx.StaticBox(parent, wx.ID_ANY, s)
    sb.Lower()
    return wx.StaticBoxSizer(sb, orient)


def wx_flag_get(flags):
    dic = {'top': wx.TOP, 'bottom': wx.BOTTOM, 'left': wx.LEFT, 'right': wx.RIGHT,
           'all': wx.ALL, 'expand': wx.EXPAND, 'fixed_minsize': wx.FIXED_MINSIZE,
           'center_v': wx.ALIGN_CENTER_VERTICAL, 'center_h': wx.ALIGN_CENTER_HORIZONTAL,
           'passwd': wx.TE_PASSWORD}
    lst = [dic.get(f) for f in flags if f in dic]
    return reduce(lambda a, b: a + b, [0] + lst)


def set_size_gdic(dlg, gdic={}):
    (w, h) = dlg.GetSize()
    if not gdic:
        gdic = getattr(dlg, 'gdic', {})
    nw = gdic.get('dialog_width', w)
    nh = gdic.get('dialog_height', h)
    if (w, h) != (nw, nh):
        dlg.SetSize((nw, nh))


def obj_refresh(obj):
    if type(obj) is CT.GenericTreeItem:
        while obj.GetParent():
            obj = obj.GetParent()
        tree = obj.GetData()
        tree.Refresh()


def enables_set(obj, k, en):
    if hasattr(obj, 'enables_proxy'):
        (obj, k) = obj.enables_proxy
    d = attr_getset(obj, 'enabLes', {})
    d[k] = en
    d['last_key'] = k
    if hasattr(obj, 'Enable'):
        obj.Enable(all(d.values()))
    obj_refresh(obj)
    if isinstance(obj, wx.HyperlinkCtrl):
        if not hasattr(obj, 'coLor'):
            obj.coLor = {True: obj.GetNormalColour(), False: '#808080'}
        c = obj.coLor.get(obj.IsEnabled())
        obj.SetNormalColour(c)
        obj.SetVisitedColour(c)


def append_tc_limit(tc, s, rm_chars=0):
    if rm_chars > 0:
        tc.Remove(0, rm_chars)
    tc.AppendText(s)


def get_proc_children(proc, r=False):
    try:
        return proc.get_children(recursive=r)
    except AttributeError:
        return proc.children(recursive=r)


def terminate_children(proc, sigint=False):
    for child in get_proc_children(proc):
        terminate_children(child, sigint)
        terminate(child, sigint)


def terminate(proc, sigint=False):
    if sigint:
        proc.send_signal(signal.SIGINT)
    else:
        proc.terminate()


def cut_esc(s):
    while True:
        i = s.find(chr(27))
        if i < 0:
            break
        j = s.find('m', i)
        if j < 0:
            break
        s = s[:i] + s[j + 1:]
    return s


def proc_wait_thread(ev, proc, obj):
    proc.wait()
    wx.CallAfter(enables_set, obj, 'proc_wait', True)
    th_end((None, ev))


def th_start(target, kwargs={}):
    ev = threading.Event()
    kwargs['ev'] = ev
    th = threading.Thread(target=target, kwargs=kwargs)
    th.daemon = True
    th.start()
    return (th, ev)


def th_end((th, ev)):
    if not th:
        th = threading.current_thread()
        threading.Timer(1.0, th_end, ((th, ev),)).start()
        return
    ev.set()
    th.join()


def que_clear(que):
    with que.mutex:
        que.queue.clear()


def set_path(tc, v):
    tc.SetValue(v)
    tc.SetInsertionPointEnd()


def set_val(obj, v):
    func = getattr(obj, 'SetValue', getattr(obj, 'Check', None))
    if func:
        func(v)
        obj_refresh(obj)
    if type(obj) is wx.ToggleButton:
        button_color_change(obj)


def file_dialog(parent, tc, path_inf_dic={}):
    path = tc.GetValue()
    path = get_top(path.split(','), path)
    (dn, fn) = os.path.split(path)
    path_type = path_inf_dic.get('path_type')
    if path_type == 'dir':
        fns = path_inf_dic.get('filenames')
        if type(fns) is str and fns[-5:] == '.yaml':
            fns = load_yaml(fns)
            if type(fns) is not list:
                fns = None
            path_inf_dic['filenames'] = fns
        dlg = wx.DirDialog(parent, defaultPath=path)
    else:
        st_dic = {'save': wx.FD_SAVE, 'multi': wx.FD_MULTIPLE}
        dlg = wx.FileDialog(parent, defaultDir=dn, defaultFile=fn,
                            style=st_dic.get(path_type, wx.FD_DEFAULT_STYLE))
    ret = show_modal(dlg)
    if ret == wx.ID_OK:
        path = ','.join(dlg.GetPaths()) if path_type == 'multi' else dlg.GetPath()
        if path_type == 'dir' and fns:
            path = ','.join([path + '/' + fn for fn in fns])
        set_path(tc, path)
    dlg.Destroy()
    return ret


def button_color_change(btn, v=None):
    if v is None and type(btn) is wx.ToggleButton:
        v = btn.GetValue()
    key = (v, btn.IsEnabled())
    dic = {(True, True): ('#F9F9F8', '#8B8BB9'), (True, False): ('#F9F9F8', '#E0E0F0')}
    (fcol, bcol) = dic.get(key, (wx.NullColour, wx.NullColour))
    btn.SetForegroundColour(fcol)
    btn.SetBackgroundColour(bcol)


def fix_link_color(obj):
    t = type(obj)
    if t is CT.GenericTreeItem or t is CT.CustomTreeCtrl:
        obj.SetHyperTextVisitedColour(obj.GetHyperTextNewColour())
    elif t is wx.HyperlinkCtrl:
        obj.SetVisitedColour(obj.GetNormalColour())


def OnButtonColorHdr(event):
    btn = event.GetEventObject()
    dic = {wx.EVT_TOGGLEBUTTON.typeId: None,
           wx.EVT_LEFT_DOWN.typeId: True,
           wx.EVT_LEFT_UP.typeId: False}
    v = dic.get(event.GetEventType(), '?')
    if v != '?':
        button_color_change(btn, v)
    event.Skip()


btn_null_bgcol = None


def is_btn_null_bgcol(btn):
    global btn_null_bgcol
    bak = btn.GetBackgroundColour()
    if btn_null_bgcol is None:
        btn.SetBackgroundColour(wx.NullColour)
        btn_null_bgcol = btn.GetBackgroundColour()
        if bak != btn_null_bgcol:
            btn.SetBackgroundColour(bak)
    return bak == btn_null_bgcol


def button_color_hdr_setup(btn):
    hdr = OnButtonColorHdr
    if type(btn) is wx.ToggleButton:
        btn.Bind(wx.EVT_TOGGLEBUTTON, hdr)
    elif type(btn) is wx.Button and is_btn_null_bgcol(btn):
        btn.Bind(wx.EVT_LEFT_DOWN, hdr)
        btn.Bind(wx.EVT_LEFT_UP, hdr)


def buttons_color_hdr_setup(frm_obj):
    key = 'button_'
    btns = [getattr(frm_obj, nm) for nm in dir(frm_obj) if nm[:len(key)] == key]
    for btn in btns:
        button_color_hdr_setup(btn)


def show_modal(dlg):
    buttons_color_hdr_setup(dlg)
    return dlg.ShowModal()


def sizer_wrap(add_objs, orient=wx.VERTICAL, prop=0, flag=0, border=0, parent=None):
    szr = wx.BoxSizer(orient)
    for obj in add_objs:
        szr.Add(obj, prop, flag, border)
    if parent:
        parent.SetSizer(szr)
    return szr


def attr_getset(obj, name, def_ret):
    if not hasattr(obj, name):
        setattr(obj, name, def_ret)
    return getattr(obj, name)


def get_top(lst, def_ret=None):
    return lst[0] if len(lst) > 0 else def_ret


def enables_get(obj, k, def_ret=None):
    return attr_getset(obj, 'enabLes', {}).get(k, def_ret)


def enables_get_last(obj):
    k = enables_get(obj, 'last_key')
    return k, enables_get(obj, k)


# dic_list util (push, pop, get)
def dic_list_push(dic, key, v):
    dic_getset(dic, key, []).append(v)


def dic_list_pop(dic, key):
    dic.get(key, [None]).pop()


def dic_list_get(dic, key, def_ret=None):
    return dic.get(key, [def_ret])[-1]


def bak_stk_push(dic, key):
    if key in dic:
        k = key + '_bak_str'
        dic_getset(dic, k, []).append(dic.get(key))


def bak_stk_pop(dic, key):
    k = key + '_bak_str'
    stk = dic.get(k, [])
    if len(stk) > 0:
        dic[key] = stk.pop()
    else:
        del dic[key]


def bak_stk_set(dic, key, v):
    # bak_str_push(dic, key)
    dic[key] = v


def lst_append_once(lst, v):
    exist = v in lst
    if not exist:
        lst.append(v)
    return exist


def lst_remove_once(lst, v):
    exist = v in lst
    if exist:
        lst.remove(v)
    return exist


def msg_path_to_obj_attr(msg, path):
    lst = path.split('.')
    obj = msg
    for attr in lst[:-1]:
        obj = getattr(obj, attr, None)
    return obj, lst[-1]


def str_to_rosval(s, type_str, def_ret=None):
    cvt_dic = {
        'int8': int, 'int16': int, 'int32': int,
        'uint8': int, 'uint16': int, 'uint32': int,
        'int64': long, 'uint64': long,
        'float32': float, 'float64': float,
    }
    t = cvt_dic.get(type_str)
    s = s.replace(',', '.') if t is float and type(s) is str else s
    return t(s) if t else def_ret


def adjust_num_str(s):
    if '.' in s:
        while s[-1] == '0':
            s = s[:-1]
        if s[-1] == '.':
            s = s[:-1]
    return s


def dic_getset(dic, key, def_ret):
    if key not in dic:
        dic[key] = def_ret
    return dic.get(key)


def eval_if_str(self, v):
    return eval(v) if type(v) is str else v


def dic_eval_if_str(self, dic, key, def_ret=None):
    return eval_if_str(self, dic.get(key, def_ret))


def gdic_dialog_name_get(gdic):
    dlg_type = dic_list_get(gdic, 'dialog_type', 'config')
    return gdic.get(dlg_type + '_dialog', gdic.get('dialog', 'MyDialogParam'))


def gdic_dialog_type_chk(gdic, name):
    dlg_type = dic_list_get(gdic, 'dialog_type', 'config')

    tail = '_dialog_only'
    lst = [(k, k[:-len(tail)]) for k in gdic.keys() if k[-len(tail):] == tail]
    only_chk = next((False for (k, type) in lst if type != dlg_type and name in gdic.get(k, [])), True)

    tail = '_dialog_allow'
    lst = [(k, k[:-len(tail)]) for k in gdic.keys() if k[-len(tail):] == tail]
    allow_chk = next((False for (k, type) in lst if type == dlg_type and name not in gdic.get(k, [])), True)

    return only_chk and allow_chk


def str_to_float(s):
    return float(s.replace(',', '.'))


class StrValObj:
    def __init__(self, s, v):
        self.s = s
        self.v = v

    def GetValue(self):
        return self.v

    def SetValue(self, v):
        self.v = v


class MyApp(wx.App):
    def OnInit(self):
        wx.InitAllImageHandlers()
        frame_1 = MyFrame(None, wx.ID_ANY, "")
        self.SetTopWindow(frame_1)
        # buttons_color_hdr_setup(frame_1)
        frame_1.Show()
        return 1


class MyDialogROSbagRecord(aslan_gui.MyDialogROSbagRecord):
    def __init__(self, *args, **kwds):
        self.cmd_dic = kwds.pop('cmd_dic')
        aslan_gui.MyDialogROSbagRecord.__init__(self, *args, **kwds)
        self.cbs = []
        self.refresh()
        self.parent = self.GetParent()
        self.cmd_dic[self.button_start] = ('rosbag record', None)
        self.toggles = [self.button_start, self.button_stop]

    def OnRef(self, event):
        tc = self.text_ctrl
        file_dialog(self, tc, {'path_type': 'save'})

    def OnStart(self, event):
        key_obj = self.button_start
        path = self.text_ctrl.GetValue()
        if path == '':
            print('path=""')
            return
        topic_opt = []
        if self.cbs[0].GetValue():  # 'All'
            topic_opt = ['-a']
        else:
            for obj in self.cbs:
                if obj.GetValue():
                    topic_opt += [obj.GetLabel()]
        if not topic_opt:
            print('topic=[]')
            return
        args = topic_opt + ['-O', path]

        split_arg = ['--split'] if self.checkbox_split.GetValue() else []
        size_arg = self.size_arg_get()
        if split_arg and not size_arg:
            wx.MessageBox('size is required, with split')
            return
        args += split_arg + size_arg

        (cmd, proc) = self.cmd_dic[key_obj]
        proc = self.parent.launch_kill(True, cmd, proc, add_args=args, obj=key_obj, kill_children=True)
        self.cmd_dic[key_obj] = (cmd, proc)
        self.parent.toggle_enables(self.toggles)

    def OnStop(self, event):
        key_obj = self.button_start
        (cmd, proc) = self.cmd_dic[key_obj]
        proc = self.parent.launch_kill(False, cmd, proc, sigint=True, obj=key_obj, kill_children=True)
        self.cmd_dic[key_obj] = (cmd, proc)
        self.parent.toggle_enables(self.toggles)
        self.Hide()

    def OnRefresh(self, event):
        self.refresh()

    def refresh(self):
        lst = ['all'] + subprocess.check_output(['rostopic', 'list']).strip().split('\n')
        panel = self.panel_1
        szr = self.sizer_topic
        for obj in self.cbs:
            szr.Remove(obj)
            obj.Destroy()
        self.cbs = []
        for topic in lst:
            obj = wx.CheckBox(panel, wx.ID_ANY, topic)
            bdr = 4 if topic == 'All' else 4 * 4
            szr.Add(obj, 0, wx.LEFT, bdr)
            self.cbs.append(obj)
        szr.Layout()
        panel.SetVirtualSize(szr.GetMinSize())

    def show(self):
        self.Show()
        self.update_filename()

    def update_filename(self):
        tc = self.text_ctrl
        path = tc.GetValue()
        (dn, fn) = os.path.split(path)
        now = datetime.datetime.now()
        fn = 'aslan-%04d%02d%02d%02d%02d%02d' % (
            now.year, now.month, now.day, now.hour, now.minute, now.second)
        path = os.path.join(dn, fn)
        set_path(tc, path)

    def size_arg_get(self):
        tc = self.text_ctrl_size
        s = tc.GetValue()
        mb = 0
        try:
            mb = str_to_float(s)
        except ValueError:
            mb = 0
        if mb <= 0:
            tc.SetValue('')
        return ['--size=' + str(int(mb * 1024 * 1024))] if mb > 0 else []


class ParamPanel(wx.Panel):
    def __init__(self, *args, **kwds):
        self.frame = kwds.pop('frame')
        self.pdic = kwds.pop('pdic')
        self.gdic = kwds.pop('gdic')
        self.prm = kwds.pop('prm')
        wx.Panel.__init__(self, *args, **kwds)

        self.gdic['param_panel'] = self

        obj = self.frame.cfg_prm_to_obj({'pdic': self.pdic, 'gdic': self.gdic, 'param': self.prm})
        (_, _, proc) = self.frame.obj_to_cmd_dic_cmd_proc(obj)

        hszr = None
        self.vps = []
        self.tmp_msg = None
        szr = wx.BoxSizer(wx.VERTICAL)

        topic_szrs = (None, None)

        vars = self.prm.get('vars')
        if self.gdic.get('show_order'):
            var_lst = lambda name, vars: [var for var in vars if var.get('name') == name]
            vars = reduce(lambda lst, name: lst + var_lst(name, vars), self.gdic.get('show_order'), [])

        for var in vars:
            name = var.get('name')

            if not gdic_dialog_type_chk(self.gdic, name):
                continue

            gdic_v = self.get_gdic_v_and_chk_enable(name)
            if gdic_v is None:
                continue

            bak_stk_push(gdic_v, 'func')
            if gdic_v.get('func'):
                continue

            v = self.pdic.get(name, var.get('v'))

            vp = VarPanel(self, var=var, v=v, update=self.update)
            vp.setup_tooltip()
            self.vps.append(vp)

            gdic_v['var'] = vp
            gdic_v['func'] = vp.get_v
            prop = gdic_v.get('prop', 0)
            border = gdic_v.get('border', 0)
            flag = wx_flag_get(gdic_v.get('flags', []))

            do_category = 'no_category' not in gdic_v.get('flags', [])
            if do_category and self.in_msg(var):
                bak = (szr, hszr)
                (szr, hszr) = topic_szrs
                if szr is None:
                    szr = static_box_sizer(self, 'topic : ' + self.prm.get('topic'))
                    bak[0].Add(szr, 0, wx.EXPAND | wx.ALL, 4)
            targ_szr = szr
            if vp.is_nl():
                hszr = None if hszr else hszr
                flag |= wx.EXPAND
            else:
                if hszr is None:
                    hszr = wx.BoxSizer(wx.HORIZONTAL)
                    szr.Add(hszr, 0, wx.EXPAND)
                flag |= wx.ALIGN_CENTER_VERTICAL
                targ_szr = hszr

            if do_category and 'rosparam' in var:
                rp_szr = static_box_sizer(self, var.get('rosparam'))
                targ_szr.Add(rp_szr, 0, wx.EXPAND | wx.ALL, 4)
                targ_szr = rp_szr

            user_category = gdic_v.get('user_category')
            if user_category is not None and hszr:
                user_szr = static_box_sizer(self, user_category, orient=wx.HORIZONTAL)
                (flgs, bdr) = gdic_v.get('user_category_add', [[], 0])
                targ_szr.Add(user_szr, 0, wx_flag_get(flgs), bdr)
                targ_szr = hszr = user_szr

            targ_szr.Add(vp, prop, flag, border)

            if 'nl' in gdic_v.get('flags', []):
                hszr = None

            if do_category and self.in_msg(var):
                topic_szrs = (szr, hszr)
                (szr, hszr) = bak

            if 'hline' in gdic_v.get('flags', []) and hszr is None:
                szr.Add(wx.StaticLine(self, wx.ID_ANY), 0, wx.EXPAND | wx.TOP | wx.BOTTOM, 4)

            if not self.in_msg(var) and var.get('rosparam'):
                k = 'ext_toggle_enables'
                self.gdic[k] = self.gdic.get(k, []) + [vp]
                enables_set(vp, 'toggle', proc is None)

            if 'disable' in gdic_v.get('flags', []):
                vp.Enable(False)
            if 'hide' in gdic_v.get('flags', []):
                vp.Hide()

        self.SetSizer(szr)
        if 'no_init_update' not in self.prm.get('flags', []):
            self.update()

    def get_gdic_v_and_chk_enable(self, var_name):
        gdic_v = dic_getset(self.gdic, var_name, {})
        if 'panel' in gdic_v and dic_eval_if_str(self.frame, gdic_v, 'panel') != self.GetParent():
            return None
        return gdic_v

    def update(self, var=None):
        update_func = self.gdic.get('update_func')
        if update_func:
            self.gdic['update_func_arg_var'] = var
            update_func(self.pdic, self.gdic, self.prm)

    def detach_func(self):
        for var in self.prm.get('vars'):
            name = var.get('name')

            if not gdic_dialog_type_chk(self.gdic, name):
                continue

            gdic_v = self.get_gdic_v_and_chk_enable(name)
            if gdic_v is None:
                continue

            if 'func' in gdic_v:
                bak_stk_pop(gdic_v, 'func')

            vp = gdic_v.get('var')
            lst_remove_once(self.gdic.get('ext_toggle_enables', []), vp)

    def in_msg(self, var):
        if 'topic' not in self.prm or 'msg' not in self.prm:
            return False
        if self.tmp_msg is None:
            klass_msg = globals().get(self.prm.get('msg'))
            if klass_msg is None:
                return False
            self.tmp_msg = klass_msg()
        (obj, attr) = msg_path_to_obj_attr(self.tmp_msg, var.get('name'))
        return obj and attr in obj.__slots__


class VarPanel(wx.Panel):
    def __init__(self, *args, **kwds):
        self.var = kwds.pop('var')
        v = kwds.pop('v')
        self.update = kwds.pop('update')
        wx.Panel.__init__(self, *args, **kwds)

        self.min = self.var.get('min')
        self.max = self.var.get('max')
        self.has_slider = self.min is not None and self.max is not None
        self.lb = None

        label = self.var.get('label', '')
        self.kind = self.var.get('kind')
        if self.kind == 'radio_box':
            choices = self.var.get('choices', [])
            style = wx.RA_SPECIFY_COLS if self.var.get('choices_style') == 'h' else wx.RA_SPECIFY_ROWS
            self.obj = wx.RadioBox(self, wx.ID_ANY, label, choices=choices, majorDimension=0, style=style)
            self.choices_sel_set(v)
            self.Bind(wx.EVT_RADIOBOX, self.OnUpdate, self.obj)
            return
        if self.kind == 'menu':
            choices = self.var.get('choices', [])
            self.obj = wx.Choice(self, wx.ID_ANY, choices=choices)
            self.choices_sel_set(v)
            self.Bind(wx.EVT_CHOICE, self.OnUpdate, self.obj)
            if label:
                self.lb = wx.StaticText(self, wx.ID_ANY, label)
                flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
                sizer_wrap((self.lb, self.obj), wx.HORIZONTAL, 0, flag, 4, self)
            return
        if self.kind == 'checkbox':
            self.obj = wx.CheckBox(self, wx.ID_ANY, label)
            self.obj.SetValue(v)
            self.Bind(wx.EVT_CHECKBOX, self.OnUpdate, self.obj)
            return
        if self.kind == 'checkboxes':
            item_n = dic_eval_if_str(self, self.var, 'item_n', 1)
            self.obj = Checkboxes(self, item_n, label)
            self.obj.set(v)
            for box in self.obj.boxes:
                self.obj.Bind(wx.EVT_CHECKBOX, self.OnUpdate, box)
            return
        if self.kind == 'toggle_button':
            self.obj = wx.ToggleButton(self, wx.ID_ANY, label)
            set_val(self.obj, v)
            self.Bind(wx.EVT_TOGGLEBUTTON, self.OnUpdate, self.obj)
            button_color_hdr_setup(self.obj)
            return
        if self.kind == 'hide':
            self.Hide()
            return
        if self.kind == 'topic':
            topic_type = self.var.get('topic_type')
            topics = self._get_topics_by_type(topic_type)
            self.obj = wx.ComboBox(self, id=wx.ID_ANY, value=v, choices=topics, style=wx.CB_DROPDOWN, size=(130, -1))
            self.lb = wx.StaticText(self, wx.ID_ANY, label)
            flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
            sizer_wrap((self.lb, self.obj), wx.HORIZONTAL, 0, flag, 4, self)
            return

        szr = wx.BoxSizer(wx.HORIZONTAL)

        self.lb = wx.StaticText(self, wx.ID_ANY, label)
        flag = wx.LEFT | wx.ALIGN_CENTER_VERTICAL
        szr.Add(self.lb, 0, flag, 4)

        if self.kind == 'path':
            v = str(v)
            v = path_expand_cmd(v)
            v = os.path.expandvars(os.path.expanduser(v))

        style = wx.TE_PROCESS_ENTER + wx_flag_get(self.var.get('str_flags', []))

        self.tc = wx.TextCtrl(self, wx.ID_ANY, str(v), style=style, size=(130, -1))
        self.Bind(wx.EVT_TEXT_ENTER, self.OnUpdate, self.tc)

        if self.kind in ('num', None):
            if self.has_slider:
                self.w = self.max - self.min
                vlst = [v, self.min, self.max, self.var['v']]
                self.is_float = len([v_ for v_ in vlst if type(v_) is not int]) > 0
                self.int_max = 1000 if self.is_float else self.max
                self.int_min = 0 if self.is_float else self.min

                self.slider = wx.Slider(self, wx.ID_ANY, self.get_int_v(), self.int_min, self.int_max)
                self.Bind(wx.EVT_COMMAND_SCROLL, self.OnScroll, self.slider)
                self.slider.SetMinSize((82, 27))
                szr.Add(self.slider, 1, wx.LEFT | wx.RIGHT | wx.ALIGN_CENTER_VERTICAL, 4)
            else:
                self.is_float = type(self.var['v']) is not int
                self.tc.SetMinSize((40, 27))

        flag = wx.ALIGN_CENTER_VERTICAL
        prop = 1 if self.kind == 'path' or self.kind == 'str' else 0
        szr.Add(self.tc, prop, flag, 4)

        if self.kind == 'path':
            self.ref = wx.Button(self, wx.ID_ANY, 'File')
            self.Bind(wx.EVT_BUTTON, self.OnRef, self.ref)
            button_color_hdr_setup(self.ref)
            self.ref.SetMinSize((40, 29))
            szr.Add(self.ref, 0, flag, 4)

        if self.has_slider or self.kind == 'num':
            vszr = wx.BoxSizer(wx.VERTICAL)
            szr.Add(vszr, 0, wx.ALIGN_CENTER_VERTICAL)

        self.SetSizer(szr)

    def _get_topics_by_type(self, message_type):
        # get list of current available topics:
        ros_topics = rospy.get_published_topics()
        matched_topics = list(filter(lambda x: x[1] == message_type, ros_topics))
        topic_names = [x[0] for x in matched_topics]
        topic_names.sort()
        return topic_names

    def setup_tooltip(self):
        if get_tooltips(self.var):
            set_tooltips(self.obj, self.var)
        if get_tooltip(self.var):
            obj = self.lb if self.lb else (self if self.kind == 'radio_box' else self.obj)
            set_tooltip(obj, self.var)

    def get_v(self):
        if self.kind in ['radio_box', 'menu']:
            return self.choices_sel_get()
        if self.kind in ['checkbox', 'toggle_button', 'topic']:
            return self.obj.GetValue()
        if self.kind == 'checkboxes':
            return self.obj.get()
        if self.kind == 'hide':
            return self.var.get('v')
        if self.kind in ['path', 'str']:
            return str(self.tc.GetValue())

        if not self.has_slider and self.tc.GetValue() == '':
            return ''
        return self.get_tc_v()

    def get_tc_v(self):
        s = self.tc.GetValue()
        v = str_to_float(s) if self.is_float else int(s)
        if self.has_slider:
            v = self.min if v < self.min else v
            v = self.max if v > self.max else v
        self.tc.SetValue(adjust_num_str(str(v)))
        return v

    def get_int_v(self):
        v = self.get_tc_v()
        if self.is_float:
            v = int(self.int_max * (v - self.min) / self.w if self.w != 0 else 0)
        return v

    def OnScroll(self, event):
        iv = self.slider.GetValue()
        s = str(iv)
        if self.is_float:
            v = self.min + float(self.w) * iv / self.int_max
            s = str(Decimal(v).quantize(Decimal(str(self.get_step()))))
        self.tc.SetValue(s)
        self.update(self.var)

    def get_step(self):
        step = self.var.get('step')
        return step if step else 0.01 if self.is_float else 1

    def OnUpdate(self, event):
        if self.has_slider:
            self.slider.SetValue(self.get_int_v())
        self.update(self.var)

    def OnRef(self, event):
        if file_dialog(self, self.tc, self.var) == wx.ID_OK:
            self.update(self.var)

    def choices_sel_get(self):
        return self.obj.GetStringSelection() if self.var.get('choices_type') == 'str' else self.obj.GetSelection()

    def choices_sel_set(self, v):
        if self.var.get('choices_type') == 'str':
            self.obj.SetStringSelection(v)
        else:
            self.obj.SetSelection(v)

    def is_nl(self):
        return self.has_slider or self.kind in ['path']


class MyDialogParam(aslan_gui.MyDialogParam):
    def __init__(self, *args, **kwds):
        pdic = kwds.pop('pdic')
        self.pdic_bak = pdic.copy()
        gdic = kwds.pop('gdic')
        prm = kwds.pop('prm')
        aslan_gui.MyDialogParam.__init__(self, *args, **kwds)
        set_size_gdic(self, gdic)

        self.Bind(wx.EVT_CLOSE, self.OnClose)

        ok_lb_key = 'open_dialog_ok_label'
        if dic_list_get(gdic, 'dialog_type', 'config') == 'open' and ok_lb_key in gdic:
            self.button_1.SetLabel(gdic.get(ok_lb_key))

        parent = self.panel_v
        frame = self.GetParent()
        self.panel = ParamPanel(parent, frame=frame, pdic=pdic, gdic=gdic, prm=prm)
        szr = sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

        self.SetTitle(prm.get('name', ''))
        (w, h) = self.GetSize()
        (w2, _) = szr.GetMinSize()
        w2 += 20
        if w2 > w:
            self.SetSize((w2, h))

    def OnOk(self, event):
        self.panel.update()
        self.panel.detach_func()
        self.EndModal(0)

    def OnCancel(self, event):
        self.panel.pdic.update(self.pdic_bak)  # restore
        self.panel.detach_func()
        self.panel.update()
        self.EndModal(-1)

    def OnClose(self, event):
        self.OnCancel(event)


class BarLabel(wx.Panel):
    def __init__(self, parent, txt='', pos=wx.DefaultPosition, size=wx.DefaultSize, style=0, hv=wx.HORIZONTAL,
                 show_lb=True):
        wx.Panel.__init__(self, parent, wx.ID_ANY, pos, size)
        self.lb = wx.StaticText(self, wx.ID_ANY, '', style=style)
        self.txt = txt
        self.hv = hv
        self.dir = wx.SOUTH if hv == wx.HORIZONTAL else wx.EAST
        self.show_lb = show_lb
        self.prg = -1

        self.dflt_col1 = wx.Colour(250, 250, 250)
        self.dflt_col2 = wx.Colour(128, 128, 128)
        self.col1 = self.dflt_col1
        self.col2 = self.dflt_col2

        self.Bind(wx.EVT_PAINT, self.OnPaint)

    def set(self, prg):
        self.prg = prg
        if self.show_lb:
            self.lb.SetLabel(self.txt + str(prg) + '%' if prg >= 0 else '')
        self.Refresh()

    def set_col(self, col1, col2):
        self.col1 = col1 if col1 != wx.NullColour else self.dflt_col1
        self.col2 = col2 if col2 != wx.NullColour else self.dflt_col2

    def clear(self):
        self.set(-1)

    def OnPaint(self, event):
        dc = wx.PaintDC(self)
        (w, h) = self.GetSize()
        h = 15
        if self.IsEnabled():
            p = (w if self.hv == wx.HORIZONTAL else h) * self.prg / 100
            rect = wx.Rect(0, 0, p, h) if self.hv == wx.HORIZONTAL else wx.Rect(0, h - p, w, p)
            dc.GradientFillLinear(rect, self.col1, self.col2, self.dir)
            rect = wx.Rect(p, 0, w - p, h) if self.hv == wx.HORIZONTAL else wx.Rect(0, 0, w, h - p)
            dc.GradientFillLinear(rect, wx.Colour(200, 200, 200), wx.Colour(250, 250, 250), self.dir)
        else:
            rect = wx.Rect(0, 0, w, h)
            dc.GradientFillLinear(rect, wx.Colour(250, 250, 250), wx.Colour(250, 250, 250), self.dir)


class Checkboxes(wx.Panel):
    def __init__(self, parent, item_n, lb):
        wx.Panel.__init__(self, parent, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize)
        self.boxes = [wx.CheckBox(self, wx.ID_ANY, lb + str(i)) for i in range(item_n)]
        vsz = wx.BoxSizer(wx.VERTICAL)
        for j in range((item_n + 7) / 8):
            hsz = wx.BoxSizer(wx.HORIZONTAL)
            for i in range(8):
                idx = j * 8 + i
                if idx < len(self.boxes):
                    hsz.Add(self.boxes[idx], 0, wx.LEFT, 8)
            vsz.Add(hsz)
        self.SetSizer(vsz)
        vsz.Fit(self)

    def set(self, vs):
        vs = vs if vs else [True for box in self.boxes]
        for (box, v) in zip(self.boxes, vs):
            box.SetValue(v)

    def get(self):
        return [box.GetValue() for box in self.boxes]


class MyDialogNDTMapping(aslan_gui.MyDialogNDTMapping):
    def __init__(self, *args, **kwds):
        self.pdic = kwds.pop('pdic')
        self.pdic_bak = self.pdic.copy()
        self.gdic = kwds.pop('gdic')
        self.prm = kwds.pop('prm')
        aslan_gui.MyDialogNDTMapping.__init__(self, *args, **kwds)
        set_size_gdic(self)

        parent = self.panel_v
        frame = self.GetParent()
        self.panel = ParamPanel(parent, frame=frame, pdic=self.pdic, gdic=self.gdic, prm=self.prm)
        sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

        self.update_filename()
        self.klass_msg = ConfigNDTMappingOutput
        self.pub = rospy.Publisher('/config/ndt_mapping_output', self.klass_msg, queue_size=10)

    def update_filename(self):
        tc = self.text_ctrl_path
        path = tc.GetValue()
        (dn, fn) = os.path.split(path)
        now = datetime.datetime.now()
        fn = 'aslan-%02d%02d%02d.pcd' % (
            now.year % 100, now.month, now.day)
        path = os.path.join(dn, fn)
        set_path(tc, path)

    def OnRef(self, event):
        tc = self.text_ctrl_path
        file_dialog(self, tc, {'path_type': 'save'})

    def OnRadio(self, event):
        v = self.radio_btn_filter_resolution.GetValue()
        tc = self.text_ctrl_filter_resolution
        tc.Enable(v)

    def OnPcdOutput(self, event):
        tc = self.text_ctrl_filter_resolution
        v = tc.GetValue() if self.radio_btn_filter_resolution.GetValue() else '0.0'
        msg = self.klass_msg()
        msg.filename = self.text_ctrl_path.GetValue()
        msg.filter_res = str_to_float(v)
        self.pub.publish(msg)

    def OnOk(self, event):
        self.panel.detach_func()
        self.EndModal(0)


class MyDialogWaypointLoader(aslan_gui.MyDialogWaypointLoader):
    def __init__(self, *args, **kwds):
        self.pdic = kwds.pop('pdic')
        self.pdic_bak = self.pdic.copy()
        self.gdic = kwds.pop('gdic')
        self.prm = kwds.pop('prm')
        aslan_gui.MyDialogWaypointLoader.__init__(self, *args, **kwds)
        set_size_gdic(self)

        parent = self.panel_v
        frame = self.GetParent()
        self.panel = ParamPanel(parent, frame=frame, pdic=self.pdic, gdic=self.gdic, prm=self.prm)
        sizer_wrap((self.panel,), wx.VERTICAL, 1, wx.EXPAND, 0, parent)

        self.klass_msg = Bool
        self.pub = rospy.Publisher('/config/waypoint_loader_output', self.klass_msg, queue_size=10)

    def OnCsvOutput(self, event):
        msg = self.klass_msg()
        msg.data = True
        self.pub.publish(msg)

    def OnOk(self, event):
        self.panel.detach_func()
        self.EndModal(0)


if __name__ == "__main__":
    gettext.install("app")

    app = MyApp(0)
    app.MainLoop()
