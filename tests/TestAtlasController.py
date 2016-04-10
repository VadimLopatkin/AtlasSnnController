# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.

import time
import unittest
import numpy as np


import rospy
from atlas_msgs.msg import AtlasState

from lib.AtlasController import AtlasController


class TestAtlasController(unittest.TestCase):

    ATLAS_STATE_TOPIC = "atlas/atlas_state"
    NODE_NAME = "fake_atlas_node"

    def test_init(self):
        atlas_controller = AtlasController()
        # self._init_fake_atlas_publisher(True)
        # self._send_fake_atlas_states(10)

    def test_atlas_controller_receives_atlas_states(self):
        atlas_controller = AtlasController()
        time.sleep(1)
        self._init_fake_atlas_publisher()
        time.sleep(1)
        self._send_fake_atlas_states(0.001)
        time.sleep(600)
        self.assertEqual(self._get_fake_atlas_state_message(1).header,
                         atlas_controller.get_current_state().header)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).position,
                              atlas_controller.get_current_state().position)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).velocity,
                              atlas_controller.get_current_state().velocity)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).effort,
                              atlas_controller.get_current_state().effort)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).kp_position,
                              atlas_controller.get_current_state().kp_position)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).kp_velocity,
                              atlas_controller.get_current_state().kp_velocity)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).ki_position,
                              atlas_controller.get_current_state().ki_position)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).kd_position,
                              atlas_controller.get_current_state().kd_position)
        self.assertItemsEqual(self._get_fake_atlas_state_message(1).i_effort_min,
                              atlas_controller.get_current_state().i_effort_min)
        self.assertItemsEqual(self._get_fake_atlas_state_message(
                1).i_effort_max, atlas_controller.get_current_state().i_effort_max)
        # self.assertItemsEqual(self._get_fake_atlas_state_message(1).k_effort,
        #                      atlas_controller.get_state().k_effort)
        # k_effort will not work (apparently because it's a flag)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).orientation, atlas_controller.get_current_state().orientation)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).angular_velocity, atlas_controller.get_current_state().angular_velocity)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).linear_acceleration, atlas_controller.get_current_state().linear_acceleration)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).l_foot, atlas_controller.get_current_state().l_foot)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).r_foot, atlas_controller.get_current_state().r_foot)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).l_hand, atlas_controller.get_current_state().l_hand)
        self.assertEqual(self._get_fake_atlas_state_message(
                1).r_hand, atlas_controller.get_current_state().r_hand)

    def test_input_layer_of_snn_is_normalized(self):
        atlas_controller = AtlasController()
        time.sleep(1)
        self._init_fake_atlas_publisher()
        time.sleep(1)
        self._send_fake_atlas_states(0.001)
        time.sleep(1)
        network = atlas_controller.get_network()
        input_layer_values = network.get_input_layer_values()
        self.assertLessEqual(np.amax(input_layer_values),1)
        self.assertGreaterEqual(np.amin(input_layer_values),0)


    def _init_fake_atlas_publisher(self,create_node=False):
        self._fake_atlas_publisher = rospy.Publisher(self.ATLAS_STATE_TOPIC,
                                              AtlasState,
                              queue_size=10)
        if(create_node):
            rospy.init_node(self.NODE_NAME, anonymous=True)

    def _send_fake_atlas_states(self, sec):
        millisec = int(sec * 1000)
        fake_atlas_state_message = self._get_fake_atlas_state_message()
        for x in xrange(millisec):
            self._fake_atlas_publisher.publish(fake_atlas_state_message)
            time.sleep(0.001)

    def _get_fake_atlas_state_message(self,seq=0):
        message = AtlasState()
        message.header.seq = seq
        message.header.stamp.secs = 0
        message.header.stamp.nsecs = 0
        message.header.frame_id = ''
        message.position = [4.76053828606382e-05, 0.0009045443148352206, -4.077929406776093e-05, -0.6108808517456055, 0.00012987040099687874, 0.061346568167209625, -0.22740185260772705, 0.5164738893508911, -0.28053218126296997, -0.05775527283549309, -0.0009372962522320449, -0.06006138026714325, -0.23658907413482666, 0.5136617422103882, -0.2668575346469879, 0.06629158556461334, 0.2995331585407257, -1.302445888519287, 2.000369071960449, 0.49927830696105957, -1.0044869895864394e-06, -0.000988953048363328, 0.2995603680610657, 1.3025296926498413, 2.000384569168091, -0.4992787539958954, -1.508746777290071e-06, 0.0009888781933113933]
        message.velocity = [0.001283991732634604, 0.00026422215159982443, 0.0004413901478983462, 0.0045036138035357, 0.0012036490952596068, 0.002529401332139969, -0.0003825286985374987, 0.001856379210948944, -0.004040474537760019, -0.0017659817822277546, 0.0013796640560030937, 0.0026894686743617058, -0.0004156484210398048, 0.0007066466496326029, -0.0029232471715658903, -0.0015185577794909477, -0.0004534637264441699, -0.0006942410254850984, 0.00026704705669544637, -0.0008810595609247684, -0.0010271456558257341, 0.0005971139180473983, -0.00013518068590201437, -0.0006011173827573657, -0.00020782652427442372, -0.0003220417711418122, -1.379005243506981e-05, -0.0002727861574385315]
        message.effort = [0.2504699230194092, -4.582019805908203, 0.20552833378314972, 0.0003910064697265625, -0.33696550130844116, -9.744351387023926, -38.863948822021484, -41.74205780029297, 16.085081100463867, -0.30354368686676025, 4.348391056060791, 10.273491859436035, 7.0164570808410645, -14.038414001464844, 21.76513671875, -0.6578343510627747, 1.4133715629577637, 7.358339309692383, -1.0975255966186523, 2.187305450439453, -0.0005222359905019403, 0.4946136772632599, 1.329885721206665, -7.574656009674072, -1.155295968055725, -2.15950345993042, 0.0007825485081411898, -0.4945334196090698]
        message.kp_position = [20.0, 4000.0, 2000.0, 20.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 5.0, 100.0, 2000.0, 1000.0, 900.0, 300.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0, 2000.0, 1000.0, 200.0, 200.0, 50.0, 100.0]
        message.ki_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        message.kd_position = [0.10000000149011612, 2.0, 1.0, 1.0, 0.009999999776482582, 1.0, 10.0, 10.0, 2.0, 1.0, 0.009999999776482582, 1.0, 10.0, 10.0, 2.0, 1.0, 3.0, 20.0, 3.0, 3.0, 0.10000000149011612, 0.20000000298023224, 3.0, 20.0, 3.0, 3.0, 0.10000000149011612, 0.20000000298023224]
        message.kp_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        message.i_effort_min = [-0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0]
        message.i_effort_max = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        message.k_effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        message.orientation.x = -0.00182031484662
        message.orientation.y = -0.00429444695192
        message.orientation.z = 0.00525427843674
        message.orientation.w = 0.999975318064
        message.angular_velocity.x = -0.0016718171428
        message.angular_velocity.y = 0.000982969839022
        message.angular_velocity.z = 0.000105132383379
        message.linear_acceleration.x = -0.0518898647139
        message.linear_acceleration.y = -0.0773248032126
        message.linear_acceleration.z = 9.72280392841
        message.l_foot.force.x = 0.0
        message.l_foot.force.y = 0.0
        message.l_foot.force.z = 432.585730518
        message.l_foot.torque.x = 0.303514777044
        message.l_foot.torque.y = -16.0485430536
        message.l_foot.torque.z = 0.0
        message.r_foot.force.x = 0.0
        message.r_foot.force.y = 0.0
        message.r_foot.force.z = 425.33735304
        message.r_foot.torque.x = 0.657809593677
        message.r_foot.torque.y = -22.1425820828
        message.r_foot.torque.z = 0.0
        message.l_hand.force.x = 4.20361814656
        message.l_hand.force.y = 21.1838897553
        message.l_hand.force.z = -5.35801509868
        message.l_hand.torque.x = -0.494613420268
        message.l_hand.torque.y = 0.000605509676244
        message.l_hand.torque.z = -0.389345878614
        message.r_hand.force.x = 4.43649970354
        message.r_hand.force.y = -21.0467996508
        message.r_hand.force.z = -5.30684580501
        message.r_hand.torque.x = 0.494531666509
        message.r_hand.torque.y = -0.000249371694277
        message.r_hand.torque.z = 0.413483026318
        return message

if __name__ == '__main__':
    unittest.main()

