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

from atlas_msgs.msg import AtlasState


class AtlasJointsInfo:
    def __init__(self):
        self._joints_info = [None]*28
        # All values are taken from
        # https://bitbucket.org/osrf/drcsim/src
        # /583228ebd72677c36a393b3260f0c5f0bc7b0f87/atlas_description/urdf
        # /atlas.urdf?at=default&fileviewer=file-view-default - correspond to
        #  Atlas v1 model
        self._set_joint_info(AtlasState.back_lbz, -0.610865, 0.610865, 'back_lbz')
        self._set_joint_info(AtlasState.back_mby, -1.2, 1.28, 'back_mby')
        self._set_joint_info(AtlasState.back_ubx, -0.790809, 0.790809, 'back_ubx')

        self._set_joint_info(AtlasState.l_arm_elx, 0, 2.35619, 'l_arm_elx')
        self._set_joint_info(AtlasState.l_arm_ely, 0, 3.14159, 'l_arm_ely')
        self._set_joint_info(AtlasState.l_arm_shx, -1.39626, 1.74533, 'l_arm_shx')
        self._set_joint_info(AtlasState.l_arm_usy, -1.9635, 1.9635, 'l_arm_usy')
        self._set_joint_info(AtlasState.l_arm_mwx, -0.436, 1.571, 'l_arm_mwx')
        self._set_joint_info(AtlasState.l_arm_uwy, -1.571, 1.571, 'l_arm_uwy')

        self._set_joint_info(AtlasState.l_leg_lax, -0.436, 0.436, 'l_leg_lax')
        self._set_joint_info(AtlasState.l_leg_lhy, -1.75, 0.524, 'l_leg_lhy')
        self._set_joint_info(AtlasState.l_leg_mhx, -0.47, 0.495, 'l_leg_mhx')
        self._set_joint_info(AtlasState.l_leg_uay, -0.698, 0.698, 'l_leg_uay')
        self._set_joint_info(AtlasState.l_leg_uhz, -0.32, 1.14,'l_leg_uhz')
        self._set_joint_info(AtlasState.l_leg_kny, 0, 2.45, 'l_leg_kny')

        self._set_joint_info(AtlasState.neck_ay, -0.610865238, 1.13446401, 'neck_ay')

        self._set_joint_info(AtlasState.r_arm_elx, -2.35619, 0, 'r_arm_elx')
        self._set_joint_info(AtlasState.r_arm_ely, 0, 3.14159, 'r_arm_ely')
        self._set_joint_info(AtlasState.r_arm_shx, -1.74533, 1.39626, 'r_arm_shx')
        self._set_joint_info(AtlasState.r_arm_usy, -1.9635, 1.9635, 'r_arm_usy')
        self._set_joint_info(AtlasState.r_arm_mwx, -1.571, 0.436, 'r_arm_mwx')
        self._set_joint_info(AtlasState.r_arm_uwy, -1.571, 1.571, 'r_arm_uwy')

        self._set_joint_info(AtlasState.r_leg_lax, -0.436, 0.436, 'r_leg_lax')
        self._set_joint_info(AtlasState.r_leg_lhy, -1.745, 0.524, 'r_leg_lhy')
        self._set_joint_info(AtlasState.r_leg_mhx, -0.495, 0.495, 'r_leg_mhx')
        self._set_joint_info(AtlasState.r_leg_uay, -0.698, 0.698, 'r_leg_uay')
        self._set_joint_info(AtlasState.r_leg_uhz, -1.14, 0.32, 'r_leg_uhz')
        self._set_joint_info(AtlasState.r_leg_kny, 0, 2.45, 'r_leg_kny')



    def _set_joint_info(self, joint_id, lower, upper, name):
        self._joints_info[joint_id] = {}
        self._joints_info[joint_id]['lower'] = lower
        self._joints_info[joint_id]['upper'] = upper
        self._joints_info[joint_id]['name'] = name

    def get_max_value_for_joint(self, joint_id):
        return self._joints_info[joint_id]['upper']

    def get_min_value_for_joint(self, joint_id):
        return self._joints_info[joint_id]['lower']

    def get_full_name_for_joint(self, joint_id):
        return 'atlas::' + self._joints_info[joint_id]['name']

    def get_short_name_for_joint(self, joint_id):
        return self._joints_info[joint_id]['name']
