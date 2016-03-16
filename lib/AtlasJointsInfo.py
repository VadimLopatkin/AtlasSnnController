from atlas_msgs.msg import AtlasState


class AtlasJointsInfo():
    def __init__(self):
        print "atlas joints' info"
        self._joints_info = [None]*27
        self._set_joint_info(AtlasState.back_lbz, -0.610865, 0.610865)
        self._set_joint_info(AtlasState.back_mby, -1.2, 1.28)
        self._set_joint_info(AtlasState.back_ubx, -0.790809, 0.790809)
        self._set_joint_info(AtlasState.l_arm_elx, 0, 2.35619)
        self._set_joint_info(AtlasState.l_arm_shx, -1.39626, 1.74533)
        self._set_joint_info(AtlasState.l_arm_usy, -1.9635, 1.9635)
        self._set_joint_info(AtlasState.l_arm_mwx, -0.436, 1.571)
        self._set_joint_info(AtlasState.l_arm_uwy, -1.571, 1.571)
        self._set_joint_info(AtlasState.l_arm_lax, -0.436, 0.436)
        self._set_joint_info(AtlasState.l_leg_lhy, -1.75, 0.524)
        self._set_joint_info(AtlasState.l_leg_mhx, -0.47, 0.495)
        self._set_joint_info(AtlasState.l_leg_uay, -0.698, 0.698)
        self._set_joint_info(AtlasState.l_leg_uhz, -0.32, 1.14)
        self._set_joint_info(AtlasState.l_leg_kny, 0, 2.45)
        self._set_joint_info(AtlasState.neck_ay, -0.610865238, 1.13446401)
        self._set_joint_info(AtlasState.r_arm_elx, -2.35619, 0)
        self._set_joint_info(AtlasState.r_arm_ely, 0, 3.14159)
        self._set_joint_info(AtlasState.r_arm_shx, -1.74533, 1.39626)
        self._set_joint_info(AtlasState.r_arm_usy, -1.9635, 1.9635)
        self._set_joint_info(AtlasState.r_arm_mwx, -1.571, 0.436)
        self._set_joint_info(AtlasState.r_arm_uwy, -1.571, 1.571)
        self._set_joint_info(AtlasState.r_leg_lax, -0.436, 0.436)
        self._set_joint_info(AtlasState.r_leg_lhy, -1.745, 0.524)
        self._set_joint_info(AtlasState.r_leg_mhx, -0.495, 0.495)
        self._set_joint_info(AtlasState.r_leg_uay, -0.698, 0.698)
        self._set_joint_info(AtlasState.r_leg_uhz, -1.14, 0.32)
        self._set_joint_info(AtlasState.r_leg_kny, 0, 2.45)



    def _set_joint_info(self, joint_id, lower, upper):
        self._joints_info[joint_id] = {}
        self._joints_info[joint_id]['lower'] = lower
        self._joints_info[joint_id]['upper'] = upper
