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

from __future__ import division

import rospy
import numpy as np
import actionlib

from lib.AtlasJointsInfo import AtlasJointsInfo
from lib.RmseSaver import RmseSaver
from atlas_msgs.msg import AtlasSimInterfaceCommand, AtlasSimInterfaceState, \
    WalkDemoAction, AtlasBehaviorStepData, WalkDemoGoal, \
    AtlasBehaviorStepParams, AtlasBehaviorStandParams, AtlasBehaviorManipulateParams
from std_msgs.msg import String, Header
from tf.transformations import quaternion_from_euler



class AtlasControllerTrainer:
    def __init__(self):
        self._rmse_saver = RmseSaver()
        self._robot_walking_initialized = False
        self._atlas_joints_info_provider = AtlasJointsInfo()
        self._current_behavior = AtlasSimInterfaceCommand.NONE
        self._sim_interface_subscriber = rospy.Subscriber(
                'atlas/atlas_sim_interface_state', AtlasSimInterfaceState,
                                            self._sim_int_cb)
        self._trainings_counter = 0
        self._calls_counter = 0
        self._rmse_queue = []
        self._network_trained = False
        self._sim_int_cb_counter = 0

    @classmethod
    def train_the_network(cls, controller):
        global INSTANCE
        INSTANCE.increase_calls_counter()
        if INSTANCE.get_current_behavior() == AtlasSimInterfaceCommand.WALK:
            print str(INSTANCE.get_trainings_counter()) + ": network is being " \
                                                          "" \
                                                          "trained"
            controller_output = controller.get_output()
            print "validating controller output"
            cls._validate_output(controller_output)
            expected_output = controller.get_current_state().position
            print "validating expected output"
            cls._validate_output(expected_output)
            absolute_deltas = cls._compute_absolute_deltas(controller_output,
                                                           expected_output)
            relative_deltas = cls._compute_relative_deltas(absolute_deltas,
                                                           expected_output)
            root_mean_square_error = cls._rmse(controller_output,
                                               expected_output)
            INSTANCE.save_rmse(root_mean_square_error)
            print "Max absolute error: " + str(
                np.amax(absolute_deltas)) + "\nMax relative " \
                                            "error: " + str(
                np.amax(relative_deltas)) + ".\nRMSE: " + str(
                    root_mean_square_error)
            if len(INSTANCE._rmse_queue) >= 100:
                INSTANCE._rmse_queue.pop(0)
                if np.average(INSTANCE._rmse_queue)<0.07:
                    print "Learning is finished, switching to walking mode"
                    INSTANCE._network_trained = True
                    INSTANCE.reset_robot_pose()
                    controller.set_learning_mode(False)
                    #TODO maybe we should let it in Stand mode set in
                    # reset_robot_pose
                    INSTANCE.set_control_mode("User")
                    INSTANCE.write_rmse_csv()
                    rospy.sleep(3)
                    return
            INSTANCE._rmse_queue.append(root_mean_square_error)
            # we need to train the network on the same input and output
            # several times
            if root_mean_square_error > 0.5:
                training_iterations = 100
                eta = 3.0
            elif root_mean_square_error > 0.35:
                training_iterations = 50
                eta = 3.0
            elif root_mean_square_error > 0.15:
                training_iterations = 50
                eta = 1.7
            elif root_mean_square_error > 0.06:
                training_iterations = 30
                eta = 1.4
            else:
                # we do not need to train the network if it's already good
                # enough
                training_iterations = 0
                eta = 3.0
            for i in xrange(training_iterations):
                cls._backpropagate_and_train(controller, eta)
            controller_output = controller.get_output()
            root_mean_square_error = cls._rmse(controller_output,
                                               expected_output)
            INSTANCE.save_rmse(root_mean_square_error)
            print "RMSE after training: " + str(root_mean_square_error)
            INSTANCE.increase_trainings_counter()
        else:
            print str(
                INSTANCE.get_calls_counter()) + ": network will not be trained"

    def get_current_behavior(self):
        return self._current_behavior

    def _sim_int_cb(self, msg):
        self._current_behavior = msg.current_behavior
        self._sim_int_cb_counter += 1
        if (msg.current_behavior != AtlasSimInterfaceCommand.WALK) and (
                self._network_trained == False) and (
                    self._sim_int_cb_counter>10000):
            self._sim_int_cb_counter = 0
            print "\nAtlas robot is standing and we engage walking"
            if not self._robot_walking_initialized:
                self._init_walking_client()
            self.reset_robot_pose()
            print "Atlas robot should start walking"
            self._send_walking_command()
            rospy.sleep(3.0)

    def increase_calls_counter(self):
        self._calls_counter += 1

    def increase_trainings_counter(self):
        self._trainings_counter += 1

    def get_calls_counter(self):
        return self._calls_counter

    def get_trainings_counter(self):
        return self._trainings_counter

    @classmethod
    def _validate_output(cls, output):
        for i in xrange(len(output)):
            value = output[i]
            if value > INSTANCE._atlas_joints_info_provider\
                    .get_max_value_for_joint(i):
                ratio = value / \
                        INSTANCE._atlas_joints_info_provider.get_max_value_for_joint(i)
                if(ratio > 1.0):
                    print "output value for joint " + str(i) + " exceeds it's " \
                                                           "maximum allowed " \
                                                           "value by " + str(
                        ratio)
            if value < INSTANCE._atlas_joints_info_provider\
                    .get_min_value_for_joint(i):
                ratio = INSTANCE._atlas_joints_info_provider\
                    .get_min_value_for_joint(i) / value
                if(ratio > 1.0):
                    print "output value for joint " + str(i) + " is lower than " \
                                                           "it's " \
                                                           "minimum allowed " \
                                                           "value by " + str(
                ratio)

    @classmethod
    def _compute_absolute_deltas(cls, controller_output, expected_output):
        expected_output_list = list(expected_output)
        result = []
        for i in xrange(len(controller_output)):
            delta = abs(expected_output_list[i] - controller_output[i])
            result.append(delta)
        return result

    @classmethod
    def _compute_relative_deltas(cls, absolute_deltas, expected_output):
        expected_output_list = list(expected_output)
        result = []
        for i in xrange(len(expected_output_list)):
            delta = absolute_deltas[i] / expected_output_list[i]
            result.append(delta)
        return result

    @classmethod
    def _rmse(cls, controller_output, expected_output):
        expected_output_list = list(expected_output)
        return np.sqrt(((np.asarray(expected_output_list) -
                         np.asarray(controller_output)) ** 2).mean())

    @classmethod
    def _backpropagate_and_train(cls, controller, eta):
        # print "\nentering _backpropagate_and_train"
        expected_output = controller.get_current_state().position
        controller_output_layer_activations = \
            controller.get_output_layer_activations()
        hidden_layer_firing_rates = controller.get_hidden_layer_firing_rates()
        hidden_layer_biases = controller.get_hidden_layer_biases()
        # print "len(expected_output) = " + str(len(expected_output))
        for i in xrange(len(expected_output)):
            expected_output_item = expected_output[i]
            # print "expected_output_item = " + str(expected_output_item)
            # print "controller output = " + str(controller.get_output()[i])
            expected_activation_value = (expected_output_item -
                INSTANCE._atlas_joints_info_provider.get_min_value_for_joint(i))/(
                INSTANCE._atlas_joints_info_provider.get_max_value_for_joint(
                        i)-INSTANCE._atlas_joints_info_provider
                .get_min_value_for_joint(i))
            # print "expected_activation_value = " + str(expected_activation_value)
            # print "controller_output_layer_activations[" + str(i) + "] = " + \
            #       str(
            #         controller_output_layer_activations[i])
            delta = (controller_output_layer_activations[i] -
                     expected_activation_value
                     )*controller_output_layer_activations[i]*(
                1-controller_output_layer_activations[i])
            # print "delta = " + str(delta)
            if abs(delta) < 0.01:
                if delta < 0:
                    delta = -0.01
                else:
                    delta = 0.01
            nabla_b = delta
            hidden_layer_weights = \
                controller.get_hidden_layer_weights_for_output_neuron(i)
            hidden_layer_weights_after_training = np.zeros(len(
                    hidden_layer_weights))
            output_layer_mapping = controller.get_mapping_for_output_neuron(i)
            # print "len(hidden_layer_weights) = "+str(len(hidden_layer_weights))
            # print "len(output_layer_mapping) = "+str(len(output_layer_mapping))
            # print "output_layer_mapping: " + str(output_layer_mapping)
            for c in xrange(len(output_layer_mapping)):
                hidden_neuron_idx = output_layer_mapping[c]
                # print "hidden_neuron_idx = " + str(hidden_neuron_idx)
                hidden_layer_firing_rate = hidden_layer_firing_rates[
                    hidden_neuron_idx]
                # print "hidden_layer_firing_rate = " + str(
                #         hidden_layer_firing_rate)
                # TODO maybe we should find a more elegant way to deal with it
                if hidden_layer_firing_rate == 0:
                    hidden_layer_firing_rate = 0.0001
                # TODO: this is a very questionable decision - to compute
                # activation  from previous layer as 1/hidden_layer_firing_rate
                nabla_w = delta*(
                    hidden_layer_firing_rate/controller.get_max_input_rate())
                # print "nabla_w = " + str(nabla_w)
                hidden_layer_weights_after_training[hidden_neuron_idx] = \
                    hidden_layer_weights[hidden_neuron_idx] - eta*nabla_w
                # print "--------"
            # print "hidden_layer_weights = " + str(hidden_layer_weights)
            # print "hidden_layer_weights_after_training = " + str(
            #         hidden_layer_weights_after_training)
            hidden_layer_biases[i] = hidden_layer_biases[i] - eta*nabla_b
            controller.set_hidden_layer_weights_for_neuron(i,
                    hidden_layer_weights_after_training)
        controller.set_hidden_layer_biases(hidden_layer_biases)
        controller.recalculate_output_layer()
        # print "leaving _backpropagate_and_train\n"

    def _init_walking_client(self):
        self._walking_client = actionlib.SimpleActionClient(
                'atlas/bdi_control', WalkDemoAction)
        print "SimpleActionClient started"
        self._mode_publisher = rospy.Publisher('/atlas/mode', String, None,
                                               False, True, queue_size=1)
        print "Mode publisher started"
        self._control_mode_publisher = rospy.Publisher('/atlas/control_mode',
                                                       String, None, False, True, queue_size=1)
        print "Control Mode publisher started"
        print "Waiting for server..."
        self._walking_client.wait_for_server()
        self._robot_walking_initialized = True

    def _send_walking_command(self):
        # This will make the robot walk controlled by the BDI controller
        # the code is modified based on the code of drcsim package
        # keyboard_teleop,py
        steps = self._build_steps()
        print "len(steps) = " + str(len(steps))
        k_effort = [0]*28
        walk_goal = WalkDemoGoal(Header(), WalkDemoGoal.WALK, steps,
                                 AtlasBehaviorStepParams(),
                                 AtlasBehaviorStandParams(),
                                 AtlasBehaviorManipulateParams(),  k_effort)
        self._walking_client.send_goal(walk_goal)

    @staticmethod
    def _build_steps():
        L = 0.15
        L_lat = 0.15
        R = 2
        W = 0.2
        X = 0
        Y = 0
        theta = 0
        dTheta = 0

        steps = []

        # Builds the sequence of steps needed
        for i in range(50):
            is_even = i%2
            is_odd = 1 - is_even
            is_right_foot = is_even
            is_left_foot = is_odd

            # left = 1, right = -1
            foot = 1 - 2 * is_right_foot

            X = (1 != 0) * (X + 1 * L)
            Y = (0 != 0) * (Y + is_odd * 0 * L_lat) + foot * W / 2

            Q = quaternion_from_euler(0, 0, theta)
            step = AtlasBehaviorStepData()

            # One step already exists, so add one to index
            step.step_index = i

            # Alternate between feet, start with left
            step.foot_index = is_right_foot

            step.duration = 0.63

            step.pose.position.x = X
            step.pose.position.y = Y
            step.pose.position.z = 0.0

            step.pose.orientation.x = Q[0]
            step.pose.orientation.y = Q[1]
            step.pose.orientation.z = Q[2]
            step.pose.orientation.w = Q[3]

            step.swing_height = 0.1
            steps.append(step)

        # Add final step to bring feet together
        is_right_foot = 1 - steps[-1].foot_index
        is_even = is_right_foot
        # foot = 1 for left, foot = -1 for right
        foot = 1 - 2 * is_right_foot

        Y = Y + foot * W

        Q = quaternion_from_euler(0, 0, theta)
        step = AtlasBehaviorStepData()
        step.step_index = len(steps)
        step.foot_index = is_right_foot
        step.duration = 0.63
        step.pose.position.x = X
        step.pose.position.y = Y
        step.pose.position.z = 0.0
        step.pose.orientation.x = Q[0]
        step.pose.orientation.y = Q[1]
        step.pose.orientation.z = Q[2]
        step.pose.orientation.w = Q[3]
        step.swing_height = 0.1

        steps.append(step)

        return steps

    def reset_robot_pose(self):
        self._mode_publisher.publish("harnessed")
        self._control_mode_publisher.publish("Freeze")
        self._control_mode_publisher.publish("StandPrep")
        rospy.sleep(2.0)
        self._mode_publisher.publish("nominal")
        rospy.sleep(0.3)
        self._control_mode_publisher.publish("Stand")
        print "walking mode switched"

    def set_control_mode(self, control_mode):
        self._control_mode_publisher.publish(control_mode)

    def save_rmse(self, rmse):
        self._rmse_saver.save_rmse(rmse)

    def write_rmse_csv(self):
        self._rmse_saver.write_rmse_csv()


INSTANCE = AtlasControllerTrainer()

def main():
    pass

if __name__ == '__main__':
    main()
