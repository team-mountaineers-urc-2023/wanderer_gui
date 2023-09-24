#!/usr/bin/env python3.8

from rover_gui_common import RoverRosLink

import rospy

from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

from hockey_stick_arm.srv import SetFloat, SetFloatRequest, SetFloatResponse
from ocean_optics_spectrometer.srv import CaptureSpectrometer, CaptureSpectrometerRequest, CaptureSpectrometerResponse
from science_servo_control.srv import FreeMoveDrum, FreeMoveDrumRequest, FreeMoveDrumResponse, \
                                      LowerLinearActuator, LowerLinearActuatorRequest, LowerLinearActuatorResponse, \
                                      MixSample, MixSampleRequest, MixSampleResponse, \
                                      MoveCuvette, MoveCuvetteRequest, MoveCuvetteResponse, \
                                      MoveLinearActuator, MoveLinearActuatorRequest, MoveLinearActuatorResponse, \
                                      MovePump, MovePumpRequest, MovePumpResponse, \
                                      PreMixDrum, PreMixDrumRequest, PreMixDrumResponse, \
                                      PreSealDrum, PreSealDrumRequest, PreSealDrumResponse, \
                                      RaiseLinearActuator, RaiseLinearActuatorRequest, RaiseLinearActuatorResponse, \
                                      ReadDrumPosition, ReadDrumPositionRequest, ReadDrumPositionResponse, \
                                      RebootDrum, RebootDrumRequest, RebootDrumResponse, \
                                      ScoopSample, ScoopSampleRequest, ScoopSampleResponse, \
                                      SealDrum, SealDrumRequest, SealDrumResponse, \
                                      SpinCentrifuge, SpinCentrifugeRequest, SpinCentrifugeResponse, \
                                      WriteGPIO, WriteGPIORequest, WriteGPIOResponse

from planner_multiplexer.srv import SetPlanner, SetPlannerRequest, SetPlannerResponse

from PyQt5.QtCore import pyqtSignal as Signal

class WandererRosLink(RoverRosLink):
	"""Supply signals for the Qt app, originating from topic callbacks."""

	arm_currents = Signal(Float32MultiArray)
	drivetrain_currents = Signal(Float32MultiArray)

	pid_planner_status = Signal(String)
	dwa_planner_status = Signal(String)

	def __init__(self):
		super().__init__()

		### autonomy #########################################################

		self.pid_planner_name = rospy.get_param("~pid_planner_name")
		self.dwa_planner_name = rospy.get_param("~dwa_planner_name")

		pid_planner_status_topic = rospy.get_param("~pid_planner_status_topic")
		dwa_planner_status_topic = rospy.get_param("~dwa_planner_status_topic")

		select_planner_service = rospy.get_param("~select_planner_service")
		pid_planner_enabled_service = rospy.get_param("~pid_planner_enabled_service")
		dwa_planner_enabled_service = rospy.get_param("~dwa_planner_enabled_service")

		self.pid_planner_status_sub = self.make_subscriber(pid_planner_status_topic, String, self.pid_planner_status)
		self.dwa_planner_status_sub = self.make_subscriber(dwa_planner_status_topic, String, self.dwa_planner_status)

		self.select_planner_serv = rospy.ServiceProxy(select_planner_service, SetPlanner)
		self.enable_pid_planner_serv = rospy.ServiceProxy(pid_planner_enabled_service, SetBool)
		self.enable_dwa_planner_serv = rospy.ServiceProxy(dwa_planner_enabled_service, SetBool)

		### drivetrain #######################################################

		drivetrain_currents_topic = "drivetrain_current_data"

		self.drivetrain_currents_sub = self.make_subscriber(drivetrain_currents_topic, Float32MultiArray, self.drivetrain_currents)

		### arm ##############################################################

		arm_currents_topic = "/arm_current"
		arm_control_service = "/UseVeloControl"
		arm_speed_multipler_service = "/SetSpeedMultiplier"
		arm_safety_check_service = "/UseArmSafetyCheck"

		self.arms_currents_sub = self.make_subscriber(arm_currents_topic, Float32MultiArray, self.arm_currents)
		self.arm_control_serv = rospy.ServiceProxy(arm_control_service, SetBool)
		self.arm_speed_multiplier_serv = rospy.ServiceProxy(arm_speed_multipler_service, SetFloat)
		self.arm_safety_check_serv = rospy.ServiceProxy(arm_safety_check_service, SetBool)

		### science ##########################################################

		self.linear_act_serv = rospy.ServiceProxy("/move_linear_actuator", MoveLinearActuator)
		self.pump_serv = rospy.ServiceProxy("/move_pump", MovePump)
		self.scoop_sample_serv = rospy.ServiceProxy("/scoop_sample", ScoopSample)
		self.pre_mix_drum_serv = rospy.ServiceProxy("/pre_mix_drum", PreMixDrum)
		self.mix_sample_serv = rospy.ServiceProxy("/mix_sample", MixSample)
		self.pre_seal_drum_serv = rospy.ServiceProxy("/pre_seal_drum", PreSealDrum)
		self.seal_drum_serv = rospy.ServiceProxy("/seal_drum", SealDrum)
		self.free_move_drum_serv = rospy.ServiceProxy("/free_move_drum", FreeMoveDrum)
		self.move_cuvette_serv = rospy.ServiceProxy("/move_cuvette", MoveCuvette)
		self.spin_centrifuge_serv = rospy.ServiceProxy("/spin_centrifuge", SpinCentrifuge)
		self.gpio_serv = rospy.ServiceProxy("/gpio", WriteGPIO)
		self.raise_linear_actuator_serv = rospy.ServiceProxy("/raise_linear_actuator", RaiseLinearActuator)
		self.lower_linear_actuator_serv = rospy.ServiceProxy("/lower_linear_actuator", LowerLinearActuator)
		self.read_drum_position_serv = rospy.ServiceProxy("/read_drum_position", ReadDrumPosition)
		self.spectrometer_serv = rospy.ServiceProxy("/spectrometer", CaptureSpectrometer)
		self.reboot_drum_serv = rospy.ServiceProxy("/reboot_drum", RebootDrum)

	### autonomy #############################################################

	def set_planner(self, planner: str) -> SetPlannerResponse:
		return self.select_planner_serv(SetPlannerRequest(planner))

	def enable_pid_planner(self, enable: bool) -> SetBoolResponse:
		return self.enable_pid_planner_serv(SetBoolRequest(enable))

	def enable_dwa_planner(self, enable: bool) -> SetBoolResponse:
		return self.enable_dwa_planner_serv(SetBoolRequest(enable))

	### arm ##################################################################

	def change_arm_control(self, velocity: bool) -> SetBoolResponse:
		return self.arm_control_serv(SetBoolRequest(data=velocity))

	def change_arm_speed_multiplier(self, multipler: bool) -> SetFloatResponse:
		return self.arm_speed_multiplier_serv(SetFloatRequest(value=multipler))

	def change_arm_safety_features(self, enable_arm_safety: bool) -> SetBoolResponse:
		return self.arm_safety_check_serv(SetBoolRequest(data=enable_arm_safety))

	### science ##############################################################

	def linear_act(self, actuator_id: int, pwm_pos: int) -> MoveLinearActuatorResponse:
		return self.linear_act_serv(MoveLinearActuatorRequest(actuator_id, pwm_pos))

	def pump(self, pump_id: int, ppm_speed: int, duration: int) -> MovePumpResponse:
		return self.pump_serv(MovePumpRequest(pump_id, ppm_speed, duration))

	def scoop_sample(self, drum_id: int) -> ScoopSampleResponse:
		return self.scoop_sample_serv(ScoopSampleRequest(drum_id))

	def pre_mix_drum(self, drum_id: int) -> PreMixDrumResponse:
		return self.pre_mix_drum_serv(PreMixDrumRequest(drum_id))

	def mix_sample(self, drum_id: int) -> MixSampleResponse:
		return self.mix_sample_serv(MixSampleRequest(drum_id))
	
	def pre_seal_drum(self, drum_id: int) -> PreSealDrumResponse:
		return self.pre_seal_drum_serv(PreSealDrumRequest(drum_id))

	def seal_drum(self, drum_id: int) -> SealDrumResponse:
		return self.seal_drum_serv(SealDrumRequest(drum_id))

	def free_move_drum(self, drum_id: int, drum_pos: int) -> FreeMoveDrumResponse:
		return self.free_move_drum_serv(FreeMoveDrumRequest(drum_id, drum_pos))

	def move_cuvette(self, cuvette_num: int) -> MoveCuvetteResponse:
		return self.move_cuvette_serv(MoveCuvetteRequest(cuvette_num))

	def spin_centrifuge(self) -> SpinCentrifugeResponse:
		return self.spin_centrifuge_serv(SpinCentrifugeRequest())

	def gpio(self, id: int, value: int) -> WriteGPIOResponse:
		return self.gpio_serv(WriteGPIORequest(id, value))

	def raise_linear_actuator(self, lin_act_id: int) -> RaiseLinearActuatorResponse:
		return self.raise_linear_actuator_serv(RaiseLinearActuatorRequest(lin_act_id))

	def lower_linear_actuator(self, lin_act_id: int) -> LowerLinearActuatorResponse:
		return self.lower_linear_actuator_serv(LowerLinearActuatorRequest(lin_act_id))

	def read_drum_position(self, id: int) -> ReadDrumPositionResponse:
		return self.read_drum_position_serv(ReadDrumPositionRequest(id))

	def capture_spectrometer(self, integration_time: int) -> CaptureSpectrometerResponse:
		return self.spectrometer_serv(CaptureSpectrometerRequest(integration_time=integration_time))
	
	def reboot_drum(self, drum_id: int) -> RebootDrumResponse:
		return self.reboot_drum_serv(RebootDrumRequest(drum_id))

if __name__ == "__main__":
	rospy.loginfo("Starting GUI Wanderer ROS link")
	roslink = WandererRosLink()
	rospy.spin()
