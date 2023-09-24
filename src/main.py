#!/usr/bin/env python3.8

from urc_gui_common import AppRunner, Window, CameraTab, SixCameraTab, FourCameraTab, SettingsTab, ToolsTab, VLine
from rover_gui_common import RoverMapTab

from current_display_widget import CurrentDisplayWidget
from science import ScienceTab
from wanderer_controls import WandererControlsTab
from wanderer_ros_link import WandererRosLink

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

class RoverWindow(Window):
	def __init__(self, *args, **kwargs):
		super().__init__('WVU URC Wanderer GUI', *args, **kwargs)

		roslink = WandererRosLink()

		single_camera_tab = CameraTab(roslink)
		six_camera_tab = SixCameraTab(roslink)
		four_camera_tab_1 = FourCameraTab(roslink)
		four_camera_tab_2 = FourCameraTab(roslink)
		science_tab = ScienceTab(roslink)

		super_camera_widgets = [
			single_camera_tab.camera_widget,
			six_camera_tab.main_cam,
			six_camera_tab.right_top_cam,
			six_camera_tab.right_mid_cam,
			six_camera_tab.bot_left_cam,
			six_camera_tab.bot_mid_cam,
			six_camera_tab.bot_right_cam,
			four_camera_tab_1.top_left_cam,
			four_camera_tab_1.top_right_cam,
			four_camera_tab_1.bot_left_cam,
			four_camera_tab_1.bot_right_cam,
			four_camera_tab_2.top_left_cam,
			four_camera_tab_2.top_right_cam,
			four_camera_tab_2.bot_left_cam,
			four_camera_tab_2.bot_right_cam,
			science_tab.ui.camera1,
			science_tab.ui.camera2,
			science_tab.ui.camera3,
		]

		settings_tab = SettingsTab(roslink, super_camera_widgets)
		tools_tab = ToolsTab(roslink)

		self.tabs.addTab(single_camera_tab,				'Camera')
		self.tabs.addTab(six_camera_tab,				'Six Camera')
		self.tabs.addTab(four_camera_tab_1,				'Four Camera 1')
		self.tabs.addTab(four_camera_tab_2,				'Four Camera 2')
		self.tabs.addTab(RoverMapTab(roslink),			'Map')
		self.tabs.addTab(science_tab,					'Science')
		self.tabs.addTab(WandererControlsTab(roslink),	'Controls')
		self.tabs.addTab(tools_tab,						'Tools')
		self.tabs.addTab(settings_tab,					'Settings')

		self.current_display = CurrentDisplayWidget(roslink)
		self.current_display_minimum_width = self.current_display.minimumSizeHint().width()

		self.splitter: QSplitter
		self.splitter.addWidget(self.current_display)
		self.splitter.setStretchFactor(0, 1)
		self.splitter.setOpaqueResize(False)
		self.splitter.setSizes((0, self.current_display_minimum_width))
		self.splitter.splitterMoved.connect(self.resize_splitter)

		self.status_bar.connect_roslink(roslink)
		self.status_bar.connect_timer(tools_tab.timer)

		self.pid_planner_status_label = QLabel("PID Planner Status:")
		self.pid_planner_status_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.pid_planner_status_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.pid_planner_status_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.pid_planner_status.connect(lambda status: self.pid_planner_status_display_label.setText(status.data))

		self.dwa_planner_status_label = QLabel("DWA Planner Status:")
		self.dwa_planner_status_display_label = QLabel("-")
		self.status_bar.layout.addWidget(self.dwa_planner_status_label, 1, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(self.dwa_planner_status_display_label, 3, alignment=Qt.AlignLeft)
		self.status_bar.layout.addWidget(VLine(), 1)
		roslink.dwa_planner_status.connect(lambda status: self.dwa_planner_status_display_label.setText(status.data))

	def resize_splitter(self):
		tabs_width, current_display_width = self.splitter.sizes()
		total_width = tabs_width + current_display_width
		if current_display_width > self.current_display_minimum_width:
			self.splitter.setSizes((total_width - self.current_display_minimum_width, self.current_display_minimum_width))

def main():
	app = AppRunner(RoverWindow)
	app.start()

if __name__ == '__main__':
	main()
