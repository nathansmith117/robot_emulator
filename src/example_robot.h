#pragma once

#include "robot_emulator.h"
#include "robot.h"


#define EXAMPLE_ROBOT_CAMERA_DIS 3.0

class ExampleRobot : public Robot {
	public:
		ExampleRobot(RobotEmulator * robot_emulator) : Robot(robot_emulator) {
			setup(robot_emulator);
		}

		void update();
		void draw();
	private:
		void setup(RobotEmulator * robot_emulator);

		static void update_camera(CAMERA_CB_ARGS);
		static void update_camera2(CAMERA_CB_ARGS);

		Vector3 robot_corners[ROBOT_BODY_SIZE];
};
