#pragma once

// Reactphysics3d.
#include <reactphysics3d/reactphysics3d.h>

// Raylib.
#include <raylib.h>
#include <raymath.h>

#define _USE_MATH_DEFINES

// C/C++
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <climits>
#include <vector>

// OS headers.
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Other.
#include "util.h"

struct Settings {

	// Window.
	int window_width = 800;
	int window_height = 600;
	bool start_fullscreen = false;

	// Debug.
	bool show_debug_info = false;

	// Display.
	float fov = 90.0;
	int fps = 60;
};

struct AngleData {
	float roll, yaw, pitch;

	Matrix get_matrix() {
		return MatrixRotateXYZ({
			roll * DEG2RAD, 
			yaw * DEG2RAD, 
			pitch * DEG2RAD}
		);
	}
};

#define CAMERA_CB_ARGS class RobotEmulator * robot_emulator, class Robot * robot, Camera * camera, void * d
typedef void (*CAMERA_CB)(CAMERA_CB_ARGS);

struct CameraWithCb {
	Camera camera;
	CAMERA_CB cb = NULL;
	class RobotEmulator * robot_emulator = NULL;
	class Robot * robot = NULL;
	void * data = NULL;

	void call_cb(class RobotEmulator * r_m, class Robot * r, void * d) {
		if (cb == NULL)
			return;

		cb(r_m, r, &camera, d);
	}

	void call_cb() {
		if (cb == NULL)
			return;

		cb(robot_emulator, robot, &camera, data);
	}
};

class RobotEmulator {
	public:
		RobotEmulator();
		~RobotEmulator();
		
		void setup();

		void update();
		void update_physics_world();

		void draw();
		void draw_world();
		void draw_robot(class Robot * robot);

		void draw_debug();

		Settings settings;
		class Robot * robot = NULL;

		rp3d::PhysicsCommon physics_common;
		rp3d::PhysicsWorld * physics_world = NULL;
		rp3d::DebugRenderer * debug_render;

		double accumulator;
		double previous_time;
		double time_factor;
};
