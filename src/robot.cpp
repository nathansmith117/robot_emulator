#include "robot.h"
#include "robot_emulator.h"

Robot::Robot(RobotEmulator * robot_emulator) {
	this->robot_emulator = robot_emulator;

	position = {0.0, 0.0, 0.0};
	size = {0.0, 0.0, 0.0};
	angle = {0.0, 0.0, 0.0};

	should_draw = true;
	current_camera = NO_CAMERA;
}

Robot::~Robot() {
}

void Robot::set_current_camera_id(int current_camera) {
	if (current_camera >= camera_list.size())
		return;
	if (current_camera < 0 && current_camera != NO_CAMERA)
		return;

	this->current_camera = current_camera;
}

CameraWithCb * Robot::get_current_camera_with_cb() {
	if (current_camera >= camera_list.size() || current_camera < 0)
		return NULL;

	return &camera_list[current_camera];
}

Camera * Robot::get_current_camera() {
	if (current_camera >= camera_list.size() || current_camera < 0)
		return NULL;

	return &camera_list[current_camera].camera;
}

void Robot::update_robot_body() {
	int i;
	float x, y, z;
    Matrix rotation;

	// Half size.
	Vector3 hs = {size.x / 2, size.y / 2, size.z / 2};

	// Corners in a rectangle.
	Vector2 top_left = {-hs.x, -hs.y};
	Vector2 top_right = {hs.x, -hs.y};
	Vector2 bottom_right = {hs.x, hs.y};
	Vector2 bottom_left = {-hs.x, hs.y};

	robot_body[0] = {top_left.x, top_left.y, hs.z};
	robot_body[1] = {top_right.x, top_right.y, hs.z};
	robot_body[2] = {bottom_right.x, bottom_right.y, hs.z};
	robot_body[3] = {bottom_left.x, bottom_left.y, hs.z};

	// Create other side of robot body.
	for (i = 4; i < ROBOT_BODY_SIZE; i++) {
		robot_body[i] = robot_body[i - 4];
		robot_body[i].z = -hs.z;
	}

	rotation = angle.get_matrix();

	// Move to position and rotate.
	for (i = 0; i < ROBOT_BODY_SIZE; i++) {
		x = robot_body[i].x;
		y = robot_body[i].y;
		z = robot_body[i].z;

		robot_body[i].x = rotation.m0 * x + rotation.m1 * y + rotation.m2 * z;
		robot_body[i].y = rotation.m4 * x + rotation.m5 * y + rotation.m6 * z;
		robot_body[i].z = rotation.m8 * x + rotation.m9 * y + rotation.m10 * z;

		robot_body[i].x += position.x;
		robot_body[i].y += position.y;
		robot_body[i].z += position.z;
	}
}
