#include "example_robot.h"
#include <raylib.h>

void ExampleRobot::setup(RobotEmulator * robot_emulator) {
	size = {1.0, 0.5, 1.0};
	position = {0.0, size.y * 2, 0.0};
	angle = {0.0, 0.0, 0.0};

	// Setup camera.
	Camera camera;
	camera.position = {0.0, 2.0, 0.0};
	camera.target = position;
	camera.up = {0.0, 1.0, 0.0};
	camera.fovy = robot_emulator->settings.fov;
	camera.projection = CAMERA_PERSPECTIVE;

	SetCameraMode(camera, CAMERA_CUSTOM);

	current_camera = 0;
	camera_list.push_back({camera, update_camera, robot_emulator, this, NULL});
	camera_list.push_back({camera, update_camera2, robot_emulator, this, NULL});

	SetTargetFPS(robot_emulator->settings.fps);

	// Settings.
	settings.speed = 8.0;
	settings.turn_power = 0.7;
	settings.mass = 100.0;

	// Setup rigid body.
	rp3d::Vector3 sposition(0.0, 0.0, 0.0); 
	rp3d::Quaternion orientation = rp3d::Quaternion::identity(); 
	rp3d::Transform transform(sposition, orientation); 

	robot_body->enableGravity(false);
	robot_body->setMass(settings.mass);

	rp3d::BoxShape * robot_shape = robot_emulator->physics_common.createBoxShape(rp3d::Vector3(size.x / 2, size.y / 2, size.z / 2));

	robot_body->addCollider(robot_shape, transform);
	robot_body->updateMassPropertiesFromColliders();

	robot_body->setLinearDamping(2.0);
	robot_body->setAngularDamping(7.0);

	reset_force_data();
}

void ExampleRobot::update() {
	CameraWithCb * camera = get_current_camera_with_cb();

	const float speed = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y) * settings.speed;
	const float turn_power = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) * settings.turn_power;

	right_force.x = -speed - turn_power;
	left_force.x = -speed + turn_power;
	
	// Event handling.
	if (IsKeyDown(KEY_W)) { // Forward.
		right_force.x = settings.speed;
		left_force.x = settings.speed;
	} else if (IsKeyDown(KEY_S)) { // Backward.
		right_force.x = -settings.speed;
		left_force.x = -settings.speed;
	}

	if (IsKeyDown(KEY_D)) { // Right.
		right_force.x -= settings.turn_power;
		left_force.x += settings.turn_power;
	} if (IsKeyDown(KEY_A)) { // Left.
		right_force.x += settings.turn_power;
		left_force.x -= settings.turn_power;
	}

	update_robot_body();

	// Update camera.
	if (camera == NULL)
		return;

	camera->call_cb();
}

void ExampleRobot::draw() {
	int i, j;
	float x, y, z;
    Matrix rotation;

	// Half size.
	Vector3 hs = {size.x / 2, size.y / 2, size.z / 2};

	// Corners in a rectangle.
	Vector2 top_left = {-hs.x, -hs.y};
	Vector2 top_right = {hs.x, -hs.y};
	Vector2 bottom_right = {hs.x, hs.y};
	Vector2 bottom_left = {-hs.x, hs.y};

	robot_corners[0] = {top_left.x, top_left.y, hs.z};
	robot_corners[1] = {top_right.x, top_right.y, hs.z};
	robot_corners[2] = {bottom_right.x, bottom_right.y, hs.z};
	robot_corners[3] = {bottom_left.x, bottom_left.y, hs.z};

	// Create other side of robot body.
	for (i = 4; i < ROBOT_BODY_SIZE; i++) {
		robot_corners[i] = robot_corners[i - 4];
		robot_corners[i].z = -hs.z;
	}

	rotation = angle.get_matrix();

	// Move to position and rotate.
	for (i = 0; i < ROBOT_BODY_SIZE; i++) {
		x = robot_corners[i].x;
		y = robot_corners[i].y;
		z = robot_corners[i].z;

		robot_corners[i].x = rotation.m0 * x + rotation.m1 * y + rotation.m2 * z;
		robot_corners[i].y = rotation.m4 * x + rotation.m5 * y + rotation.m6 * z;
		robot_corners[i].z = rotation.m8 * x + rotation.m9 * y + rotation.m10 * z;

		robot_corners[i].x += position.x;
		robot_corners[i].y += position.y;
		robot_corners[i].z += position.z;
	}

	if (!should_draw)
		return;
	
	for (i = 0; i < ROBOT_BODY_SIZE; i++)
		for (j = 0; j < ROBOT_BODY_SIZE; j++)
			DrawLine3D(robot_corners[i], robot_corners[j], BLACK);
}

void ExampleRobot::update_camera(CAMERA_CB_ARGS) {
	AngleData angle = robot->get_angle();
	Vector3 position = robot->get_position();

	Vector2 camera_pos = {EXAMPLE_ROBOT_CAMERA_DIS, 0.0};
	camera_pos = Vector2Rotate(camera_pos, (angle.yaw + 180.0) * DEG2RAD);

	camera->position.x = position.x + camera_pos.x;
	camera->position.y = EXAMPLE_ROBOT_CAMERA_DIS;
	camera->position.z = position.z + camera_pos.y;

	camera->target = position;
}

void ExampleRobot::update_camera2(CAMERA_CB_ARGS) {
	camera->target = robot->get_position();
}
