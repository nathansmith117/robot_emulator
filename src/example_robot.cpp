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

	// Setup rigid body.
	rp3d::Vector3 sposition(0.0, 0.0, 0.0); 
	rp3d::Quaternion orientation = rp3d::Quaternion::identity(); 
	rp3d::Transform transform(sposition, orientation); 

	rig_body = robot_emulator->physics_world->createRigidBody(transform);
	rig_body->setType(rp3d::BodyType::DYNAMIC);
	rig_body->enableGravity(false);
	rig_body->setMass(100.0);

	robot_shape = robot_emulator->physics_common.createBoxShape(rp3d::Vector3(size.x / 2, size.y / 2, size.z / 2));

	rig_body->addCollider(robot_shape, transform);
	rig_body->updateMassPropertiesFromColliders();

	force = rp3d::Vector3(0.0, 0.0, 0.0);
	point_of_force = rp3d::Vector3(size.x, 0.0, size.z);

	force2 = rp3d::Vector3(0.0, 0.0, 0.0);
	point_of_force2 = rp3d::Vector3(-size.x, 0.0, -size.z);

	rig_body->setLinearDamping(2.0);
	rig_body->setAngularDamping(10.0);
}

void ExampleRobot::update() {
	CameraWithCb * camera = get_current_camera_with_cb();

	const float speed = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y) * settings.speed;
	const float turn_power = GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X) * settings.turn_power;

	force.x = -speed - turn_power;
	force2.x = -speed + turn_power;
	
	// Event handling.
	if (IsKeyDown(KEY_W)) { // Forward.
		force.x = settings.speed;
		force2.x = settings.speed;
	} else if (IsKeyDown(KEY_S)) { // Backward.
		force.x = -settings.speed;
		force2.x = -settings.speed;
	}

	if (IsKeyDown(KEY_D)) { // Right.
		force.x -= settings.turn_power;
		force2.x += settings.turn_power;
	} if (IsKeyDown(KEY_A)) { // Left.
		force.x += settings.turn_power;
		force2.x -= settings.turn_power;
	}

	rig_body->applyLocalForceAtLocalPosition(force, point_of_force);
	rig_body->applyLocalForceAtLocalPosition(force2, point_of_force2);

	// Stuff.
	rp3d::Transform curr_transform = rig_body->getTransform();
	rp3d::Transform inter_transform;

	inter_transform = rp3d::Transform::interpolateTransforms(prev_transform, curr_transform, robot_emulator->time_factor);

	rp3d::Vector3 transform_angle = inter_transform.getOrientation().getVectorV();
	rp3d::Matrix3x3 transform_matrix = inter_transform.getOrientation().getMatrix();
	rp3d::Quaternion q = inter_transform.getOrientation();
	rp3d::Vector3 transform_position = inter_transform.getPosition();


	rp3d::Vector3 the_angles = euler_angles(transform_matrix);
	
	angle.pitch = the_angles.y;
	angle.yaw = the_angles.x;
	angle.roll = the_angles.z;

	angle.yaw *= RAD2DEG;
	angle.pitch *= RAD2DEG;
	angle.roll *= RAD2DEG;

	angle = get_non_neg_angle(angle);
	angle = wrap_angle_deg(angle);

	position.x = transform_position.x;
	position.y = transform_position.y;
	position.z = transform_position.z;

	prev_transform = curr_transform;

	update_robot_body();

	// Update camera.
	if (camera == NULL)
		return;

	camera->call_cb();
}

void ExampleRobot::draw() {
	int x, y;

	if (!should_draw)
		return;
	
	for (y = 0; y < ROBOT_BODY_SIZE; y++)
		for (x = 0; x < ROBOT_BODY_SIZE; x++)
			DrawLine3D(robot_body[x], robot_body[y], BLACK);
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
