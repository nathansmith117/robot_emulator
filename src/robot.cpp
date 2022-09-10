#include "robot.h"
#include "robot_emulator.h"

Robot::Robot(RobotEmulator * robot_emulator) {
	this->robot_emulator = robot_emulator;

	position = {0.0, 0.0, 0.0};
	size = {0.0, 0.0, 0.0};
	angle = {0.0, 0.0, 0.0};

	should_draw = true;
	current_camera = NO_CAMERA;

	// Create robot body.
	rp3d::Vector3 start_position(0.0, 0.0, 0.0); 
	rp3d::Quaternion orientation = rp3d::Quaternion::identity(); 
	rp3d::Transform transform(start_position, orientation); 

	robot_body = robot_emulator->physics_world->createRigidBody(transform);
	robot_body->setType(rp3d::BodyType::DYNAMIC);
}

Robot::~Robot() {
	// Destory robot body.
	if (robot_body != NULL)
		robot_emulator->physics_world->destroyRigidBody(robot_body);
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
	// Apply force.
	robot_body->applyLocalForceAtLocalPosition(right_force, right_point_of_force);
	robot_body->applyLocalForceAtLocalPosition(left_force, left_point_of_force);

	// Get robot body transform.
	rp3d::Transform curr_transform = robot_body->getTransform();
	rp3d::Transform inter_transform;

	inter_transform = rp3d::Transform::interpolateTransforms(prev_transform, curr_transform, robot_emulator->time_factor);

	// Get angle.
	rp3d::Matrix3x3 transform_matrix = inter_transform.getOrientation().getMatrix();
	rp3d::Vector3 transform_position = inter_transform.getPosition();

	rp3d::Vector3 the_angles = euler_angles(transform_matrix);
	
	angle.pitch = the_angles.y * RAD2DEG;
	angle.yaw = the_angles.x * RAD2DEG;
	angle.roll = the_angles.z * RAD2DEG;

	angle = get_non_neg_angle(angle);
	angle = wrap_angle_deg(angle);

	// Set position.
	position.x = transform_position.x;
	position.y = transform_position.y;
	position.z = transform_position.z;

	prev_transform = curr_transform;
}

void Robot::set_position(Vector3 position) {
	this->position = position;

	if (robot_body == NULL)
		return;

	// Get transform and position.
	rp3d::Transform trans = robot_body->getTransform();
	rp3d::Vector3 pos = trans.getPosition();

	pos.x = position.x;
	pos.y = position.y;
	pos.z = position.z;

	// Set transform and position.
	trans.setPosition(pos);
	robot_body->setTransform(trans);
}

void Robot::reset_force_data() {
	// Right.
	right_force = rp3d::Vector3(0.0, 0.0, 0.0);
	right_point_of_force = rp3d::Vector3(size.x, 0.0, size.z);

	// Left.
	left_force = rp3d::Vector3(0.0, 0.0, 0.0);
	left_point_of_force = rp3d::Vector3(-size.x, 0.0, -size.z);
}
