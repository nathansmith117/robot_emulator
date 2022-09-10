#pragma once

#include "robot_emulator.h"
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>

struct RobotSettings {
	float speed;
	float turn_power;
	float mass;
};

#define NO_CAMERA -1
#define ROBOT_BODY_SIZE 8

class Robot {
	public:
		Robot(RobotEmulator * robot_emulator);
		~Robot();

		virtual void update() = 0;
		virtual void draw() = 0;

		virtual void update_robot_body();

		virtual void reset_force_data();

		Vector3 get_position() { return position; }
		virtual void set_position(Vector3 position);

		Vector3 get_size() { return size; }
		void set_size(Vector3 size) { this->size = size; }

		AngleData get_angle() { return angle; }
		void set_angle(AngleData angle) { this->angle = angle; }

		bool get_should_draw() { return should_draw; }
		void set_should_draw(bool should_draw) { this->should_draw = should_draw; }

		RobotSettings get_settings() { return settings; }
		void set_settings(RobotSettings settings) { this->settings = settings; }

		int get_current_camera_id() { return current_camera; }
		void set_current_camera_id(int current_camera);

		CameraWithCb * get_current_camera_with_cb();
		Camera * get_current_camera();

		void add_camera(CameraWithCb camera) { camera_list.push_back(camera); }
		int get_camera_count() { return camera_list.size(); }

		std::vector<CameraWithCb> get_camera_list() { return camera_list; }
		void set_camera_list(std::vector<CameraWithCb> camera_list) { this->camera_list = camera_list; }
	protected:
		RobotEmulator * robot_emulator = NULL;
		bool should_draw;

		RobotSettings settings;

		Vector3 position;
		Vector3 size;
		AngleData angle;

		int current_camera;
		std::vector<CameraWithCb> camera_list;

		rp3d::RigidBody * robot_body = NULL;
		rp3d::Transform prev_transform;

		rp3d::Vector3 right_force;
		rp3d::Vector3 right_point_of_force;

		rp3d::Vector3 left_force;
		rp3d::Vector3 left_point_of_force;
};	
