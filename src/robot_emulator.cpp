#include "robot_emulator.h"
#include "robot.h"

RobotEmulator::RobotEmulator() {
	// Do not put video stuff in here.
	
	accumulator = 0.0;
	time_factor = 0.0;
}

RobotEmulator::~RobotEmulator() {
}

void RobotEmulator::setup() {
	physics_world = physics_common.createPhysicsWorld();
	debug_render = &physics_world->getDebugRenderer();

	// Select the contact points and contact normals to be displayed 
	debug_render->setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_POINT, true); 
	debug_render->setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::CONTACT_NORMAL, true);
	debug_render->setIsDebugItemDisplayed(rp3d::DebugRenderer::DebugItem::COLLISION_SHAPE, true);
}

void RobotEmulator::update() {
	int char_pressed = GetCharPressed();

	if (robot != NULL) {
		robot->update();

		// Set current camera. Skip 0.
		if (char_pressed >= '1' && char_pressed <= '9')
			robot->set_current_camera_id(char_pressed - '1');
	}

	update_physics_world();
}

void RobotEmulator::update_physics_world() {
	const double time_step = 1.0 / settings.fps;
	double current_time = GetTime();
	double update_time = current_time - previous_time;

	previous_time = current_time;
	accumulator += update_time;

	while (accumulator >= time_step) {
		physics_world->update(time_step);
		accumulator -= time_step;
	}

	time_factor = accumulator / time_step;
}

void RobotEmulator::draw() {
	BeginDrawing();
    ClearBackground(RAYWHITE);

	draw_robot(robot);

	EndDrawing();
}

void RobotEmulator::draw_world() {
	DrawGrid(100, 1.0f);
	draw_debug();
}

void RobotEmulator::draw_robot(class Robot * robot) {
	Camera * robot_camera;

	if (robot == NULL)
		return;

	robot_camera = robot->get_current_camera();

	// Later me.  Add default camera.
	if (robot_camera == NULL) {
		return;
	}

	BeginMode3D(*robot_camera);

	draw_world();
	robot->draw();

	EndMode3D();
}

void RobotEmulator::draw_debug() {
	int i;

	int num_of_lines;
	int num_of_tri;

	if (!settings.show_debug_info)
		return;

	const rp3d::DebugRenderer::DebugLine * lines = NULL;
	const rp3d::DebugRenderer::DebugTriangle * triangles = NULL;

	const rp3d::DebugRenderer::DebugLine * line;
	const rp3d::DebugRenderer::DebugTriangle * triangle;

	Vector3 start_pos;
	Vector3 end_pos;

	Vector3 point1;
	Vector3 point2;
	Vector3 point3;

	num_of_lines = debug_render->getNbLines();
	num_of_tri = debug_render->getNbTriangles();

	if (num_of_lines > 0)
		lines = debug_render->getLinesArray();
	if (num_of_tri > 0)
		triangles = debug_render->getTrianglesArray();

	// Draw lines.
	if (lines != NULL)
		for (i = 0; i < num_of_lines; i++) {
			lines = &lines[i];

			start_pos = {line->point1.x, line->point1.y, line->point1.z};
			end_pos = {line->point2.x, line->point2.y, line->point2.z};

			DrawLine3D(start_pos, end_pos, get_debug_color(line->color1));
		}

	if (triangles != NULL)
		for (i = 0; i < num_of_tri; i++) {
			triangle = &triangles[i];

			point1 = {triangle->point1.x, triangle->point1.y, triangle->point1.z};
			point2 = {triangle->point2.x, triangle->point2.y, triangle->point2.z};
			point3 = {triangle->point3.x, triangle->point3.y, triangle->point3.z};

			DrawTriangle3D(point1, point2, point3, get_debug_color(triangle->color1));
		}
}
