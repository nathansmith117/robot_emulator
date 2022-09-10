#include "robot_emulator.h"
#include "robot.h"

bool closeEnough(const float& a, const float& b, const float& epsilon) {
    return (epsilon > std::abs(a - b));
}

rp3d::Vector3 euler_angles(rp3d::Matrix3x3 R) {

    //check for gimbal lock
    if (closeEnough(R[0][2], -1.0f)) {
        float x = 0; //gimbal lock, value of x doesn't matter
        float y = PI / 2;
        float z = x + atan2(R[1][0], R[2][0]);
        return { x, y, z };
    } else if (closeEnough(R[0][2], 1.0f)) {
        float x = 0;
        float y = -PI / 2;
        float z = -x + atan2(-R[1][0], -R[2][0]);
        return { x, y, z };
    } else { //two solutions exist
        float x1 = -asin(R[0][2]);
        float x2 = PI - x1;

        float y1 = atan2(R[1][2] / cos(x1), R[2][2] / cos(x1));
        float y2 = atan2(R[1][2] / cos(x2), R[2][2] / cos(x2));

        float z1 = atan2(R[0][1] / cos(x1), R[0][0] / cos(x1));
        float z2 = atan2(R[0][1] / cos(x2), R[0][0] / cos(x2));

        //choose one solution to return
        //for example the "shortest" rotation
        if ((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2))) {
            return rp3d::Vector3(x1, y1, z1);
        } else {
            return rp3d::Vector3(x2, y2, z2);
        }
    }
}

AngleData wrap_angle_deg(AngleData angle) {
	AngleData new_angle;

	new_angle.roll = Wrap(angle.roll, 0, 360.0);
	new_angle.yaw = Wrap(angle.yaw, 0, 360.0);
	new_angle.pitch = Wrap(angle.pitch, 0, 360.0);

	return new_angle;
}

AngleData get_non_neg_angle(AngleData angle) {
	AngleData res;
	
	res.roll = make_angle_non_neg(angle.roll);
	res.yaw = make_angle_non_neg(angle.yaw);
	res.pitch = make_angle_non_neg(angle.pitch);

	return res;
}

float make_angle_non_neg(float angle) {
	return (angle >= 0.0) ? angle : 360.0 + angle;
}

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

	if (!settings.show_debug_into)
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

			DrawLine3D(start_pos, end_pos, BLUE);
		}

	if (triangles != NULL)
		for (i = 0; i < num_of_tri; i++) {
			triangle = &triangles[i];

			point1 = {triangle->point1.x, triangle->point1.y, triangle->point1.z};
			point2 = {triangle->point2.x, triangle->point2.y, triangle->point2.z};
			point3 = {triangle->point3.x, triangle->point3.y, triangle->point3.z};

			DrawTriangle3D(point1, point2, point3, BLUE);
		}
}
