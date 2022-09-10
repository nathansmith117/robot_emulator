#include "robot_emulator.h"
#include "example_robot.h"

int main(int argc, char ** argv) {
	RobotEmulator * robot_emulator = new RobotEmulator();

	// Open window and setup camera.
	InitWindow(robot_emulator->settings.window_width, robot_emulator->settings.window_height, argv[0]);

	// Set window state.
	unsigned int window_state = FLAG_WINDOW_RESIZABLE;

	if (robot_emulator->settings.start_fullscreen)
		window_state |= FLAG_FULLSCREEN_MODE;

	SetWindowState(window_state);

	// Setup.
	robot_emulator->setup();

	robot_emulator->robot = new ExampleRobot(robot_emulator);
	robot_emulator->physics_world->setIsDebugRenderingEnabled(true);

	
	while (!WindowShouldClose()) {
		// Update.
		robot_emulator->update();

		// Draw.
		robot_emulator->draw();
	}

	CloseWindow();
	return 0;
}
