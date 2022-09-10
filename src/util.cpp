#include "util.h"
#include "robot_emulator.h"

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

Color get_debug_color(uint32_t c) {
	switch (c) {
		case 0xff0000:
			return RED;
		case 0x00ff00:
			return GREEN;
		case 0x0000ff:
			return BLUE;
		case 0x0:
			return BLACK;
		case 0xffffff:
			return WHITE;
		case 0xffff00:
			return YELLOW;
		case 0xff00ff:
			return MAGENTA;
		case 0x00ffff: // Cyan
			return {0, 255, 255, 255};
		default:
			return BLACK;
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
