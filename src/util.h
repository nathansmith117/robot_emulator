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

bool closeEnough(const float& a, const float& b, const float& epsilon = std::numeric_limits<float>::epsilon());

rp3d::Vector3 euler_angles(rp3d::Matrix3x3 R);

// https://www.reactphysics3d.com/documentation/api/html/classreactphysics3d_1_1_debug_renderer.html
Color get_debug_color(uint32_t c);

struct AngleData wrap_angle_deg(struct AngleData angle);
struct AngleData get_non_neg_angle(struct AngleData angle);
float make_angle_non_neg(float angle);
