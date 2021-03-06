#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "p5/spring.hpp"
#include "p5/body.hpp"
#include "p5/spherebody.hpp"
#include <iostream>
#include "stdio.h"

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero();
    body2_offset = Vector3::Zero();
    damping = 0.0;
}

void Spring::step( real_t dt )
{
	// relative distance
	real_t x = distance(body1->position + body1->orientation * body1_offset, body2->position + body2->orientation * body2_offset) - equilibrium;
	// direction from spring to body1
	Vector3 springToBody1Dir = normalize((body1->position + body1_offset) - (body2->position + body2_offset));
	// direction from spring to body2
	Vector3 springToBody2Dir = normalize((body2->position + body2_offset) - (body1->position + body1_offset));
	// relative velocity of body1 to spring
	real_t veloctiyRelativeValue_1 = dot(body1->velocity - body2->velocity, springToBody1Dir);
	// relative velocity of body2 to spring
	real_t veloctiyRelativeValue_2 = dot(body2->velocity - body1->velocity, springToBody2Dir);
	// force value applied to body1
	real_t fValue_1 = (-constant * x - damping * veloctiyRelativeValue_1);
	// force value applied to body2
	real_t fValue_2 = (-constant * x - damping * veloctiyRelativeValue_2);
	// force 1 direction
	Vector3 F1 = springToBody1Dir * fValue_1;
	// force 2 direction
	Vector3 F2 = springToBody2Dir * fValue_2;
	// apply force and torque
	body1->apply_force(F1, body1->orientation * body1_offset);
	body2->apply_force(F2, body2->orientation * body2_offset);
}

}


