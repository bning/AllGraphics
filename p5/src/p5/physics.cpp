#include "p5/physics.hpp"
#include "stdio.h"
namespace _462 {

/*
 * Given the orientation q(t), current angular velocity omega and time step deltaT,
 * return the orientation q(t+1) after deltaT time step
 *
 */
inline Quaternion quatIntegrate(const Quaternion& q, const Vector3& omega, real_t deltaT) 
{ 
	Quaternion deltaQ; 
	Vector3 theta = omega * deltaT * 0.5f; 
	float thetaMagSq = squared_length(theta); 
	float s; 
	if(thetaMagSq * thetaMagSq / 24.0f < EPSILON) { 
		deltaQ.w = 1.0f - thetaMagSq / 2.0f; 
		s = 1.0f - thetaMagSq / 6.0f; 
	} 
	else { 
		float thetaMag = sqrt(thetaMagSq); 
		deltaQ.w = cos(thetaMag); 
		s = sin(thetaMag) / thetaMag; 
	} 
	deltaQ.x = theta.x * s; 
	deltaQ.y = theta.y * s; 
	deltaQ.z = theta.z * s; 
	return normalize(deltaQ * q); 
}

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

void Physics::step( real_t dt )
{
// printf("sphere %d position: %f %f %f \n", 0, spheres[0]->position[0], spheres[0]->position[1], spheres[0]->position[2]);
	for (unsigned int i = 0; i < spheres.size(); i++) {
		if (spheres[i]->velocity != Vector3::Zero()) {
			// sphere to sphere
			for (unsigned int j = 0; j < spheres.size(); j++) {
				if (i != j) {
					collides(*spheres[i], *spheres[j], collision_damping);
				}
			}
			// sphere to plane
			for (unsigned int j = 0; j < planes.size(); j++) {
				collides(*spheres[i], *planes[j], collision_damping);
			}
			// sphere to triangle
			for (unsigned int j = 0; j < triangles.size(); j++) {
				collides(*spheres[i], *triangles[j], collision_damping);
			}
		}
	}
	updateStates(dt);
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    
    gravity = Vector3::Zero();
	collision_damping = 0.0;
}

void inline Physics::updateStates(real_t dt) {

	/************************************** Eular *****************************************************/
	if (MODE == 0) {
		// update force and torque
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->velocity += (gravity + spheres[i]->force / spheres[i]->mass) * dt;
			spheres[i]->position += spheres[i]->velocity * dt;	
			// update orientation
			spheres[i]->angular_velocity += (spheres[i]->torque / spheres[i]->getInertia()) * dt;
			spheres[i]->orientation = quatIntegrate(spheres[i]->orientation, spheres[i]->angular_velocity, dt);
			// apply changes to body
			spheres[i]->step_position(dt, 0);
			spheres[i]->step_orientation(dt, 0);
		}
		

	}

	/************************************** Leap Frog *************************************************/
	if (MODE == 1) {

		// Leap frog integration
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->velocity += 0.5 * (gravity + spheres[i]->force / spheres[i]->mass) * dt;
			spheres[i]->position += 0.5 * spheres[i]->velocity * dt;
			// update orientation
			spheres[i]->angular_velocity += 0.5 * (spheres[i]->torque / spheres[i]->getInertia()) * dt;
			spheres[i]->orientation = quatIntegrate(spheres[i]->orientation, 0.5 * spheres[i]->angular_velocity, 0.5 * dt);

		}
		// update force and torque
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->velocity += 0.5 * (gravity + spheres[i]->force / spheres[i]->mass) * dt;
			// update orientation
			spheres[i]->angular_velocity += 0.5 * (spheres[i]->torque / spheres[i]->getInertia()) * dt;
			// apply changes to body
			spheres[i]->step_position(dt, 0);
			spheres[i]->step_orientation(dt, 0);
		}
	}
	
	/******************************************* RK4 **************************************************/
	if (MODE == 2) {	
		// step 1
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->x[0] = spheres[i]->position;
			spheres[i]->v[0] = spheres[i]->velocity;
			spheres[i]->a[0] = gravity + spheres[i]->force / spheres[i]->mass;
			spheres[i]->position = spheres[i]->x[0] + 0.5 * spheres[i]->v[0] * dt;
			spheres[i]->velocity = spheres[i]->v[0] + 0.5 * spheres[i]->a[0] * dt;
			// update orientation
			spheres[i]->q[0] = spheres[i]->orientation;
			spheres[i]->omega[0] = spheres[i]->angular_velocity;
			spheres[i]->alpha[0] = spheres[i]->torque / spheres[i]->getInertia();
			spheres[i]->orientation = quatIntegrate(spheres[i]->q[0], 0.5 * spheres[i]->omega[0], 0.5 * dt);
			spheres[i]->angular_velocity = spheres[i]->omega[0] + 0.5 * spheres[i]->alpha[0] * dt;
		}
		// step 2
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->x[1] = spheres[i]->position;
			spheres[i]->v[1] = spheres[i]->velocity;
			spheres[i]->a[1] = gravity + spheres[i]->force / spheres[i]->mass;
			spheres[i]->position = spheres[i]->x[0] + 0.5 * spheres[i]->v[1] * dt;
			spheres[i]->velocity = spheres[i]->v[0] + 0.5 * spheres[i]->a[1] * dt;
			// update orientation
			spheres[i]->q[1] = spheres[i]->orientation;
			spheres[i]->omega[1] = spheres[i]->angular_velocity;
			spheres[i]->alpha[1] = spheres[i]->torque / spheres[i]->getInertia();
			spheres[i]->orientation = quatIntegrate(spheres[i]->q[0], 0.5 * spheres[i]->omega[1], 0.5 * dt);
			spheres[i]->angular_velocity = spheres[i]->omega[0] + 0.5 * spheres[i]->alpha[1] * dt;
		}
		// step 3
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->x[2] = spheres[i]->position;
			spheres[i]->v[2] = spheres[i]->velocity;
			spheres[i]->a[2] = gravity + spheres[i]->force / spheres[i]->mass;
			spheres[i]->position = spheres[i]->x[0] + spheres[i]->v[2] * dt;
			spheres[i]->velocity = spheres[i]->v[0] + spheres[i]->a[2] * dt;
			// update orientation
			spheres[i]->q[2] = spheres[i]->orientation;
			spheres[i]->omega[2] = spheres[i]->angular_velocity;
			spheres[i]->alpha[2] = spheres[i]->torque / spheres[i]->getInertia();
			spheres[i]->orientation = quatIntegrate(spheres[i]->q[0], spheres[i]->omega[2], dt);
			spheres[i]->angular_velocity = spheres[i]->omega[0] + spheres[i]->alpha[2] * dt;
		}
		// step 4
		for (unsigned int i = 0; i < springs.size(); i++) {
			springs[i]->step(dt);
		}
		for (unsigned int i = 0; i < spheres.size(); i++) {
			// update position
			spheres[i]->x[3] = spheres[i]->position;
			spheres[i]->v[3] = spheres[i]->velocity;
			spheres[i]->a[3] = gravity + spheres[i]->force / spheres[i]->mass;
			spheres[i]->position = spheres[i]->x[0] + (dt / 6.0) * 
				(spheres[i]->v[0] + 2.0 * spheres[i]->v[1] + 2.0 * spheres[i]->v[2] + spheres[i]->v[3]);
			spheres[i]->velocity = spheres[i]->v[0] + (dt / 6.0) * 
				(spheres[i]->a[0] + 2.0 * spheres[i]->a[1] + 2.0 * spheres[i]->a[2] + spheres[i]->a[3]);
			// updata orientation
			spheres[i]->q[3] = spheres[i]->orientation;
			spheres[i]->omega[3] = spheres[i]->angular_velocity;
			spheres[i]->alpha[3] = spheres[i]->torque / spheres[i]->getInertia();
			spheres[i]->orientation = quatIntegrate(spheres[i]->q[0], (1.0 / 6.0) * 
				(spheres[i]->omega[0] + 2.0 * spheres[i]->omega[1] + 2.0 * spheres[i]->omega[2] + spheres[i]->omega[3]), dt);
			spheres[i]->angular_velocity = spheres[i]->omega[0] + (dt / 6.0) *
				(spheres[i]->alpha[0] + 2.0 * spheres[i]->alpha[1] + 2.0 * spheres[i]->alpha[2] + spheres[i]->alpha[3]);
			// apply changes to body
			spheres[i]->step_position(dt, 0);
			spheres[i]->step_orientation(dt, 0);
		}
	}
}

}
