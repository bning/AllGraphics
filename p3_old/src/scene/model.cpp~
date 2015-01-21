/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ) {
	intersection.time = INFINITY;
	intersection.alpha = INFINITY;
	intersection.beta = INFINITY;
	intersection.gama = INFINITY;
	intersection.index = -1;
}
Model::~Model() {}

bool Model::initialize() {
	// initialize as the super class does
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	Matrix4 mat;
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);
	// Add bounding volume to the model
	real_t max_x, min_x, max_y, min_y, max_z, min_z;
	max_x = max_y = max_z = 0;
	min_x = min_y = min_z = INFINITY;
	for (unsigned int i = 0; i < mesh->vertices.size(); i++) {
		// max x
		max_x = std::max(mesh->vertices[i].position.x, max_x);
		// min x
		min_x = std::min(mesh->vertices[i].position.x, min_x);
		// max y
		max_y = std::max(mesh->vertices[i].position.y, max_y);
		// min y
		min_y = std::min(mesh->vertices[i].position.y, min_y);
		// max z
		max_z = std::max(mesh->vertices[i].position.z, max_z);
		// min z
		min_z = std::min(mesh->vertices[i].position.z, min_z);
	}
	bound.localCenter = 0.5 * Vector3(max_x + min_x, max_y + min_y, max_z + min_z);
	bound.localRadius = 0.5 * length(Vector3(max_x - min_x, max_y - min_y, max_z - min_z));
std::cout << "A model\n";
	return true;
}

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}

bool Model::checkIntersection(Ray ray) {
	
	Vector3 local_e = invMat.transform_point(ray.e); // (invMat * Vector4(ray.e, 1)).xyz();
	Vector3 local_d = normalize(invMat.transform_vector(ray.d)); // normalize((invMat * Vector4(ray.d, 1)).xyz());
	// Check bounding sphere intersection
	real_t det = pow(dot(local_d, local_e - bound.localCenter), 2) -
		dot(local_d, local_d) * (dot(local_e - bound.localCenter, local_e - bound.localCenter) - pow(bound.localRadius, 2));
	if (det < 0) {
	}
	if (ray.inAir) {
		// Check each mesh triangles
		real_t t = INFINITY;
		real_t inter_t;
		int interIndex = -1;
		real_t u;
		real_t v;
		for (unsigned int i = 0; i < mesh->num_triangles(); i++) {
			Vector3 vert0 = mesh->vertices[mesh->triangles[i].vertices[0]].position;
			Vector3 vert1 = mesh->vertices[mesh->triangles[i].vertices[1]].position;
			Vector3 vert2 = mesh->vertices[mesh->triangles[i].vertices[2]].position;
			// if intersects with a triangle and the time is less than the last one, 
			// update the time to the new one
			if (checkTriangleIntersection(local_e, local_d, vert0, vert1, vert2, &inter_t, &u, &v) &&
				inter_t < t) {
				t = inter_t;
				intersection.alpha = 1 - u - v;
				intersection.beta = u;
				intersection.gama = v;
				interIndex = i;
			}
		}
		// No triangle intersected with
		if (interIndex < 0) {
			return false;
		}
		intersection.time = t;
		intersection.index = interIndex;
		return true;
	}
	else {
		// Check each mesh triangles
		real_t t = INFINITY;
		real_t inter_t;
		int interIndex = -1;
		real_t u;
		real_t v;
		for (unsigned int i = 0; i < mesh->num_triangles(); i++) {
			Vector3 vert0 = mesh->vertices[mesh->triangles[i].vertices[0]].position;
			Vector3 vert1 = mesh->vertices[mesh->triangles[i].vertices[1]].position;
			Vector3 vert2 = mesh->vertices[mesh->triangles[i].vertices[2]].position;
			// if intersects with a triangle and the time is less than the last one, 
			// update the time to the new one
			if (checkTriangleIntersection(local_e, local_d, vert0, vert1, vert2, &inter_t, &u, &v) &&
				inter_t < t && inter_t > 0.001) {
				t = inter_t;
				intersection.alpha = 1 - u - v;
				intersection.beta = u;
				intersection.gama = v;
				interIndex = i;
			}
		}
		intersection.time = t;
		intersection.index = interIndex;
		return true;	
	}
}

real_t Model::getIntersectionTime() {
	return intersection.time;
}

Vector3 Model::getIntersectionNormal() {
	int v0_ind = mesh->triangles[intersection.index].vertices[0];
	int v1_ind = mesh->triangles[intersection.index].vertices[1];
	int v2_ind = mesh->triangles[intersection.index].vertices[2];
	Vector3 v0_norm = mesh->vertices[v0_ind].normal;
	Vector3 v1_norm = mesh->vertices[v1_ind].normal;
	Vector3 v2_norm = mesh->vertices[v2_ind].normal;
	return normalize(normMat *
		(intersection.alpha * v0_norm +
		intersection.beta * v1_norm +
		intersection.gama * v2_norm));
}

Vector3 Model::getIntersectionPosition(Ray ray) {
	return ray.e + intersection.time * ray.d;
}

Color3 Model::getIntersectionTextureColor(Ray ray) {
	int v0_ind = mesh->triangles[intersection.index].vertices[0];
	int v1_ind = mesh->triangles[intersection.index].vertices[1];
	int v2_ind = mesh->triangles[intersection.index].vertices[2];
	Vector2 v0_tex_coord = mesh->vertices[v0_ind].tex_coord;
	Vector2 v1_tex_coord = mesh->vertices[v1_ind].tex_coord;
	Vector2 v2_tex_coord = mesh->vertices[v2_ind].tex_coord;
	Vector2 intersect_tex_coord =
		intersection.alpha * v0_tex_coord +
		intersection.beta * v1_tex_coord +
		intersection.gama * v2_tex_coord;
	int texWidth;
	int texHeight;
	material->get_texture_size(&texWidth, &texHeight);
	int pxl_x = (intersect_tex_coord[0] - (int)intersect_tex_coord[0]) * texWidth;
	int pxl_y = (intersect_tex_coord[1] - (int)intersect_tex_coord[1]) * texHeight;
	return material->get_texture_pixel(pxl_x, pxl_y);
}

Color3 Model::getIntersectionAmbientColor() {
	return material->ambient;
}

Color3 Model::getIntersectionDiffuseColor() {
	return material->diffuse;
}

Color3 Model::getIntersectionSpecularColor() {
	return material->specular;
}

real_t Model::getIntersectionRefIndex() {
	return material->refractive_index;
}

bool Model::checkTriangleIntersection(Vector3 local_e, Vector3 local_d,
	Vector3 vert0, Vector3 vert1, Vector3 vert2,
	real_t* t, real_t* u, real_t* v) {
	
	const real_t epsilon = 1e-100;
	Vector3 edge1, edge2;
	Vector3 P, Q, T;
	real_t det, inv_det;

	edge1 = vert1 - vert0;
	edge2 = vert2 - vert0;

	P = cross(local_d, edge2);
	det = dot(edge1, P);
	// if (det > - epsilon && det < epsilon) {
	if (det < epsilon) {
		return false;
	}

	inv_det = 1.0 / det;
	T = local_e - vert0;
	*u = dot(T, P) * inv_det;
	if (*u < 0.0 || *u > 1.0) {
		return false;
	}

	Q = cross(T, edge1);
	*v = dot(local_d, Q) * inv_det;
	if (*v < 0.0 || *u + *v > 1.0) {
		return false;
	}

	*t = dot(edge2, Q) * inv_det;
	if (*t > epsilon) {
		return true;
	}
	return false;
}

} /* _462 */
