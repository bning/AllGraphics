/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
    intersection.time = INFINITY;
std::cout << "A triangle\n";
}

Triangle::~Triangle() { }

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

bool Triangle::checkIntersection(Ray ray) {

	const real_t epsilon = 0.001;
	Vector3 edge1, edge2;
	Vector3 P, Q, T;
	Vector3 O, D;
	real_t det, inv_det, u, v;
	real_t t;

	O = invMat.transform_point(ray.e);
	D = normalize(invMat.transform_vector(ray.d));

	edge1 = vertices[1].position - vertices[0].position;
	edge2 = vertices[2].position - vertices[0].position;

	P = cross(D, edge2);
	det = dot(edge1, P);
	if (det > -epsilon  && det < 0) {
		return false;
	}

	inv_det = 1.0 / det;
	T = O - vertices[0].position;
	u = dot(T, P) * inv_det;
	if (u < 0.0 || u > 1.0) {
		return false;
	}

	Q = cross(T, edge1);
	v = dot(D, Q) * inv_det;
	if (v < 0.0 || u + v > 1.0) {
		return false;
	}

	t = dot(edge2, Q) * inv_det;
	if (t > epsilon) {
		intersection.time = t;
		intersection.alpha = 1 - u - v;
		intersection.beta = u;
		intersection.gama = v;
		return true;
	}

	return false;
}

real_t Triangle::getIntersectionTime() {
	return intersection.time;
}

Vector3 Triangle::getIntersectionNormal() {
	Vector3 v0_norm = vertices[0].normal;
	Vector3 v1_norm = vertices[1].normal;
	Vector3 v2_norm = vertices[2].normal;
	return normalize(normMat * 
		(intersection.alpha * v0_norm + 
		 intersection.beta * v1_norm + 
		 intersection.gama * v2_norm));
}

Vector3 Triangle::getIntersectionPosition(Ray ray) {
	return ray.e + intersection.time * ray.d;
}

Color3 Triangle::getIntersectionTextureColor(Ray ray) {
	Vector2 v0_tex_coord = vertices[0].tex_coord;
	Vector2 v1_tex_coord = vertices[1].tex_coord;
	Vector2 v2_tex_coord = vertices[2].tex_coord;
	Vector2 intersect_tex_coord =
		intersection.alpha * v0_tex_coord +
		intersection.beta * v1_tex_coord +
		intersection.gama * v2_tex_coord;
	int texWidth;
	int texHeight;
	int pxl_x;
	int pxl_y;
	vertices[0].material->get_texture_size(&texWidth, &texHeight);
	pxl_x = (intersect_tex_coord[0] - (int)intersect_tex_coord[0]) * texWidth;
	pxl_y = (intersect_tex_coord[1] - (int)intersect_tex_coord[1]) * texHeight;
	Color3 v0_tex_pxl = vertices[0].material->get_texture_pixel(pxl_x, pxl_y);
	vertices[1].material->get_texture_size(&texWidth, &texHeight);
	pxl_x = (intersect_tex_coord[0] - (int)intersect_tex_coord[0]) * texWidth;
	pxl_y = (intersect_tex_coord[1] - (int)intersect_tex_coord[1]) * texHeight;
	Color3 v1_tex_pxl = vertices[1].material->get_texture_pixel(pxl_x, pxl_y);
	vertices[2].material->get_texture_size(&texWidth, &texHeight);
	pxl_x = (intersect_tex_coord[0] - (int)intersect_tex_coord[0]) * texWidth;
	pxl_y = (intersect_tex_coord[1] - (int)intersect_tex_coord[1]) * texHeight;
	Color3 v2_tex_pxl = vertices[2].material->get_texture_pixel(pxl_x, pxl_y);
	return intersection.alpha * v0_tex_pxl + intersection.beta * v1_tex_pxl + intersection.gama * v2_tex_pxl;
}

Color3 Triangle::getIntersectionAmbientColor(){
	Color3 v0_amb_col = vertices[0].material->ambient;
	Color3 v1_amb_col = vertices[1].material->ambient;
	Color3 v2_amb_col = vertices[2].material->ambient;
	return intersection.alpha * v0_amb_col + intersection.beta * v1_amb_col + intersection.gama * v2_amb_col;
}

Color3 Triangle::getIntersectionDiffuseColor() {
	Color3 v0_dif_col = vertices[0].material->diffuse;
	Color3 v1_dif_col = vertices[1].material->diffuse;
	Color3 v2_dif_col = vertices[2].material->diffuse;
	return intersection.alpha * v0_dif_col + intersection.beta * v1_dif_col + intersection.gama * v2_dif_col;
}

Color3 Triangle::getIntersectionSpecularColor() {
	Color3 v0_spec_col = vertices[0].material->specular;
	Color3 v1_spec_col = vertices[1].material->specular;
	Color3 v2_spec_col = vertices[2].material->specular;
	return intersection.alpha * v0_spec_col + intersection.beta * v1_spec_col + intersection.gama * v2_spec_col;
}

real_t Triangle::getIntersectionRefIndex() {
	real_t v0_ref_ind = vertices[0].material->refractive_index;
	real_t v1_ref_ind = vertices[1].material->refractive_index;
	real_t v2_ref_ind = vertices[2].material->refractive_index;
	return intersection.alpha * v0_ref_ind + intersection.beta * v1_ref_ind + intersection.gama * v2_ref_ind;

}

} /* _462 */
