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

Color3 get_tex_color(const Vector3& bary, const Material& mat, const Vector2& tx0,
							const Vector2& tx1, const Vector2& tx2)
{
	int width, height;
	mat.get_texture_size(&width, &height);

	if (width == 0 || height == 0) return Color3::White();

	Vector2 texcoord = lerp(bary, tx0, tx1, tx2);

	int x = texcoord.x*width;
	int y = texcoord.y*height;
	x = x % width;
	y = y % height;
	if (x < 0) x += width;
	if (y < 0) y += height;

	return mat.get_texture_pixel(x, y);
}

void Triangle::get_extended_info(Intersection* info)
{
	Vector3 dir = info->normal;
	info->normal = lerp(info->bary, vertices[0].normal,
						vertices[1].normal, vertices[2].normal);
	if (dot(dir, info->normal) > 0.0) info->normal = -info->normal;
	    
	info->diffuse = lerp(info->bary, vertices[0].material->diffuse,
						 vertices[1].material->diffuse, vertices[2].material->diffuse);
	info->ambient = lerp(info->bary, vertices[0].material->ambient,
						 vertices[1].material->ambient, vertices[2].material->ambient);
	info->specular = lerp(info->bary, vertices[0].material->specular,
						 vertices[1].material->specular, vertices[2].material->specular);
	info->refractive_index = lerp(info->bary, vertices[0].material->refractive_index,
	vertices[1].material->refractive_index, vertices[2].material->refractive_index);

	const Vector2& tx0 = vertices[0].tex_coord;
	const Vector2& tx1 = vertices[1].tex_coord;
	const Vector2& tx2 = vertices[2].tex_coord;

	Color3 c0 = get_tex_color(info->bary, *vertices[0].material, tx0, tx1, tx2);
	
	Color3 c1 = get_tex_color(info->bary, *vertices[1].material, tx0, tx1, tx2);
	Color3 c2 = get_tex_color(info->bary, *vertices[2].material, tx0, tx1, tx2);
	
	info->texColor = lerp(info->bary, c0, c1, c2);

}

bool triangle_intersect(const Ray& r, const Vector3& v0, const Vector3& v1, const Vector3& v2, Intersection* info)
{
	Vector3 e1 = v1 - v0;
	Vector3 e2 = v2 - v0;
	Vector3 h = cross(r.d, e2);

	real_t a = dot(e1, h);
	if (std::abs(a) < 0.00001)
		return false;

	real_t f = 1 / a;
	Vector3 s = r.e - v0;
	real_t u = f * dot(s, h);
	if (u < 0.0 || u > 1.0)
	return false ;

	Vector3 q = cross(s, e1);
	real_t v = f * dot(r.d, q);
	if (v < 0.0 || u + v > 1.0)
	return false;

	real_t time = f * dot(e2, q);
	if (time < 0.00001)
	return false;

	info->t = time;
	info->bary.x = 1 - u - v;
	info->bary.y = u;
	info->bary.z = v;
	return true;
}

bool Triangle::intersect(const Ray& r, Intersection* info)
{
	const Vector3& v0 = vertices[0].position;
	const Vector3& v1 = vertices[1].position;
	const Vector3& v2 = vertices[2].position;
	info->normal = r.d;
	return triangle_intersect(r, v0, v1, v2, info);

}

} /* _462 */
