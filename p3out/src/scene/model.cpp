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

Model::Model() : mesh( 0 ), material( 0 ) { }
Model::~Model() { }

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

bool Model::intersect(const Ray& r, Intersection* info)
{
	unsigned int N = mesh->num_triangles();
	const MeshVertex* verts = mesh->get_vertices();
	Intersection tmpinfo;
	info->t = DBL_MAX;
	info->idx = -1;
	for (unsigned int i = 0; i < N; i++)
	{
		const unsigned int* vIdxs = mesh->triangles[i].vertices;
		if (triangle_intersect(r, verts[vIdxs[0]].position, verts[vIdxs[1]].position, verts[vIdxs[2]].position, &tmpinfo))
		{
			if (tmpinfo.t < info->t)
			{
				*info = tmpinfo;
				info->idx = i;
			}
		}
	}
	return info->idx != -1;
}

void Model::get_extended_info(Intersection* info)
{
	const unsigned int* vIdxs = mesh->triangles[info->idx].vertices;

	info->ambient = material->ambient;
	info->diffuse = material->diffuse;
	info->specular = material->specular;
	info->refractive_index = material->refractive_index;

	const Vector2& tx0 = mesh->vertices[vIdxs[0]].tex_coord;
	const Vector2& tx1 = mesh->vertices[vIdxs[1]].tex_coord;
	const Vector2& tx2 = mesh->vertices[vIdxs[2]].tex_coord;

	info->texColor = get_tex_color(info->bary, *material, tx0, tx1, tx2);

	const Vector3& n0 = mesh->vertices[vIdxs[0]].normal;
	const Vector3& n1 = mesh->vertices[vIdxs[1]].normal;
	const Vector3& n2 = mesh->vertices[vIdxs[2]].normal;

	info->normal = lerp(info->bary, n0, n1, n2);


}

} /* _462 */
