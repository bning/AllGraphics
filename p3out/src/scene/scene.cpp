/**
 * @file scene.cpp
 * @brief Function definitions for scenes.
 *
 * @author Eric Butler (edbutler)
 * @author Kristin Siu (kasiu)
 */

#include "scene/scene.hpp"

namespace _462 {


Geometry::Geometry() :
	position(Vector3::Zero()),
	orientation(Quaternion::Identity()),
	scale(Vector3::Ones())
{

}

Geometry::~Geometry() { }

bool Geometry::initialize()
{
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	Matrix4 mat;
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);

	return true;
}

bool SphereLight::initialize()
{

	//std::cout << position.x << std::endl;
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	//Matrix4 mat;
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);
	return true;
}

SphereLight::SphereLight() :
	position(Vector3::Zero()),
	color(Color3::White()),
	radius(real_t(0)),
	orientation(Quaternion::Identity()),
	scale(Vector3::Ones())
{
	attenuation.constant = 1;
	attenuation.linear = 0;
	attenuation.quadratic = 0;

}

bool solveQuadratic(const real_t &a, const real_t &b, const real_t &c, real_t &x0, real_t &x1) { real_t discr = b * b - 4 * a * c; if (discr < 0) return false; else if (discr == 0) x0 = x1 = -0.5 * b / a; else { real_t q = (b > 0) ? -0.5 * (b + sqrt(discr)) : -0.5 * (b - sqrt(discr)); x0 = q / a; x1 = c / q; } if (x0 > x1) std::swap(x0, x1); return true; }

bool SphereLight::intersect(const Ray& tr, real_t& t) const
{
	Ray r;
	r.d = invMat.transform_vector(tr.d);
	r.e = invMat.transform_point(tr.e);

	//std::cout << r.e.x << " " << tr.e.x << std::endl;

	real_t a = squared_length(r.d);
	real_t b = real_t(2)*dot(r.d, r.e);
	real_t c = squared_length(r.e) - radius * radius;
	real_t disc = b*b - real_t(4)*a*c;

	if (disc < real_t(0)) return false;

	int df = b < 0 ? -1 : 1;
	real_t q = (-b + df*sqrt(disc)) / real_t(2);

	real_t t0 = q / a;
	real_t t1 = c / q;

	if (t0 > t1) std::swap(t0, t1);

	if (t1 < real_t(0)) return false;

	t = t0 > real_t(0) ? t0 : t1;
	//info->normal = r.d*(info->t+1e-9) + r.e;
	//info->normal = normalize(normMat*info->normal);

	return true;
}

void Geometry::get_extended_info(Intersection* info)
{}

Scene::Scene()
{
	reset();
}

Scene::~Scene()
{
	reset();
}

bool Scene::initialize()
{
	bool res = true;
	for (unsigned int i = 0; i < num_geometries(); i++)
		res &= geometries[i]->initialize();
	for (unsigned int i = 0; i < num_lights(); i++)
		res &= point_lights[i].initialize();
	return res;
}

bool Scene::intersect(const Ray& r, Intersection* info)
{
	info->t = DBL_MAX;
	Intersection tmpinfo;
	Ray tr(r);
	int objIdx = -1;
	for (unsigned int i = 0; i < num_geometries(); i++)
	{
		tr.d = (geometries[i]->invMat.transform_vector(r.d));
		tr.e = geometries[i]->invMat.transform_point(r.e);

		if (geometries[i]->intersect(tr, &tmpinfo) && tmpinfo.t < info->t)
		{
			*info = tmpinfo;
			objIdx = i;
		}
	}
	if (objIdx != -1)
	{
		info->pos = r.d*(info->t + 1e-9) + r.e;
		geometries[objIdx]->get_extended_info(info);
		info->normal = normalize(geometries[objIdx]->normMat*info->normal);
		return true;
	}

	return false;
}

real_t Scene::intersect_lights(const Ray& r, unsigned int& idx)
{
	unsigned int N = num_lights();
	const SphereLight* lights = get_lights();

	real_t mint = DBL_MAX;

	for (unsigned int i = 0; i < N; i++)
	{
		real_t tmpt = DBL_MAX;
		if (lights[i].intersect(r, tmpt) && tmpt < mint)
		{
			mint = tmpt;
			idx = i;
		}
	}

	return mint;
}

Geometry* const* Scene::get_geometries() const
{
	return geometries.empty() ? NULL : &geometries[0];
}

size_t Scene::num_geometries() const
{
	return geometries.size();
}

const SphereLight* Scene::get_lights() const
{
	return point_lights.empty() ? NULL : &point_lights[0];
}

size_t Scene::num_lights() const
{
	return point_lights.size();
}

Material* const* Scene::get_materials() const
{
	return materials.empty() ? NULL : &materials[0];
}

size_t Scene::num_materials() const
{
	return materials.size();
}

Mesh* const* Scene::get_meshes() const
{
	return meshes.empty() ? NULL : &meshes[0];
}

size_t Scene::num_meshes() const
{
	return meshes.size();
}

void Scene::reset()
{
	for (GeometryList::iterator i = geometries.begin(); i != geometries.end(); ++i) {
		delete *i;
	}
	for (MaterialList::iterator i = materials.begin(); i != materials.end(); ++i) {
		delete *i;
	}
	for (MeshList::iterator i = meshes.begin(); i != meshes.end(); ++i) {
		delete *i;
	}

	geometries.clear();
	materials.clear();
	meshes.clear();
	point_lights.clear();

	camera = Camera();

	background_color = Color3::Black();
	ambient_light = Color3::Black();
	refractive_index = 1.0;
}

void Scene::add_geometry(Geometry* g)
{
	geometries.push_back(g);
}

void Scene::add_material(Material* m)
{
	materials.push_back(m);
}

void Scene::add_mesh(Mesh* m)
{
	meshes.push_back(m);
}

void Scene::add_light(const SphereLight& l)
{
	point_lights.push_back(l);
}


} /* _462 */
