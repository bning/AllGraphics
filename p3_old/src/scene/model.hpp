/**
 * @file model.hpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_MODEL_HPP_
#define _462_SCENE_MODEL_HPP_

#include "scene/scene.hpp"
#include "scene/mesh.hpp"

namespace _462 {

/**
 * A mesh of triangles.
 */
class Model : public Geometry
{
public:

    struct Intersection {
	real_t time;
	real_t alpha;
	real_t beta;
	real_t gama;
	int index;
    };

    const Mesh* mesh;
    const Material* material;
	Intersection intersection;

    // Bounding Volume of the geometry
    struct BoundingVolume{
	Vector3 localCenter;
	real_t localRadius;
    };

    BoundingVolume bound;

    Model();
    virtual ~Model();
    virtual bool initialize();
    virtual bool checkIntersection(Ray ray);
    virtual real_t getIntersectionTime();
    virtual Vector3 getIntersectionNormal();
    virtual Vector3 getIntersectionPosition(Ray ray);
    virtual Color3 getIntersectionTextureColor(Ray ray);
    virtual Color3 getIntersectionAmbientColor();
    virtual Color3 getIntersectionDiffuseColor();
    virtual Color3 getIntersectionSpecularColor();
    virtual real_t getIntersectionRefIndex();
    virtual void render() const;


private:
    bool checkTriangleIntersection(Vector3 local_e, Vector3 local_d, 
	Vector3 vert0, Vector3 vert1, Vector3 vert2, 
	real_t* t, real_t* u, real_t* v);
};



} /* _462 */

#endif /* _462_SCENE_MODEL_HPP_ */

