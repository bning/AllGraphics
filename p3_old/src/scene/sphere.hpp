/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    struct Intersection {
	Vector3 normal;
	real_t time;
    };

    real_t radius;
    const Material* material;
	Intersection intersection;

    Sphere();
    virtual ~Sphere();
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

};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

