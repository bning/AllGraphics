/**
 * @file triangle.hpp
 * @brief Class definition for Triangle.
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_TRIANGLE_HPP_
#define _462_SCENE_TRIANGLE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * a triangle geometry.
 * Triangles consist of 3 vertices. Each vertex has its own position, normal,
 * texture coordinate, and material. These should all be interpolated to get
 * values in the middle of the triangle.
 * These values are all in local space, so it must still be transformed by
 * the Geometry's position, orientation, and scale.
 */
class Triangle : public Geometry
{
public:

    struct Intersection {
	real_t time;
	real_t alpha;
	real_t beta;
	real_t gama;
    };

    struct Vertex
    {
        // note that position and normal are in local space
        Vector3 position;
        Vector3 normal;
        Vector2 tex_coord;
        const Material* material;
    };

    // the triangle's vertices, in CCW order
    Vertex vertices[3];
    Intersection intersection;

    Triangle();
    virtual ~Triangle();
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

#endif /* _462_SCENE_TRIANGLE_HPP_ */
