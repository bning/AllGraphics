/**
 * @file raytacer.hpp
 * @brief Raytracer class
 *
 * Implement these functions for project 3.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_RAYTRACER_HPP_
#define _462_RAYTRACER_HPP_

#define MAX_DEPTH 5

#include "math/color.hpp"
#include "math/random462.hpp"
#include "scene/scene.hpp"


namespace _462 {

class Scene;
class Ray;
struct Intersection;
class Raytracer
{
public:

    Raytracer();

    ~Raytracer();

    bool initialize(Scene* scene, size_t num_samples,
                    size_t width, size_t height);

    bool raytrace(unsigned char* buffer, real_t* max_time);

private:

    Color3 trace_pixel(const Scene* scene,
		       size_t x,
		       size_t y,
		       size_t width,
		       size_t height);

    // the scene to trace
    Scene* scene;

    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;

    unsigned int num_samples;

	// Number of light rays in sampling
	const unsigned int NUM_LIGHT_SAMPLE = 10;

	const Vector3 EPSILON = Vector3(1e-10, 1e-10, 1e-10);

	// Get the index of the intersected geometry
	int getIntersectionGeometryIndex(Ray r);
	// Check whether a ray intersects with a light
	bool checkLightIntersection(Ray r, int lightInd, real_t* t);
	// Get the intersected light index
	int getIntersectionLightIndex(Ray r);
	// Get the direct illumination color
	Color3 getLightsColor(Vector3 interPos, Vector3 interNorm, Color3 diffuseColor);
	// Recursive function
	Color3 tracyOneRay(Ray r);
	// Get the reflect direction of the given direction and norm
	Vector3 getReflectDirection(Vector3 d, Vector3 n);
	// Get the refrection direction of the given direction, norm, n_d and n_r
	Vector3 getRefractionDirection(Vector3 d, Vector3 n, real_t n_d, real_t n_r);
	// Get the parameter R for fractioning refraction and reflection
	real_t getR(real_t denserN, Vector3 norm, Vector3 largerD);
	// Check whether a light ray is directed to the light
	bool checkDirectToLight(Ray lightRay, real_t time_ray_light);
};

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
