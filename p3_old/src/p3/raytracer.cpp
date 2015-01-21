/**
 * @file raytacer.cpp
 * @brief Raytracer class
 *
 * Implement these functions for project 4.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "raytracer.hpp"
#include "scene/scene.hpp"

#include <SDL_timer.h>
#include <iostream>
#include <random>

#ifdef OPENMP // just a defense in case OpenMP is not installed.

#include <omp.h>

#endif
namespace _462 {

// max number of threads OpenMP can use. Change this if you like.
#define MAX_THREADS 8

static const unsigned STEP_SIZE = 8;

Raytracer::Raytracer()
    : scene(0), width(0), height(0) { }

// random real_t in [0, 1)
static inline real_t random()
{
    return real_t(rand())/RAND_MAX;
}

Raytracer::~Raytracer() { }

/**
 * Initializes the raytracer for the given scene. Overrides any previous
 * initializations. May be invoked before a previous raytrace completes.
 * @param scene The scene to raytrace.
 * @param width The width of the image being raytraced.
 * @param height The height of the image being raytraced.
 * @return true on success, false on error. The raytrace will abort if
 *  false is returned.
 */
bool Raytracer::initialize(Scene* scene, size_t num_samples,
			   size_t width, size_t height)
{
    /*
     * omp_set_num_threads sets the maximum number of threads OpenMP will
     * use at once.
     */
#ifdef OPENMP
    omp_set_num_threads(MAX_THREADS);
#endif
    this->scene = scene;
    this->num_samples = num_samples;
    this->width = width;
    this->height = height;

    current_row = 0;

    Ray::init(scene->camera);
    scene->initialize();

    const SphereLight* lights = scene->get_lights();

    return true;
}

/**
 * Performs a raytrace on the given pixel on the current scene.
 * The pixel is relative to the bottom-left corner of the image.
 * @param scene The scene to trace.
 * @param x The x-coordinate of the pixel to trace.
 * @param y The y-coordinate of the pixel to trace.
 * @param width The width of the screen in pixels.
 * @param height The height of the screen in pixels.
 * @return The color of that pixel in the final image.
 */
Color3 Raytracer::trace_pixel(const Scene* scene,
			      size_t x,
			      size_t y,
			      size_t width,
			      size_t height)
{
    assert(x < width);
    assert(y < height);

    real_t dx = real_t(1)/width;
    real_t dy = real_t(1)/height;

    Color3 res = Color3::Black();
    unsigned int iter;
    for (iter = 0; iter < num_samples; iter++)
    {
        // pick a point within the pixel boundaries to fire our
        // ray through.
        real_t i = real_t(2)*(real_t(x)+random())*dx - real_t(1);
        real_t j = real_t(2)*(real_t(y)+random())*dy - real_t(1);

        Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(i, j), true, -1, 0);

		res += tracyOneRay(r);
    }
    return res*(real_t(1)/num_samples);
}

/**
 * Raytraces some portion of the scene. Should raytrace for about
 * max_time duration and then return, even if the raytrace is not copmlete.
 * The results should be placed in the given buffer.
 * @param buffer The buffer into which to place the color data. It is
 *  32-bit RGBA (4 bytes per pixel), in row-major order.
 * @param max_time, If non-null, the maximum suggested time this
 *  function raytrace before returning, in seconds. If null, the raytrace
 *  should run to completion.
 * @return true if the raytrace is complete, false if there is more
 *  work to be done.
 */
bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time)
{
    static const size_t PRINT_INTERVAL = 64;

    // the time in milliseconds that we should stop
    unsigned int end_time = 0;
    bool is_done;

    if (max_time)
    {
        // convert duration to milliseconds
        unsigned int duration = (unsigned int) (*max_time * 1000);
        end_time = SDL_GetTicks() + duration;
    }

    // until time is up, run the raytrace. we render an entire group of
    // rows at once for simplicity and efficiency.
    for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE)
    {
        // we're done if we finish the last row
        is_done = current_row >= height;
        // break if we finish
        if (is_done) break;

        int loop_upper = std::min(current_row + STEP_SIZE, height);

        // This tells OpenMP that this loop can be parallelized.
#pragma omp parallel for
        for (int c_row = current_row; c_row < loop_upper; c_row++)
        {
            /*
             * This defines a critical region of code that should be
             * executed sequentially.
             */
#pragma omp critical
            {
                if (c_row % PRINT_INTERVAL == 0)
                    printf("Raytracing (Row %d)\n", c_row);
            }

            for (size_t x = 0; x < width; x++)
            {
                // trace a pixel
                Color3 color = trace_pixel(scene, x, c_row, width, height);
                // write the result to the buffer, always use 1.0 as the alpha
                color.to_array(&buffer[4 * (c_row * width + x)]);
            }
        }
    }

    if (is_done) printf("Done raytracing!\n");

    return is_done;
}


Color3 Raytracer::tracyOneRay(Ray ray) {
	// If recursion depth is greater than 3
	if (ray.numOfBounce > 3) {
		return scene->background_color;
	}
	// If the current ray is inside a geometry
	if (!ray.inAir) {
		// Ptr to geometry list
		Geometry* const* geomVec = scene->get_geometries();
		// Inside the geometry, check intersection with current geom
		if (!geomVec[ray.geomIndex]->checkIntersection(ray)) {
			printf("Ray is inside the geometry but cannot intersect with it!\n");
			return scene->background_color;
		}
		// Intersection norm
		Vector3 interNorm = geomVec[ray.geomIndex]->getIntersectionNormal();
		// Refraction index of current geometry
		real_t geomRefInd = geomVec[ray.geomIndex]->getIntersectionRefIndex();
		// Texture color 
		Color3 interTexCol = geomVec[ray.geomIndex]->getIntersectionTextureColor(ray);
		// Specular color
		Color3 interSpecCol = geomVec[ray.geomIndex]->getIntersectionSpecularColor();

		Vector3 new_e = geomVec[ray.geomIndex]->getIntersectionPosition(ray);
		Vector3 new_d = getRefractionDirection(ray.d, -interNorm, geomRefInd, scene->refractive_index);
		// Total internal reflection
		if (dot(new_d, interNorm) < 0) {
			Ray newRay = Ray(new_e, new_d, false, ray.geomIndex, ray.numOfBounce + 1);
			return interTexCol * interSpecCol * tracyOneRay(newRay);
		}
		// Shoot to the air
		else {
			// Ray refracted
			Ray newRay_refract = Ray(new_e, new_d, true, ray.geomIndex, ray.numOfBounce + 1);
			// Ray reflected
			Vector3 new_d_reflect = getReflectDirection(ray.d, -interNorm);
			Ray newRay_reflect = Ray(new_e, new_d_reflect, false, ray.geomIndex, ray.numOfBounce + 1);
			real_t R = getR(geomRefInd, interNorm, new_d);
			// Put both together
			return (1 - R) * tracyOneRay(newRay_refract) + 
				interTexCol * interSpecCol * R * tracyOneRay(newRay_reflect);
		}
	}
	// ELSE: The ray is in the air
	// Find the intersected geometry
	int currentGoemInd = getIntersectionGeometryIndex(ray);
	// If no geometries intersected
	if (currentGoemInd < 0) {
		int intersectLightIndex = getIntersectionLightIndex(ray);
		// If no light intersected
		// return background color
		if (intersectLightIndex < 0) {
			return scene->background_color;
		}
		// Light intersected
		// return light color
		return scene->get_lights()[intersectLightIndex].color;
	}
	// Intersected with a geometry
	// Ptr to geometry list
	ray.geomIndex = currentGoemInd;
	Geometry* const* geomVec = scene->get_geometries();
	// Intersection position
	Vector3 interPos = geomVec[ray.geomIndex]->getIntersectionPosition(ray);
	// Intersection normal
	Vector3 interNorm = geomVec[ray.geomIndex]->getIntersectionNormal();
	// If opaque geometry
	if (geomVec[ray.geomIndex]->getIntersectionRefIndex() == 0) {
		// Texture color at intersection
		Color3 interTexCol = geomVec[ray.geomIndex]->getIntersectionTextureColor(ray);
		// Ambient color at intersection
		Color3 interAmbCol = scene->ambient_light * geomVec[ray.geomIndex]->getIntersectionAmbientColor();
		// Specular color at intersection
		Color3 interSpecCol = geomVec[ray.geomIndex]->getIntersectionSpecularColor();
		// Diffuse color at intersection
		Color3 interDiffCol = geomVec[ray.geomIndex]->getIntersectionDiffuseColor();
		// All lights color, the SUM
		Color3 interLightsCol = getLightsColor(interPos, interNorm, interDiffCol);
		// Create the new reflected ray
		Vector3 new_e = interPos;
		Vector3 new_d = getReflectDirection(ray.d, interNorm);
		Ray newRay = Ray(new_e, new_d, true, ray.geomIndex, ray.numOfBounce + 1); 
		return interTexCol * (interAmbCol + interLightsCol) +
			interTexCol * interSpecCol * tracyOneRay(newRay); 
	}
	// Get into the geometry
	// Get refraction index
	real_t geomRefInd = geomVec[ray.geomIndex]->getIntersectionRefIndex();
	real_t airRefInd = scene->refractive_index;
	// Create new reflect ray
	Vector3 new_e_reflect = interPos;
	Vector3 new_d_reflect = getReflectDirection(ray.d, interNorm);
	Ray newRay_reflect = Ray(new_e_reflect, new_d_reflect, true, ray.geomIndex, ray.numOfBounce + 1);
	// Create new refract ray
	Vector3 new_e_refract = interPos;
	Vector3 new_d_refract = getRefractionDirection(ray.d, interNorm, airRefInd, geomRefInd);
	Ray newRay_refract = Ray(new_e_refract, new_d_refract, false, ray.geomIndex, ray.numOfBounce + 1);
	// Compute R
	real_t R = getR(geomRefInd, interNorm, new_d_reflect);
	// Color stuff
	Color3 interTexCol = geomVec[ray.geomIndex]->getIntersectionTextureColor(ray);
	Color3 interSpecCol = geomVec[ray.geomIndex]->getIntersectionSpecularColor();
	// Recursion Reflect + Refract
	return interTexCol * interSpecCol * R * tracyOneRay(newRay_reflect) + (1 - R) * tracyOneRay(newRay_refract); ///////R, 1-R
}

/*
 * Check whether the given ray intersects with the light of the given index in light list. If intersects, modify the intersection time
 */
bool Raytracer::checkLightIntersection(Ray r, int ind, real_t* t) {
	Vector3 position = scene->get_lights()[ind].position;
	real_t radius = scene->get_lights()[ind].radius;
	real_t det = pow(dot(r.d, r.e - position), 2) -
		dot(r.d, r.d) * (dot(r.e - position, r.e - position) - pow(radius, 2));
	if (det < 0) {
		return false;
	}
	*t = (-dot(r.d, r.e - position) - pow(det, 0.5)) / dot(r.d, r.d);
	return true;
}

/*
 * Get the nearest light that the given ray intersects with
 */
int Raytracer::getIntersectionLightIndex(Ray ray) {
	int index = -1;
	real_t min_time = INFINITY;
	real_t current_time = INFINITY;
	for (unsigned int i = 0; i < scene->num_lights(); i++) {
		if (checkLightIntersection(ray, i, &current_time) &&
			current_time < min_time) {
			min_time = current_time;
			index = i;
		}
	}
	return index;
}

/*
 * Get the index of the geometry that the given ray intersects with
 */
int Raytracer::getIntersectionGeometryIndex(Ray ray) {
	int index = -1;
	real_t min_time = INFINITY;
	for (unsigned int i = 0; i < scene->num_geometries(); i++) {
		if (ray.geomIndex == i) {
			continue;		
		}
		if (scene->get_geometries()[i]->checkIntersection(ray) &&
			scene->get_geometries()[i]->getIntersectionTime() < min_time) {
			min_time = scene->get_geometries()[i]->getIntersectionTime();
			index = i;
		}
	}
	return index;
}

/*
 * Check whether the given ray is directed to the given light
 */
bool Raytracer::checkDirectToLight(Ray lightRay, real_t time_ray_light) {
	for (unsigned int k = 0; k < scene->num_geometries(); k++) {
		if (scene->get_geometries()[k]->checkIntersection(lightRay) &&
			scene->get_geometries()[k]->getIntersectionTime() < time_ray_light) {
			return false;
		}
	}
	return true;
}

/*
 * Get the sum of the light colors in the given intersection position
 */
Color3 Raytracer::getLightsColor(Vector3 interPos, Vector3 interNorm, Color3 diff) {
	// Initialize to black
	Color3 lightsColor = Color3::Black();
	for (unsigned int i = 0; i < scene->num_lights(); i++) {
		Color3 eachLightsColor = Color3::Black();
		for (unsigned int j = 0; j < NUM_LIGHT_SAMPLE; j++) {
			Vector3 randomLightPoint = scene->get_lights()[i].position + 
				scene->get_lights()[i].radius * normalize(Vector3(random_gaussian(), random_gaussian(), random_gaussian()));
			Vector3 lightDirVec = normalize(randomLightPoint - interPos);
			Ray lightRay = Ray(interPos, lightDirVec, true, -1, 0);
			real_t time_ray_light = distance(randomLightPoint, interPos);
			if (!checkDirectToLight(lightRay, time_ray_light)) {
				 continue;
			}
			real_t dist = distance(randomLightPoint, interPos);
			Color3 c_i = scene->get_lights()[i].color * 
				(1.0 / (scene->get_lights()[i].attenuation.constant + 
				scene->get_lights()[i].attenuation.linear * dist + 
				scene->get_lights()[i].attenuation.quadratic * pow(dist, 2)));
			eachLightsColor += c_i * diff * std::max(dot(interNorm, lightDirVec), 0.0);
		}
		lightsColor += eachLightsColor * (1.0 / NUM_LIGHT_SAMPLE);
	}
	return lightsColor;
}

/*
 * Get the reflection direction of the given direction and norm
 */
Vector3 Raytracer::getReflectDirection(Vector3 d, Vector3 n) {
	return normalize(d - 2 * dot(d, n) * n);
}

/*
 * Get the refraction direction of the given direction, norm and refraction indices
 */
Vector3 Raytracer::getRefractionDirection(Vector3 d, Vector3 n, real_t n_d, real_t n_r) {
	real_t det = 1 - pow(n_d, 2) * (1 - pow(dot(d, n), 2)) / pow(n_r, 2);
	if (det < 0) {
		return getReflectDirection(d, n);
	}
	return normalize(n_d * (d - n * dot(d, n)) / n_r -
				     n * pow(det, 0.5));
}

/*
 * Compute R
 */
real_t Raytracer::getR(real_t denserN, Vector3 norm, Vector3 largerD) {
	real_t R0 = pow((denserN - scene->refractive_index) / (denserN + scene->refractive_index), 2);
	real_t cosin = dot(norm, largerD);
	return R0 + (1 - R0) * pow(1 - cosin, 5);
}

} /* _462 */
