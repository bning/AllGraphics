/**
 * @file raytacer.cpp
 * @brief Raytracer class
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

	// max number fof threads OpenMP can use. Change this if you like.
#define MAX_THREADS 12

static const unsigned STEP_SIZE = 8;

Raytracer::Raytracer()
	: scene(0), width(0), height(0) { }

// random real_t in [0, 1)
static inline real_t random()
{
	return real_t(rand()) / RAND_MAX;
}

static inline real_t normal()
{
	static std::default_random_engine generator;
	static std::normal_distribution<real_t> dist =
		std::normal_distribution<real_t>();
	return dist(generator);
}

static inline Vector3 point_on_sphere()
{
	Vector3 r;
	r.x = normal();
	r.y = normal();
	r.z = normal();
	return normalize(r);
}
// get left child index in the tree list
static inline int leftChildIndex(const int parentIndex) {
	return 2 * (parentIndex + 1) - 1;
};
// get right child index in the tree list
static inline int rightChildIndex(const int parentIndex) {
	return 2 * (parentIndex + 1);
};
// get the tree depth by given index in the treeVect
static inline int index2depth(const int index) {
	int depth = 0;
	int fakeInd = index + 1;
	while (fakeInd != 1) {
		fakeInd = fakeInd >> 1;
		depth++;
	}
	return depth;
};
// get the size of tree vector from the size of photon vector
static inline int indSize2treeSize(const int indexVectSize) {
	int n = 1;
	while (n <= indexVectSize) {
		n = n * 2;
	}
	return n - 1;
};
// max of three number of size_t
static inline real_t max3(const real_t num1, 
	const real_t num2, const real_t num3) {
	return std::max(std::max(num1, num2), num3);
};
// min of three number of size_t
static inline real_t min3(const real_t num1, 
	const real_t num2, const real_t num3) {
	return std::min(std::min(num1, num2), num3);
}
// uniform point on hemisphere
static inline Vector3 uniformSampleHemiSphere(const Vector3& normal) {
	Vector3 newDir = point_on_sphere();
	if (dot(newDir, normal) < 0.0) newDir = -newDir;
	return newDir;
}
// check whether two vectors are almost vertical
static inline bool checkVertical(Vector3 v1, Vector3 v2) {
	return abs(dot(v1, v2)) < SMALL_ANGLE;
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

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// build the photon maps ------------------------------------------------------------- BUILD PHOTON MAP
	printf("Setting up photon maps...");
	setPhotonMap();
	printf("done\n");
	printf("Caustic photons: %ld, Global photons: %ld\n", causticPhotonList.size(), globalPhotonList.size());
	// build the k-d tree for both caustic photon ---------------------------------------- BUILD K-D TREE
	printf("Builing K-D tree for caustic photon list...");
	buildKDTree(&causticPhotonList, &causticTreeVect);
	printf("done...Caustic tree size: %ld\n", causticTreeVect.size());
	printf("Builing K-D tree for global photon list...");
	buildKDTree(&globalPhotonList, &globalTreeVect);
	printf("done...Global tree size: %ld\n", globalTreeVect.size());
	////////////////////////////////////////////////////////////////////////////////////////////////////////

	return true;
}

static inline Vector3 reflect(const Vector3& I, const Vector3& N)
{
	return normalize(I - real_t(2.0)*dot(N, I)*N);
}

inline Color3 Raytracer::direct_illumination(const Intersection& info)
{
	const SphereLight* lights = scene->get_lights();
	unsigned int N = scene->num_lights();

	Color3 res = Color3::Black();
	Intersection tinfo;
	for (unsigned int i = 0; i < N; i++)
	{
		for (unsigned int j = 0; j < LIGHT_SAMPLE; j++) {
			tinfo.t = DBL_MAX;
			Vector3 lightDir = (lights[i].position +
				point_on_sphere()*lights[i].radius) - info.pos;
			real_t dist = length(lightDir);

			Ray shadow(info.pos + lightDir*0.00001, lightDir);
			if (!scene->intersect(shadow, &tinfo) || tinfo.t > real_t(1))
			{
				lightDir /= dist;

				real_t weight = std::max(dot(info.normal, lightDir), real_t(0.0));

				res += lights[i].color*info.diffuse*weight;
			}
		}
	}
	return res * (1.0 / LIGHT_SAMPLE);
}

static inline bool refract(const Vector3& d, const Vector3& n, real_t d_n,
	real_t refI, Vector3* t)
{
	// create refraction ray
	real_t disc = 1.0 - refI*refI*(1 - d_n*d_n);
	if (disc < 0.0)
		return false; // total internal reflection

	real_t rad = sqrt(disc);

	if (d_n > 0.0)
		*t = refI*(d - n*d_n) + n*rad;
	else
		*t = refI*(d - n*d_n) - n*rad;

	return true;
}

Color3 Raytracer::refraction(const Ray& r, const Intersection* info,
	const Color3& c, unsigned int depth)
{
	if (depth <= 0) return Color3::Black();

	real_t d_n = dot(r.d, info->normal);

	real_t objRI = info->refractive_index,
		refIndex = objRI / scene->refractive_index;

	if (d_n < 0.0)
	{
		refIndex = 1.0 / refIndex; // inside object.. kinda hacky.
	}

	Vector3 t;

	Color3 rest = Color3::Black();
	real_t R = 1.0;
	Color3 cc = c;

	if (refract(r.d, info->normal, d_n, refIndex, &t))
	{
		// do refraction calculations and fresnel effect.

		Ray nr = Ray(r.e + 0.00001*t, t);
		real_t R0 = (objRI - 1.0)*(objRI - 1.0) / ((objRI + 1.0)*(objRI + 1.0));
		real_t f = 1 - fabs(d_n);
		real_t g = f*f;
		g *= g;

		R = R0 + (1.0 - R0)*g*f;

		if (random() < R)
		{
			Ray reflected;
			reflected.d = reflect(r.d, info->normal);
			reflected.e = info->pos + 0.0001*reflected.d;
			Color3 reflColor = trace(reflected, depth - 1);
			cc = info->specular*reflColor;
			return cc;
		}
		else
		{
			return trace(nr, depth - 1);
		}

	}
	else
	{
		Vector3 newDir = reflect(r.d, info->normal);
		Ray newr = Ray(r.e + 0.00001*newDir, newDir);
		cc = trace(newr, depth - 1);
	}
	return (R*cc + rest);
}

Color3 Raytracer::trace(const Ray& r, unsigned int depth)
{
	if (depth == 0) return Color3::Black();

	Intersection info;

	if (!scene->intersect(r, &info))
		return scene->background_color;

	Color3 color = scene->ambient_light*info.ambient;

	Color3 reflColor = Color3::Black();
	Color3 refrColor = Color3::Black();

	color += direct_illumination(info);

	if (info.refractive_index > 0)
	{
		Ray reflected;
		reflected.d = reflect(r.d, info.normal);
		reflected.e = info.pos + 0.0001*reflected.d;
		reflColor = trace(reflected, depth - 1);
		color = info.specular*reflColor;
		Ray R = Ray(info.pos + r.d*0.00001, r.d);
		color = refraction(R, &info, color, depth);
	}
	else if (info.specular != Color3::Black())
	{
		Ray reflected;
		reflected.d = reflect(r.d, info.normal);
		reflected.e = info.pos + 0.00001*reflected.d;
		reflColor = trace(reflected, depth - 1);
		color += info.specular*reflColor;
	}
	Color3 causticComp = Color3::Black();
	Color3 globalComp = Color3::Black();
	if (depth == MAX_DEPTH && info.refractive_index == 0) {
		IDPairVect nearestIndDistPair;
		// caustic component
		if (causticPhotonList.size() > 0) {
			knn(&causticPhotonList, &causticTreeVect, &nearestIndDistPair, &info);
			causticComp = getRadiance(&causticPhotonList, &nearestIndDistPair, &info);
		}
		// global component
		if (globalPhotonList.size() > 0 ) {
			knn(&globalPhotonList, &globalTreeVect, &nearestIndDistPair, &info);
			globalComp = getRadiance(&globalPhotonList, &nearestIndDistPair, &info);
		}
	}
	return R_SCALE * info.texColor*color + C_SCALE* causticComp + G_SCALE * globalComp;
}

real_t squared_length(const Color3& color)
{
	return color.r*color.r + color.g*color.g + color.b*color.b;
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
	size_t x, size_t y, size_t width, size_t height)
{
	assert(x < width);
	assert(y < height);

	real_t dx = real_t(1) / width;
	real_t dy = real_t(1) / height;

	Color3 res = Color3::Black();
	unsigned int iter;
	for (iter = 0; iter < num_samples; iter++)
	{
		real_t i = real_t(2)*(real_t(x) + random())*dx - real_t(1);
		real_t j = real_t(2)*(real_t(y) + random())*dy - real_t(1);
		Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(i, j));
		res += trace(r, MAX_DEPTH);
	}
	return res*(real_t(1) / num_samples);
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
		unsigned int duration = (unsigned int)(*max_time * 1000);
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

bool Raytracer::buildKDTree(const PtList* photonListPtr, IntVect* treeVectPtr) {
	(*treeVectPtr).clear();
	IntVect indexVect;
	indexVect.reserve((*photonListPtr).size());
	for (unsigned int i = 0; i < (*photonListPtr).size(); i++) {
		indexVect.push_back(i);
	}
	*treeVectPtr = IntVect(indSize2treeSize(indexVect.size()), -1);
	kdTree(photonListPtr, &indexVect, treeVectPtr, 0, 1);
	return true;
}

void Raytracer::kdTree(const PtList* photonListPtr, 
	IntVect* indexVectPtr, IntVect* treeVectPtr, 
	const int axis, const int treeIndex) {
	if ((*indexVectPtr).size() == 0) {
		return;
	}
	int ax = axis % 3;
	std::sort((*indexVectPtr).begin(), (*indexVectPtr).end(), SortAxis(photonListPtr, ax));
	int medianIndex = (*indexVectPtr).size() / 2;
	(*treeVectPtr)[treeIndex - 1] = (*indexVectPtr)[medianIndex];
	// recursive call
	IntVect leftIndexVect = IntVect((*indexVectPtr).begin(), (*indexVectPtr).begin() + medianIndex);
	kdTree(photonListPtr, &leftIndexVect, treeVectPtr, axis + 1, 2 * treeIndex);
	IntVect rightIndexVect = IntVect((*indexVectPtr).begin() + medianIndex + 1, (*indexVectPtr).end());
	kdTree(photonListPtr, &rightIndexVect, treeVectPtr, axis + 1, 2 * treeIndex + 1);
}

void Raytracer::knn(const PtList* photonListPtr,
	const IntVect* treeVectPtr, IDPairVect* nearestIndDistPairPtr,
	const Intersection* infoPtr) {
	(*nearestIndDistPairPtr).reserve(NUM_NEIGHBORS + 1);
	(*nearestIndDistPairPtr).clear();
	search(photonListPtr, treeVectPtr, 0, nearestIndDistPairPtr, infoPtr);
}

void Raytracer::search(const PtList* photonListPtr, 
	const IntVect* treeVectPtr, const int currentTreeInd,
	IDPairVect* nearestIndDistPairPtr, const Intersection* infoPtr) {
	int unwindDir;
	if (currentTreeInd >= (*treeVectPtr).size() || (*treeVectPtr)[currentTreeInd] == -1) {
		return;
	}
	const int currenPhotonInd = (*treeVectPtr)[currentTreeInd];
	const Vector3 currentPhotonPos = (*photonListPtr)[currenPhotonInd].pos;
	int currentAxis = index2depth(currentTreeInd) % 3;
	if ((*infoPtr).pos[currentAxis] < currentPhotonPos[currentAxis]) {
		unwindDir = 0;
		search(photonListPtr, treeVectPtr, leftChildIndex(currentTreeInd), nearestIndDistPairPtr, infoPtr);
	}
	else {
		unwindDir = 1;
		search(photonListPtr, treeVectPtr, rightChildIndex(currentTreeInd), nearestIndDistPairPtr, infoPtr);
	}
	Vector3 pt2pVect = normalize(currentPhotonPos - (*infoPtr).pos);
	const real_t currentPhotonDist = squared_distance(currentPhotonPos, (*infoPtr).pos);
	if ((*nearestIndDistPairPtr).size() < NUM_NEIGHBORS && checkVertical(pt2pVect, (*infoPtr).normal)) {
		(*nearestIndDistPairPtr).push_back(IDPair(currenPhotonInd, currentPhotonDist));
	}
	else {
		std::sort((*nearestIndDistPairPtr).begin(), (*nearestIndDistPairPtr).end());
		if (checkVertical(pt2pVect, (*infoPtr).normal) && currentPhotonDist < (*nearestIndDistPairPtr).back().distance) {
			(*nearestIndDistPairPtr).pop_back();
			(*nearestIndDistPairPtr).push_back(IDPair(currenPhotonInd, currentPhotonDist));
		}
	}
	std::sort((*nearestIndDistPairPtr).begin(), (*nearestIndDistPairPtr).end());
	real_t p2plane_real = currentPhotonPos[currentAxis] - (*infoPtr).pos[currentAxis];
	real_t p2plane = p2plane_real * p2plane_real;
	real_t p2far = (*nearestIndDistPairPtr).back().distance;
	if (p2plane > p2far) {
		return;
	}
	if (unwindDir == 0) {
		search(photonListPtr, treeVectPtr, rightChildIndex(currentTreeInd), nearestIndDistPairPtr, infoPtr);
	}
	else {
		search(photonListPtr, treeVectPtr, leftChildIndex(currentTreeInd), nearestIndDistPairPtr, infoPtr);
	}
}

void Raytracer::setPhotonMap() {
	causticPhotonList.reserve(MAX_PHOTONS);
	globalPhotonList.reserve(MAX_PHOTONS); 
	intersectStack.reserve(MAX_DEPTH);
	causticPhotonList.clear();
	globalPhotonList.clear();
	for (unsigned int i = 0; i < scene->num_lights(); i++)
	{
		for (unsigned int j = 0; j < MAX_PHOTONS; j++) {
			Vector3 rayDir = point_on_sphere();
			Vector3 rayPos = scene->get_lights()[i].radius * rayDir + scene->get_lights()[i].position + 0.00001 * rayDir;
			Ray photonRay = Ray(rayPos, rayDir);
			intersectStack.clear();
			tracePhoton(photonRay, MAX_DEPTH, scene->get_lights()[i].color * (1.0 / MAX_PHOTONS));
		}
	}
}

void Raytracer::tracePhoton(const Ray& photonRay,
	const unsigned int depth, const Color3& rayColor) {
	// reach recursion depth
	if (depth == 0) return;
	Intersection info;
	// no intersection with scene
	if (!scene->intersect(photonRay, &info)) return;	
	// refraction and reflection---------------------------------------------------------------- DIELECTRICS
	if (info.refractive_index > 0) {
		// printf("intersect ref index greater than 0: %f...", info.refractive_index);
		real_t d_n = dot(photonRay.d, info.normal);
		real_t objRI = info.refractive_index;
		real_t refIndex = objRI / scene->refractive_index;
		// inside object
		if (d_n < 0.0) {
			refIndex = 1.0 / refIndex; 
		}
		// refracted (both inside and outside)
		Vector3 t;
		if (refract(photonRay.d, info.normal, d_n, refIndex, &t)) {
			// printf("refract\n");
			real_t R0 = (objRI - 1.0)*(objRI - 1.0) / ((objRI + 1.0)*(objRI + 1.0));
			real_t f = 1 - fabs(d_n);
			real_t g = f*f;
			g *= g;
			real_t R = R0 + (1.0 - R0)*g*f;
			if (random() < R) { // turned out to be reflected
				Ray reflected;
				reflected.d = reflect(photonRay.d, info.normal);
				reflected.e = info.pos + 0.00001*reflected.d;
				intersectStack.push_back(NON_DIFF_REF);
				tracePhoton(reflected, depth - 1, rayColor);
			}
			else { // turned out to be refracted
				Ray nr = Ray(info.pos + 0.00001*t, t);
				intersectStack.push_back(NON_DIFF_REF);
				tracePhoton(nr, depth - 1, rayColor);
			}
		}
		// total internal reflection
		else {
			// printf("total inner reflect\n");
			Vector3 newDir = reflect(photonRay.d, info.normal);
			Ray newr = Ray(info.pos + 0.00001*newDir, newDir);
			intersectStack.push_back(NON_DIFF_REF);
			tracePhoton(newr, depth - 1, rayColor);
		}
	}
	// diffuse reflection, absorsion and specular reflection ----------------------------------- DIFFUSE
	else {
		real_t p_d = max3(info.diffuse.r * rayColor.r, info.diffuse.g * rayColor.g, info.diffuse.b * rayColor.b) / 
			max3(rayColor.r, rayColor.g, rayColor.b);
		real_t p_s = max3(info.specular.r * rayColor.r, info.specular.g * rayColor.g, info.specular.b * rayColor.b) /
			max3(rayColor.r, rayColor.g, rayColor.b);
		real_t thresh1, thresh2;
		// Case 1: consider diffuse reflection, specular reflection and absorsion -------- P < 1
		if (p_d + p_s < 1) {
			// printf("diff case p_d + p_s <<<< 1... ");
			thresh1 = p_d;
			thresh2 = p_d + p_s;
			real_t rand = random();
			// Case 1: -----------------------------------------------diffuse reflection
			if (rand <= thresh1) {
				// printf("diff refl...");
				// add photon
				addPhoton(info.pos, photonRay.d, rayColor);
				// trace new reflected photon ray
				Ray reflected; 
				reflected.d = uniformSampleHemiSphere(info.normal); 
				reflected.e = info.pos + 0.00001 * reflected.d;
				Color3 reflectedColor;
				// without texture color
				if (info.texColor == Color3::Black() || info.texColor == Color3::White()) {
					reflectedColor = info.diffuse * rayColor;
					// printf("ray color added diff %f, %f, %f\n", info.diffuse[0], info.diffuse[1], info.diffuse[2]);
				}
				// with texture color
				else {
					reflectedColor = info.texColor * rayColor;
					// printf("ray color added tex %f, %f, %f\n", info.texColor[0], info.texColor[1], info.texColor[2]);
				}
				// current ray is from a diffuse reflection
				intersectStack.push_back(DIFF_REF);
				tracePhoton(reflected, depth - 1, reflectedColor);
			} 
			// Case 2: ----------------------------------------------specular reflection
			else if (rand > thresh1 && rand <= thresh2) {
				// printf("spec refl\n");
				Ray reflected;
				reflected.d = reflect(photonRay.d, info.normal);
				reflected.e = info.pos + 0.00001 * reflected.d;
				Color3 reflectedColor;
				// without texture color
				if (info.texColor == Color3::Black() || info.texColor == Color3::White()) reflectedColor = info.specular * rayColor;
				// with texture color
				else reflectedColor = info.texColor * rayColor;
				// current ray is from a diffuse reflection
				intersectStack.push_back(NON_DIFF_REF);
				tracePhoton(reflected, depth - 1, reflectedColor);
			}
			// Case 3: ----------------------------------------------absorbed
			else {
				// printf("absorb\n");
				addPhoton(info.pos, photonRay.d, rayColor);
			}
		}
		// Case 2: consider diffuse reflection and specular reflection ------------------ P >= 1
		else {
			// printf("diff case p_d + p_s >>>> 1... ");
			thresh1 = p_d / (p_d + p_s);
			real_t rand = random();
			// Case 1: diffuse reflection --------------------------diffuse reflection
			if (rand <= thresh1) {
				// printf("diff refl\n");
				// add photon
				addPhoton(info.pos, photonRay.d, rayColor);
				// trace new reflected photon ray
				Ray reflected;
				reflected.d = uniformSampleHemiSphere(info.normal);
				reflected.e = info.pos + 0.00001 * reflected.d;
				Color3 reflectedColor;
				// without texture color
				if (info.texColor == Color3::Black() || info.texColor == Color3::White()) reflectedColor = info.diffuse * rayColor;
				// with texture color
				else reflectedColor = info.texColor * rayColor;
				// current ray is from a diffuse reflection
				intersectStack.push_back(DIFF_REF);
				tracePhoton(reflected, depth - 1, reflectedColor);
			}
			// Case 2: specular reflection -------------------------specular reflection
			else {
				// printf("spec refl\n");
				Ray reflected;
				reflected.d = reflect(photonRay.d, info.normal);
				reflected.e = info.pos + 0.00001 * reflected.d;
				Color3 reflectedColor;
				// without texture color
				if (info.texColor == Color3::Black() || info.texColor == Color3::White()) reflectedColor = info.specular * rayColor;
				// with texture color
				else reflectedColor = info.texColor * rayColor;
				// current ray is from a diffuse reflection
				intersectStack.push_back(NON_DIFF_REF);
				tracePhoton(reflected, depth - 1, reflectedColor);
			}
		}
	}
}

inline void Raytracer::addPhoton(const Vector3& photonPos,
	const Vector3& photonDir, const Color3& photonColor) {
	// printf("photon added color %f, %f, %f\n", photonColor[0], photonColor[1], photonColor[2]);
	if (intersectStack.size() > 0) {
		Photon pt;
		pt.col = photonColor;
		pt.dir = photonDir;
		pt.pos = photonPos;
		// photon ray from a diffuse reflection, stored in global
		if (intersectStack.back() == DIFF_REF) {
			globalPhotonList.push_back(pt);
		}
		// photon ray from other reflections, stored in caustic
		else {
			causticPhotonList.push_back(pt);
		}
	}
}

inline Color3 Raytracer::getRadiance(const PtList* photonListPtr,
	const IDPairVect* nearestIndDistPairPtr, 
	const Intersection* infoPtr) {
	// real_t sqrRadius = (*nearestIndDistPairPtr).front().distance;
	real_t sqrRadius = (*nearestIndDistPairPtr).back().distance;
	// printf("squared R: %f\n", sqrRadius);
	Color3 totalRadiance = Color3::Black();
	for (unsigned int i = 0; i < (*nearestIndDistPairPtr).size(); i++) {
		int ptIndex = (*nearestIndDistPairPtr)[i].index;
		// printf("pt index: %d ", ptIndex);
		Color3 ptColor = (*photonListPtr)[ptIndex].col;
		// printf("pt color: %d %d %d ", ptColor[0],ptColor[1], ptColor[2]);
		Vector3 ptDir = (*photonListPtr)[ptIndex].dir;
		totalRadiance += ptColor * std::max(0.0, dot((*infoPtr).normal, -ptDir));
	}
	// printf("\n");
	totalRadiance = totalRadiance * (1.0 / (PI * sqrRadius));
	if ((*infoPtr).texColor == Color3::Black() || (*infoPtr).texColor == Color3::White()) {
		return (*infoPtr).diffuse * totalRadiance;
	}
	return (*infoPtr).texColor * totalRadiance;
}

} /* _462 */
