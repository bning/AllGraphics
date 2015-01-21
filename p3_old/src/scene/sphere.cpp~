/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "application/opengl.hpp"

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere()
    : radius(0), material(0) {
intersection.time = INFINITY;
std::cout << "A sphere\n";
}

Sphere::~Sphere() {}

void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}

bool Sphere::checkIntersection(Ray ray) {

	const real_t epsilon = 0.001;
	Vector3 local_e = invMat.transform_point(ray.e);
	Vector3 local_d = normalize(invMat.transform_vector(ray.d));
	real_t det = pow(dot(local_d, local_e), 2.0) -
		dot(local_d, local_d) * (dot(local_e, local_e) - pow(radius, 2.0));
	if (ray.inAir) {
		if (det < 0) {
			return false;
		}
		intersection.time  = (-dot(local_d, local_e) - pow(det, 0.5)) / dot(local_d, local_d);
		if (intersection.time > epsilon) {
			intersection.normal = normalize(normMat * (local_e + intersection.time * local_d));
			return true;
		}
		return false;
	}
	else {
		intersection.time = std::max((-dot(local_d, local_e) - pow(det, 0.5)) / dot(local_d, local_d), (-dot(local_d, local_e) + pow(det, 0.5)) / dot(local_d, local_d));
		intersection.normal = normalize(normMat * (local_e + intersection.time * local_d));
		return true;
	}
}

real_t Sphere::getIntersectionTime() {
	return intersection.time;
}

Vector3 Sphere::getIntersectionNormal() {
	return intersection.normal;
}

Vector3 Sphere::getIntersectionPosition(Ray ray) {
	return ray.e + intersection.time * ray.d;
}

Color3 Sphere::getIntersectionTextureColor(Ray ray) {
	Vector3 local_e = invMat.transform_point(ray.e); // (invMat * Vector4(ray.e, 1)).xyz();
	Vector3 local_d = normalize(invMat.transform_vector(ray.d)); // normalize((invMat * Vector4(ray.d, 1)).xyz());
	Vector3 local_pos = local_e + intersection.time * local_d;
	real_t lat = acos(local_pos.z / radius); 
	real_t log = atan2(local_pos.y, local_pos.x);
	Vector2 intersect_tex_coord = Vector2(log / 2 / PI, (PI - lat) / PI);
	return material->get_texture_pixel(intersect_tex_coord[0], intersect_tex_coord[1]);
}

Color3 Sphere::getIntersectionAmbientColor() {
	return material->ambient;
}

Color3 Sphere::getIntersectionDiffuseColor() {
	return material->diffuse;
}

Color3 Sphere::getIntersectionSpecularColor() {
	return material->specular;
}

real_t Sphere::getIntersectionRefIndex() {
	return material->refractive_index;
}


} /* _462 */

