#include "p5/collisions.hpp"

namespace _462 {

/*
 * Given a point p in the space and the three vertices (a, b, c) of a triangle,
 * return the nearest point inside the triangle to the given point
 */
inline Vector3 closestPointTriangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3 c)
{
	Vector3 ab = b - a;
	Vector3 ac = c - a;
	Vector3 bc = c - b;

	// Compute parametric position s for projection P' of P on AB,
	// P' = A + s*AB, s = snom/(snom+sdenom)
	real_t snom = dot(p - a, ab), sdenom = dot(p - b, a - b);

	// Compute parametric position t for projection P' of P on AC,
	// P' = A + t*AC, s = tnom/(tnom+tdenom)
	real_t tnom = dot(p - a, ac), tdenom = dot(p - c, a - c);

	if (snom <= 0.0f && tnom <= 0.0f) return a; // Vertex region early out

	// Compute parametric position u for projection P' of P on BC,
	// P' = B + u*BC, u = unom/(unom+udenom)
	real_t unom = dot(p - b, bc), udenom = dot(p - c, b - c);

	if (sdenom <= 0.0f && unom <= 0.0f) return b; // Vertex region early out
	if (tdenom <= 0.0f && udenom <= 0.0f) return c; // Vertex region early out

	// P is outside (or on) AB if the triple scalar product [N PA PB] <= 0
	Vector3 n = cross(b - a, c - a);
	real_t vc = dot(n, cross(a - p, b - p));
	// If P outside AB and within feature region of AB,
	// return projection of P onto AB
	if (vc <= 0.0f && snom >= 0.0f && sdenom >= 0.0f)
		return a + snom / (snom + sdenom) * ab;

	// P is outside (or on) BC if the triple scalar product [N PB PC] <= 0
	real_t va = dot(n, cross(b - p, c - p));
	// If P outside BC and within feature region of BC,
	// return projection of P onto BC
	if (va <= 0.0f && unom >= 0.0f && udenom >= 0.0f)
		return b + unom / (unom + udenom) * bc;

	// P is outside (or on) CA if the triple scalar product [N PC PA] <= 0
	real_t vb = dot(n, cross(c - p, a - p));
	// If P outside CA and within feature region of CA,
	// return projection of P onto CA
	if (vb <= 0.0f && tnom >= 0.0f && tdenom >= 0.0f)
		return a + tnom / (tnom + tdenom) * ac;

	// P must project inside face region. Compute Q using barycentric coordinates
	real_t u = va / (va + vb + vc);
	real_t v = vb / (va + vb + vc);
	real_t w = 1.0f - u - v;
	return u * a + v * b + w * c;
}

bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
	Vector3 v1_rel = body1.velocity - body2.velocity;
	Vector3 dir_nn = body2.position - body1.position;
	if (dot(v1_rel, dir_nn) <= 0) {
		return false;
	}
	if (squared_distance(body1.position, body2.position) >= 
		(body1.radius + body2.radius) * (body1.radius + body2.radius)) {
		return false;
	}
	Vector3 dir_n = normalize(dir_nn);
	Vector3 v2_apart_rel = 2 * dir_n * (body1.mass / (body1.mass + body2.mass)) *
		dot(v1_rel, dir_n);
	Vector3 v2_new = body2.velocity + v2_apart_rel;
	Vector3 v1_new = body1.velocity + body2.mass / body1.mass * (body2.velocity - v2_new);
	body1.velocity = (1 - collision_damping) * v1_new;
	if (squared_length(body1.velocity) < EPSION) {
		body1.velocity = Vector3::Zero();
	}
	body2.velocity = (1 - collision_damping) * v2_new;
	if (squared_length(body2.velocity) < EPSION) {
		body2.velocity = Vector3::Zero();
	}
	return true;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
	Vector3 edge01 = body2.vertices[1] - body2.vertices[0];
	Vector3 edge02 = body2.vertices[2] - body2.vertices[0];
	Vector3 norm2_nn = cross(edge01, edge02);
	if (dot(body1.velocity, norm2_nn) >= 0) {
		return false;
	}
	Vector3 nearestPoint = closestPointTriangle(body1.position, body2.vertices[0], body2.vertices[1], body2.vertices[2]);
	if (squared_distance(body1.position, nearestPoint) >= body1.radius * body1.radius) {
		return false;
	}
	Vector3 reflectNorm = normalize(body1.position - nearestPoint);
	Vector3 v1_new = body1.velocity - 2 * dot(body1.velocity, reflectNorm) * reflectNorm;
	body1.velocity = (1 - collision_damping) * v1_new;
	if (squared_length(body1.velocity) < EPSION) {
		body1.velocity = Vector3::Zero();
	}
	return true;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
	if (dot(body1.velocity, body2.normal) >= 0) {
		return false;
	}
	Vector3 someVect = body1.position - body2.position;
	real_t d = dot(someVect, body2.normal);
	if (abs(d) >= body1.radius) {
		return false;
	}
	Vector3 v1_new = body1.velocity - 2 * dot(body1.velocity, body2.normal) * body2.normal;
	body1.velocity = (1 - collision_damping) * v1_new;
	if (squared_length(body1.velocity) < EPSION) {
		body1.velocity = Vector3::Zero();
	}
	return true;
}

}
