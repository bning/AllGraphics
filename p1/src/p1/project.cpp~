/**
 * @file project.cpp
 * @brief OpenGL project
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "p1/project.hpp"

// use this header to include the OpenGL headers
// DO NOT include gl.h or glu.h directly; it will not compile correctly.
#include "application/opengl.hpp"

// A namespace declaration. All proejct files use this namespace.
// Add this declration (and its closing) to all source/headers you create.
// Note that all #includes should be BEFORE the namespace declaration.
namespace _462 {

// definitions of functions for the OpenglProject class

// constructor, invoked when object is created
OpenglProject::OpenglProject()
{
    // TODO any basic construction or initialization of members
    // Warning: Although members' constructors are automatically called,
    // ints, floats, pointers, and classes with empty contructors all
    // will have uninitialized data!
}

// destructor, invoked when object is destroyed
OpenglProject::~OpenglProject()
{
    // TODO any final cleanup of members
    // Warning: Do not throw exceptions or call virtual functions from deconstructors!
    // They will cause undefined behavior (probably a crash, but perhaps worse).
	// destroy();
}

/**
 * Initialize the project, doing any necessary opengl initialization.
 * @param camera An already-initialized camera.
 * @param scene The scene to render.
 * @return true on success, false on error.
 */
bool OpenglProject::initialize( Camera* camera, Scene* scene )
{
	std::cout << "Initializing! " << std::endl;
    	// copy scene
    	this->scene = *scene;
	// Member numbers
	numHeightmapVertices = (GRID_RESOLUTION + 1) * (GRID_RESOLUTION + 1);
	numHeightmapTriangles = 2 * GRID_RESOLUTION  * GRID_RESOLUTION;
	// Initialize mesh and hightmap angles
	meshAngle = 0;
	meshRotateVector = Vector3::Zero;
	heightmapAngle = 0;
	heightmapRotateVector = Vector3::Zero;
	// Initalize the non-nomalized norms of the vertices
	meshVertexNorms = new Vector3[scene->mesh.num_vertices];
	// Initialize Vertices' coordinates
	heightmapVertices = new Vector3[numHeightmapVertices];
	// Allocate memeries for heighmap triangles;
	heightmapTriangles = new Triangle[numHeightmapTriangles];
	//  Allocate memories
	heightmapVertexNorms = new Vector3[numHeightmapVertices];

	/******************  Mesh Data Structures Initialization  ******************/
	// Initalize the non-nomalized norms of the vertices
	for (unsigned int i = 0; i < scene->mesh.num_vertices; i++) {
		meshVertexNorms[i] = Vector3::Zero;
	}
	/******************  Heightmap Data Structures Initialization  ******************/
	// Get the x,y and z coordinates of each vertex of the heightmap
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		int row = i / (GRID_RESOLUTION + 1);
		int col = i % (GRID_RESOLUTION + 1);
		heightmapVertices[i].x = -1.0 + 2.0 / GRID_RESOLUTION * col;
		heightmapVertices[i].z = -1.0 + 2.0 / GRID_RESOLUTION * row;
		heightmapVertices[i].y = scene->heightmap->compute_height(Vector2(heightmapVertices[i].x, heightmapVertices[i].z));
	}
	// Get the indices of the vertices in each triangle
	for (unsigned int i = 0; i < numHeightmapTriangles - 1; i += 2) {
		heightmapTriangles[i].vertices[0] = i / 2 + i / 2 / GRID_RESOLUTION;
		heightmapTriangles[i].vertices[1] = i / 2 + i / 2 / GRID_RESOLUTION + GRID_RESOLUTION + 1;
		heightmapTriangles[i].vertices[2] = i / 2 + i / 2 / GRID_RESOLUTION + GRID_RESOLUTION + 1 + 1;
		heightmapTriangles[i + 1].vertices[0] = i / 2 + i / 2 / GRID_RESOLUTION;
		heightmapTriangles[i + 1].vertices[1] = i / 2 + i / 2 / GRID_RESOLUTION + GRID_RESOLUTION + 2;
		heightmapTriangles[i + 1].vertices[2] = i / 2 + i / 2 / GRID_RESOLUTION + 1;
	}

	// Initialize the non-normalized heightmap vertex norms list
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		heightmapVertexNorms[i] = Vector3::Zero;
	}

	/******************  Compute norms  ******************/
	// Mesh triangle and vertex norms
	for (unsigned int i = 0; i < scene->mesh.num_triangles; i++) {
		computeNorms_mesh(i);
	}
	// Normalize the norms
	for (unsigned int i = 0; i < scene->mesh.num_vertices; i++) {
		meshVertexNorms[i] = normalize(meshVertexNorms[i]);
	}
	// Heightmap triangle and vertex norms
	for (unsigned int i = 0; i < numHeightmapTriangles; i++) {
		computeNorms_heightmap(i);
	}
	// Normalize the norms
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		heightmapVertexNorms[i] = normalize(heightmapVertexNorms[i]);
	}	
	// Return
	std::cout << "Initialized! " << std::endl;
	return true;
}

/**
 * Clean up the project. Free any memory, etc.
 */
void OpenglProject::destroy() {
	// Delete mesh data structures
	delete[] meshVertexNorms;
	// Delete heightmap data structures
	delete[] heightmapVertices;
	delete[] heightmapTriangles;
	delete[] heightmapVertexNorms;
}

/**
 * Perform an update step. This happens on a regular interval.
 * @param dt The time difference from the previous frame to the current.
 */
void OpenglProject::update( real_t dt ) {
    	// update our heightmap
    	scene.heightmap->update( dt );
	// Reset the norms of the heightmap vertices to ZERO
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		heightmapVertexNorms[i] = Vector3::Zero;
	}
	// Compute the new height
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		heightmapVertices[i].y = scene.heightmap->compute_height(Vector2(heightmapVertices[i].x, heightmapVertices[i].z));
	}
	// Compute the new Norms
	for (unsigned int i = 0; i < numHeightmapTriangles; i++) {
		computeNorms_heightmap(i);
	}
	// Normalize the norms
	for (unsigned int i = 0; i < numHeightmapVertices; i++) {
		heightmapVertexNorms[i] = normalize(heightmapVertexNorms[i]);
	}
	// std::cout << "updated once! " << std::endl;
}

/**
 * Clear the screen, then render the mesh using the given camera.
 * @param camera The logical camera to use.
 * @see math/camera.hpp
 */
void OpenglProject::render( const Camera* camera ) {
	// Basic setups
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	// Set up the light source
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

	// Initiate the matrix as MODEL_VIEW, set to IDENTITY
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	/********************* Configure and drow the mesh ***********************/
	// Push modelview matrix before transformation
	glPushMatrix();
	// Set to Vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	// Specify the vertices
	glVertexPointer(3, GL_DOUBLE, 0, &scene.mesh.vertices[0].x);
	// Set to norm array
	glEnableClientState(GL_NORMAL_ARRAY);
	// Specify the norms
	glNormalPointer(GL_DOUBLE, 0, &meshVertexNorms[0].x);
	// Configure the COLOR and MATERIAL
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, MESH_AMB_DIF);
	glMaterialfv(GL_FRONT, GL_SPECULAR, MESH_SPEC);
	// Translation -----STEP 3
	glTranslatef(scene.mesh_position.position.x, scene.mesh_position.position.y, scene.mesh_position.position.z);
	// Rotation -----STEP 2
	scene.mesh_position.orientation.to_axis_angle(&meshRotateVector, &meshAngle);
	glRotatef(meshAngle, meshRotateVector.x, meshRotateVector.y, meshRotateVector.z);
	// Scale -----STEP 1
	glScalef(scene.mesh_position.scale.x, scene.mesh_position.scale.y, scene.mesh_position.scale.z);
	// Draw Elements
	glDrawElements(GL_TRIANGLES, 3 * scene.mesh.num_triangles, GL_UNSIGNED_INT, scene.mesh.triangles);
	// Pop model view matrix after drawing
	glPopMatrix();

	/********************* Configure and drow the mesh ***********************/
	// Push modelview matrix before transformation
	glPushMatrix();
	// Set to Vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	// Specify the vertices
	glVertexPointer(3, GL_DOUBLE, 0, &heightmapVertices[0].x);
	// Set to norm array
	glEnableClientState(GL_NORMAL_ARRAY);
	// Specify the norms
	glNormalPointer(GL_DOUBLE, 0, &heightmapVertexNorms[0].x);
	// Configure the COLOR and MATERIAL
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, HEIGHTMAP_AMB_DIF);
	glMaterialfv(GL_FRONT, GL_SPECULAR, HEIGHTMAP_SPEC);
	glMaterialfv(GL_FRONT, GL_SHININESS, HEIGHTMAP_SHIN);
	// Translation -----STEP 3
	glTranslatef(scene.heightmap_position.position.x, scene.heightmap_position.position.y, scene.heightmap_position.position.z);
	// Rotation -----STEP 2
	scene.heightmap_position.orientation.to_axis_angle(&heightmapRotateVector, &heightmapAngle);
	glRotatef(heightmapAngle, heightmapRotateVector.x, heightmapRotateVector.y, heightmapRotateVector.z);
	// Scale -----STEP 1
	glScalef(scene.heightmap_position.scale.x, scene.heightmap_position.scale.y, scene.heightmap_position.scale.z);
	// Draw Elements
	glDrawElements(GL_TRIANGLES, 3 * numHeightmapTriangles, GL_UNSIGNED_INT, heightmapTriangles);
	// Pop model view matrix after drawing
	glPopMatrix();

	/***************************** Configure Camera ******************************/
	// Set matrix to projection model
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	// Set up Perspective
	gluPerspective(camera->get_fov_degrees(), camera->get_aspect_ratio(), camera->get_near_clip(), camera->get_far_clip());
	// Set up lookAt
	gluLookAt(camera->get_position().x, \
		camera->get_position().y, \
		camera->get_position().z, \
		camera->get_position().x + camera->get_direction().x, \
		camera->get_position().y + camera->get_direction().y, \
		camera->get_position().z + camera->get_direction().z, \
		camera->get_up().x, \
		camera->get_up().y, \
		camera->get_up().z);
	// Set up the Display Parameters
	glViewport(0, 0, 1024, 768);
	// Disable the used functions
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_NORMALIZE);
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

/**
 * Compute the normalized norm vector of the triagle given by its index
 */
void OpenglProject::computeNorms_mesh(unsigned int triangleInd) {
	// Get the indices of the three vertices of the triangle
	int ind_a = scene.mesh.triangles[triangleInd].vertices[0];
	int ind_b = scene.mesh.triangles[triangleInd].vertices[1];
	int ind_c = scene.mesh.triangles[triangleInd].vertices[2];
	// std::cout << "updated once! " << std::endl;
	// Get the vertices' coordinates
	Vector3 vtx_a = scene.mesh.vertices[ind_a];
	Vector3 vtx_b = scene.mesh.vertices[ind_b];
	Vector3 vtx_c = scene.mesh.vertices[ind_c];
	// Compute the norm and assign to meshTriangleNorms
	Vector3 norm = normalize(cross(vtx_b - vtx_a, vtx_c - vtx_a));
	// The total norms of the three vertices also added
	meshVertexNorms[ind_a] += norm;
	meshVertexNorms[ind_b] += norm;
	meshVertexNorms[ind_c] += norm;
}

/**
* Compute the normalized norm vector of the triagle given by its index
*/
void OpenglProject::computeNorms_heightmap(unsigned int triangleInd) {
	// Get the indices of the three vertices of the triangle
	int ind_a = heightmapTriangles[triangleInd].vertices[0];
	int ind_b = heightmapTriangles[triangleInd].vertices[1];
	int ind_c = heightmapTriangles[triangleInd].vertices[2];
	// Get the vertices' coordinates
	Vector3 vtx_a = heightmapVertices[ind_a];
	Vector3 vtx_b = heightmapVertices[ind_b];
	Vector3 vtx_c = heightmapVertices[ind_c];
	// Compute the norm and assign to meshTriangleNorms
	Vector3 norm = normalize(cross(vtx_b - vtx_a, vtx_c - vtx_a));
	// The total norms of the three vertices also added
	heightmapVertexNorms[ind_a] += norm;
	heightmapVertexNorms[ind_b] += norm;
	heightmapVertexNorms[ind_c] += norm;
	// std::cout << "heightmap norm computed ones " << std::endl;
}

} /* _462 */

