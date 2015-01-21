/**
 * @file mesh.hpp
 * @brief Mesh class and OBJ loader.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#ifndef _462_SCENE_MESH_HPP_
#define _462_SCENE_MESH_HPP_

#include "math/vector.hpp"

#include <vector>
#include <cassert>

namespace _462 {

struct MeshVertex
{
    std::vector < unsigned int > adjacentTriangles; // Added attribute
    //Vector4 color;
    Vector3 position;
    Vector3 normal;
    Vector2 tex_coord;

    // Defaul Constructor
    MeshVertex() {
	adjacentTriangles = std::vector < unsigned int > ();
	adjacentTriangles.clear();
	position = Vector3::Zero;
	normal = Vector3::Zero;
	tex_coord = Vector2::Zero;
    }
};

struct MeshTriangle
{
    // index into the vertex list of the 3 vertices
    unsigned int vertices[3];
};

// Edge infomation of the mesh
struct MeshEdge
{
    std::vector < unsigned int > adjacentTriangles;
    int newVertexInd;
    bool isInterior;

    // Default Constructor
    MeshEdge() {
	isInterior = false;
	newVertexInd = -1; // Not available
	adjacentTriangles = std::vector< unsigned int > ();
	adjacentTriangles.reserve(2); // At most two adjacentTriangles
	adjacentTriangles.clear();
    }
};
/**
 * A mesh of triangles.
 */
class Mesh
{
public:

    Mesh();
    virtual ~Mesh();

    typedef std::vector< MeshTriangle > MeshTriangleList;
    typedef std::vector< MeshVertex > MeshVertexList;
    typedef std::vector< std::vector < MeshEdge > > MeshEdgeAdjacencyMatrix;

    // The list of all triangles in this model.
    MeshTriangleList triangles;

    // The list of all vertices in this model.
    MeshVertexList vertices;

    // The list of all edges in this model, represented as an adjacency matrix
    MeshEdgeAdjacencyMatrix edgeMatrix;
    unsigned int numEdges;

    // scene loader stores the filename of the mesh here
    std::string filename;

    bool has_tcoords;
    bool has_normals;
    int has_colors;

    // Loads the model into a list of triangles and vertices.
    bool load();

    // Creates opengl data for rendering and computes normals if needed
    bool create_gl_data();

    bool subdivide();

    // Renders the mesh using opengl.
    void render() const;
private:
    typedef std::vector< float > FloatList;
    typedef std::vector< unsigned int > IndexList;

    // the vertex data used for GL rendering
    FloatList vertex_data;
    // the index data used for GL rendering
    IndexList index_data;

    // Compute the new vertex on the given edge and return the index of the new vertex in vertices
    int getNewVertexIndex(unsigned int edgeRow, unsigned int edgeCol);

    // Update the positions of the even vertices
    Vector3 computeNewEvenVertexPosition(unsigned int vertexIndex);

    // Compute the positions of the Interior vertices
    Vector3 computeInteriorVertex(unsigned int vetexIndex);

    // Compute the positions of the Boundary vertices
    Vector3 computeBoundartVertex(unsigned int vertexIndex, std::vector < unsigned int > linkedBoundPointsInd);

    // prevent copy/assignment
    Mesh( const Mesh& );
    Mesh& operator=( const Mesh& );

};


} /* _462 */

#endif /* _462_SCENE_MESH_HPP_ */
