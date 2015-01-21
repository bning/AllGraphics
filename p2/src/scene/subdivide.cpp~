#include "scene/mesh.hpp"

namespace _462 {

bool Mesh::subdivide()
{
    /*
      You should implement loop subdivision here.

      Triangles are stored in an std::vector<MeshTriangle> in 'triangles'.
      Vertices are stored in an std::vector<MeshVertex> in 'vertices'.

      Check mesh.hpp for the Mesh class definition.
     */

    // Resize the edgeMatrix to contain more vertices
    unsigned int oldVerticesSize = vertices.size();
    unsigned int oldTrianglesSize = triangles.size();

    std::cout << "Current number of vertices " << oldVerticesSize << "\n";
    std::cout << "Current number of edges " << numEdges << "\n";
    std::cout << "Current number of triangles " << oldTrianglesSize << "\n";
    
    edgeMatrix.resize(oldVerticesSize + numEdges, std::vector< MeshEdge >(vertices.size(), MeshEdge()));
    for (unsigned int i = 0; i < edgeMatrix.size(); i++) {
	edgeMatrix[i].resize(oldVerticesSize + numEdges, MeshEdge());
    }

    // Update the relationships and compute the new positions
    for (unsigned int i = 0; i < oldTrianglesSize; i++) {
	// Get the indices of the three new vertices
	int newVertexIndex_edge0 = getNewVertexIndex( triangles[i].vertices[0], triangles[i].vertices[1] );
	int newVertexIndex_edge1 = getNewVertexIndex( triangles[i].vertices[1], triangles[i].vertices[2] );
	int newVertexIndex_edge2 = getNewVertexIndex( triangles[i].vertices[2], triangles[i].vertices[0] );

	// Append new generated triangles
	MeshTriangle newTriangle;
	// v0, e0, e2
	newTriangle.vertices[0] = triangles[i].vertices[0];
	newTriangle.vertices[1] = newVertexIndex_edge0;
	newTriangle.vertices[2] = newVertexIndex_edge2;
	triangles.push_back(newTriangle);
	// v1, e1, e0
	newTriangle.vertices[0] = triangles[i].vertices[1];
	newTriangle.vertices[1] = newVertexIndex_edge1;
	newTriangle.vertices[2] = newVertexIndex_edge0;
	triangles.push_back(newTriangle);
	// v2, e2, e1
	newTriangle.vertices[0] = triangles[i].vertices[2];
	newTriangle.vertices[1] = newVertexIndex_edge2;
	newTriangle.vertices[2] = newVertexIndex_edge1;
	triangles.push_back(newTriangle);
	// e0, e1, e2
	newTriangle.vertices[0] = newVertexIndex_edge0;
	newTriangle.vertices[1] = newVertexIndex_edge1;
	newTriangle.vertices[2] = newVertexIndex_edge2;
	triangles.push_back(newTriangle);

	// Update the edgeMatrix, HANDEL the symmetry
	// Update whether the new edge is interior or not
	// For the edges formed from the boundaries of the triangle, the isInterior is determined by the old boundaries
	// For the edges formed from the all new generated vertices, the inInterior is always TRUE
	// v0 e0
	edgeMatrix[triangles[i].vertices[0]][newVertexIndex_edge0].isInterior = edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[1]].isInterior;
	edgeMatrix[newVertexIndex_edge0][triangles[i].vertices[0]].isInterior = edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[0]].isInterior;
	// v1 e0
	edgeMatrix[triangles[i].vertices[1]][newVertexIndex_edge0].isInterior = edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[0]].isInterior;
	edgeMatrix[newVertexIndex_edge0][triangles[i].vertices[1]].isInterior = edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[1]].isInterior;
	// v1 e1
	edgeMatrix[triangles[i].vertices[1]][newVertexIndex_edge1].isInterior = edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[2]].isInterior;
	edgeMatrix[newVertexIndex_edge1][triangles[i].vertices[1]].isInterior = edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[1]].isInterior;
	// v2 e1
	edgeMatrix[triangles[i].vertices[2]][newVertexIndex_edge1].isInterior = edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[1]].isInterior;
	edgeMatrix[newVertexIndex_edge1][triangles[i].vertices[2]].isInterior = edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[2]].isInterior;
	// v2 e2
	edgeMatrix[triangles[i].vertices[2]][newVertexIndex_edge2].isInterior = edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[0]].isInterior;
	edgeMatrix[newVertexIndex_edge2][triangles[i].vertices[2]].isInterior = edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[2]].isInterior;
	// v0 e2
	edgeMatrix[triangles[i].vertices[0]][newVertexIndex_edge2].isInterior = edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[2]].isInterior;
	edgeMatrix[newVertexIndex_edge2][triangles[i].vertices[0]].isInterior = edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[0]].isInterior;
	// e0 e1
	edgeMatrix[newVertexIndex_edge0][newVertexIndex_edge1].isInterior = true;
	edgeMatrix[newVertexIndex_edge1][newVertexIndex_edge0].isInterior = true;
	// e1 e2
	edgeMatrix[newVertexIndex_edge1][newVertexIndex_edge2].isInterior = true;
	edgeMatrix[newVertexIndex_edge2][newVertexIndex_edge1].isInterior = true;
	// e2 e0
	edgeMatrix[newVertexIndex_edge2][newVertexIndex_edge0].isInterior = true;
	edgeMatrix[newVertexIndex_edge0][newVertexIndex_edge2].isInterior = true;
    }
    // Update the even vertices
    // A new temporary identical newVertices to store the computed new even vertices
    MeshVertexList newVertices;
    // Same size
    newVertices.resize(oldVerticesSize);
    for (unsigned int i = 0; i < oldVerticesSize; i++) {
	// Update the position
	newVertices[i].position = computeNewEvenVertexPosition(i);
    }
    // Copy the newVertices to vertices
    for (unsigned int i = 0; i < oldVerticesSize; i++) {
	vertices[i] = newVertices[i];
    }

    // Delete all the old triangles
    triangles.erase(triangles.begin(), triangles.begin() + oldTrianglesSize);
	
    // Compute the adjacentTriangles of each edge and vertex
    // Vertices' adjacent triangle list has been cleared
    // No need to clear the edges' adjacent triangle lists. All edges are new
    for (unsigned int i = 0; i < triangles.size(); i++) {
	// First vertex
 	vertices[triangles[i].vertices[0]].adjacentTriangles.push_back(i);
	// Second vertex
	vertices[triangles[i].vertices[1]].adjacentTriangles.push_back(i);
	// Third vertex
	vertices[triangles[i].vertices[2]].adjacentTriangles.push_back(i);
	// Row = vertices[0], Col = vertices[1], HANDLE the symmetry of the matrix
	edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[1]].adjacentTriangles.push_back(i);
	edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[0]].adjacentTriangles.push_back(i);
	// Row = vertices[0], Col = vertices[2], take care of the symmetry of the matrix
	edgeMatrix[triangles[i].vertices[0]][triangles[i].vertices[2]].adjacentTriangles.push_back(i);
	edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[0]].adjacentTriangles.push_back(i);
	// Row = vertices[1], Col = vertices[2], take care of the symmetry of the matrix
	edgeMatrix[triangles[i].vertices[1]][triangles[i].vertices[2]].adjacentTriangles.push_back(i);
	edgeMatrix[triangles[i].vertices[2]][triangles[i].vertices[1]].adjacentTriangles.push_back(i);
    }

    // Update the new edge number
    numEdges = 2 * numEdges + 3 * oldTrianglesSize;

    // Compute the normals
    for (unsigned int i = 0; i < vertices.size(); ++i) {
	vertices[i].normal = Vector3::Zero;
    }
    // then sum in all triangle normals
    for (unsigned int i = 0; i < triangles.size(); ++i) {
	Vector3 pos[3];
	for (size_t j = 0; j < 3; ++j) {
	    pos[j] = vertices[triangles[i].vertices[j]].position;
	}
	Vector3 normal = normalize(cross(pos[1] - pos[0], pos[2] - pos[0]));
	for (unsigned int j = 0; j < 3; ++j) {
	    vertices[triangles[i].vertices[j]].normal += normal;
	}
    }
    // then normalize
    for (unsigned int i = 0; i < vertices.size(); ++i) {
	vertices[i].normal = normalize(vertices[i].normal);
    }

    create_gl_data();

    std::cout << "Subdivision completed Once" << std::endl;
    return true;
}

} /* _462 */
