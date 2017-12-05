//#include "MeshVolume\MeshVolume.h"
#include <map>
#include <iostream>
int main() {
	//using namespace Volume;
	//// Create mesh object
	//MeshVolume myMesh;
	//// Add eight vertices
	//OpenVolumeMesh::VertexHandle v0 = myMesh.add_vertex(Vec3d(-1.0, 0.0, 0.0));
	//OpenVolumeMesh::VertexHandle v1 = myMesh.add_vertex(Vec3d(0.0, 0.0, 1.0));
	//OpenVolumeMesh::VertexHandle v2 = myMesh.add_vertex(Vec3d(1.0, 0.0, 0.0));
	//OpenVolumeMesh::VertexHandle v3 = myMesh.add_vertex(Vec3d(0.0, 0.0, -1.0));
	//OpenVolumeMesh::VertexHandle v4 = myMesh.add_vertex(Vec3d(0.0, 1.0, 0.0));
	//std::vector<OpenVolumeMesh::VertexHandle> vertices;
	//// Add faces
	//vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v4);
	//OpenVolumeMesh::FaceHandle f0 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v1); vertices.push_back(v2); vertices.push_back(v4);
	//OpenVolumeMesh::FaceHandle f1 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v0); vertices.push_back(v1); vertices.push_back(v2);
	//OpenVolumeMesh::FaceHandle f2 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v0); vertices.push_back(v4); vertices.push_back(v2);
	//OpenVolumeMesh::FaceHandle f3 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v0); vertices.push_back(v4); vertices.push_back(v3);
	//OpenVolumeMesh::FaceHandle f4 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v2); vertices.push_back(v3); vertices.push_back(v4);
	//OpenVolumeMesh::FaceHandle f5 = myMesh.add_face(vertices);
	//vertices.clear();
	//vertices.push_back(v0); vertices.push_back(v2); vertices.push_back(v3);
	//OpenVolumeMesh::FaceHandle f6 = myMesh.add_face(vertices);
	//std::vector<OpenVolumeMesh::HalfFaceHandle> halffaces;
	//// Add first tetrahedron
	//halffaces.push_back(myMesh.halfface_handle(f0, 1));
	//halffaces.push_back(myMesh.halfface_handle(f1, 1));
	//halffaces.push_back(myMesh.halfface_handle(f2, 0));
	//halffaces.push_back(myMesh.halfface_handle(f3, 1));
	//myMesh.add_cell(halffaces);
	//// Add second tetrahedron
	//halffaces.clear();
	//halffaces.push_back(myMesh.halfface_handle(f4, 1));
	//halffaces.push_back(myMesh.halfface_handle(f5, 1));
	//halffaces.push_back(myMesh.halfface_handle(f3, 0));
	//halffaces.push_back(myMesh.halfface_handle(f6, 0));
	//myMesh.add_cell(halffaces);

	//HalfEdgeHandle h1 = myMesh.halfedge(v2, v4);
	//HalfEdgeHandle h2 = myMesh.halfedge(v4, v2);

	//for (HalfEdgeCellIter heciter = myMesh.hec_iter(h1); heciter.valid(); ++heciter) {
	//	CellHandle c = *heciter;
	//	auto verts = myMesh.CellVertices(c);
	//	std::cout << verts[0].idx() << verts[1].idx() << verts[2].idx() << verts[3].idx() << std::endl;
	//}

	//for (HalfEdgeCellIter heciter = myMesh.hec_iter(h2); heciter.valid(); ++heciter) {
	//	CellHandle c = *heciter;
	//	auto verts = myMesh.CellVertices(c);
	//	std::cout << verts[0].idx() << verts[1].idx() << verts[2].idx() << verts[3].idx() << std::endl;
	//}

	//std::cout << (-1) % 2;
	
	std::map<int, int> test;

	std::cout << test.count(1);
	
	system("pause");
}