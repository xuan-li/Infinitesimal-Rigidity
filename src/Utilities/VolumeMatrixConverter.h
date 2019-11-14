#ifndef _VOLUMEMATRIXCONVERTER
#define _VOLUMEMATRIXCONVERTER

#include "Eigen/Dense"
#include "MeshVolume/OpenVolumeHeader.h"
#include <vector>
#include <map>
#include <algorithm>
#include <list>

unsigned long long GetHalfFaceIndex(Eigen::Vector3i vert_indices) {
	std::list<int> verts;
	verts.push_back(vert_indices(0));
	verts.push_back(vert_indices(1));
	verts.push_back(vert_indices(2));
	while (verts.front() > *(++verts.begin()) || verts.front() > verts.back()) {
		verts.push_back(verts.front());
		verts.pop_front();
	}
	return verts.front() * 1e10 + *(++verts.begin()) * 1e5 + verts.back();
}


template<class VolumeMesh>
void MatrixToVolume(VolumeMesh &mesh, Eigen::MatrixXd &TV,
					Eigen::MatrixXi &TT) 
{
	using namespace OpenVolumeMesh;
	//mesh = VolumeMesh();
	std::vector<VertexHandle> vertices;
	for (int i = 0; i < TV.rows(); i++) {
		vertices.push_back(mesh.add_vertex(Vec3d(TV.row(i)[0], TV.row(i)[1], TV.row(i)[2])));
	}

	std::map<unsigned long long, HalfFaceHandle> halffaces;	
	std::cout << "Construct faces and index halffaces "<<std::endl;

	for (int i = 0; i < TT.rows(); i++) {
		
		Eigen::Vector3i vert_indices1;
		Eigen::Vector3i vert_indices2;
		unsigned long long index1;
		unsigned long long index2;

		vert_indices1 = Eigen::Vector3i(TT(i, 0), TT(i, 3), TT(i, 1));
		vert_indices2 = Eigen::Vector3i(TT(i, 0), TT(i, 1), TT(i, 3));
		index1 = GetHalfFaceIndex(vert_indices1);
		index2 = GetHalfFaceIndex(vert_indices2);
		if (!halffaces[index1].is_valid()) {
			std::vector<VertexHandle> verts;
			verts.push_back(vertices[vert_indices1[0]]);
			verts.push_back(vertices[vert_indices1[1]]);
			verts.push_back(vertices[vert_indices1[2]]);
			FaceHandle f = mesh.add_face(verts);
			halffaces[index1] = mesh.halfface_handle(f, 0);
			halffaces[index2] = mesh.halfface_handle(f, 1);
		}

		vert_indices1 = Eigen::Vector3i(TT(i, 0), TT(i, 2), TT(i, 3));
		vert_indices2 = Eigen::Vector3i(TT(i, 0), TT(i, 3), TT(i, 2));
		index1 = GetHalfFaceIndex(vert_indices1);
		index2 = GetHalfFaceIndex(vert_indices2);
		if (!halffaces[index1].is_valid()) {
			std::vector<VertexHandle> verts;
			verts.push_back(vertices[vert_indices1[0]]);
			verts.push_back(vertices[vert_indices1[1]]);
			verts.push_back(vertices[vert_indices1[2]]);
			FaceHandle f = mesh.add_face(verts);
			halffaces[index1] = mesh.halfface_handle(f, 0);
			halffaces[index2] = mesh.halfface_handle(f, 1);
		}

		vert_indices1 = Eigen::Vector3i(TT(i, 0), TT(i, 1), TT(i, 2));
		vert_indices2 = Eigen::Vector3i(TT(i, 0), TT(i, 2), TT(i, 1));
		index1 = GetHalfFaceIndex(vert_indices1);
		index2 = GetHalfFaceIndex(vert_indices2);
		if (!halffaces[index1].is_valid()) {
			std::vector<VertexHandle> verts;
			verts.push_back(vertices[vert_indices1[0]]);
			verts.push_back(vertices[vert_indices1[1]]);
			verts.push_back(vertices[vert_indices1[2]]);
			FaceHandle f = mesh.add_face(verts);
			halffaces[index1] = mesh.halfface_handle(f, 0);
			halffaces[index2] = mesh.halfface_handle(f, 1);
		}

		vert_indices1 = Eigen::Vector3i(TT(i, 1), TT(i, 3), TT(i, 2));
		vert_indices2 = Eigen::Vector3i(TT(i, 1), TT(i, 2), TT(i, 3));
		index1 = GetHalfFaceIndex(vert_indices1);
		index2 = GetHalfFaceIndex(vert_indices2);
		if (!halffaces[index1].is_valid()) {
			std::vector<VertexHandle> verts;
			verts.push_back(vertices[vert_indices1[0]]);
			verts.push_back(vertices[vert_indices1[1]]);
			verts.push_back(vertices[vert_indices1[2]]);
			FaceHandle f = mesh.add_face(verts);
			halffaces[index1] = mesh.halfface_handle(f, 0);
			halffaces[index2] = mesh.halfface_handle(f, 1);
		}
	}

	std::cout << "Construct Cells" << std::endl;
	for (int i = 0; i < TT.rows(); i++) {
		std::vector<HalfFaceHandle> cell_halfface;
		
		Eigen::Vector3i vert_indices;
		unsigned long long index;

		vert_indices = Eigen::Vector3i(TT(i, 0), TT(i, 3), TT(i, 1));
		index = GetHalfFaceIndex(vert_indices);
		cell_halfface.push_back(halffaces[index]);

		vert_indices = Eigen::Vector3i(TT(i, 0), TT(i, 1), TT(i, 2));
		index = GetHalfFaceIndex(vert_indices);
		cell_halfface.push_back(halffaces[index]);

		vert_indices = Eigen::Vector3i(TT(i, 1), TT(i, 3), TT(i, 2));
		index = GetHalfFaceIndex(vert_indices);
		cell_halfface.push_back(halffaces[index]);

		vert_indices = Eigen::Vector3i(TT(i, 0), TT(i, 2), TT(i, 3));
		index = GetHalfFaceIndex(vert_indices);
		cell_halfface.push_back(halffaces[index]);

		mesh.add_cell(cell_halfface);
	}

}

template<class VolumeMesh>
void VolumeToMatrix(VolumeMesh &mesh, 
	Eigen::MatrixXd &TV,
	Eigen::MatrixXi &TT)
{
	TV.resize(mesh.n_vertices(), 3);
	TT.resize(mesh.n_cells(), 4);
	using namespace OpenVolumeMesh;
	for (VertexIter viter = mesh.vertices_begin(); viter != mesh.vertices_end(); viter++) {
		VertexHandle  v = *viter;
		auto point = mesh.vertex(v);
		TV.row(v.idx()) = Eigen::Vector3d(point[0], point[1], point[2]);
	}
	
	for (CellIter citer = mesh.cells_begin(); citer != mesh.cells_end(); citer++) {
		CellHandle c = *citer;
		std::vector<VertexHandle> verts;
		for (CellVertexIter cviter = mesh.cv_iter(c); cviter.valid(); ++cviter) {
			VertexHandle v = *cviter;
			verts.push_back(v);
		}
		Vec3d center = mesh.vertex(verts[0]);
		Vec3d p1 = mesh.vertex(verts[1]);
		Vec3d p2 = mesh.vertex(verts[2]);
		Vec3d normal = cross((p1 - center),(p2 - center));
		normal = normal / normal.norm();
		Vec3d up = mesh.vertex(verts[3]) - center;
		up = up / up.norm();
		if (dot(up , normal) < 0) {
			auto tem = verts[0];
			verts[0] = verts[1];
			verts[1] = tem;
		}
		for (int i = 0; i < 4; i++) {
			TT(c.idx(), i) = verts[i].idx();
		}
	}
}


#endif // !_VOLUMEMATRIXCONVERTER
