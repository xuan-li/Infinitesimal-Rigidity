#ifndef _SURFACEMATRIXCONVERTER
#define _SURFACEMATRIXCONVERTER

#include "Eigen\Dense"

template <class Mesh> 
void SurfaceToMatrix(Mesh &mesh, Eigen::MatrixXd &V, Eigen::MatrixXi &F) 
{
	V.resize(mesh.n_vertices(), 3);
	F.resize(mesh.n_faces(), 3);
	for (Mesh::VertexIter viter = mesh.vertices_begin(); viter != mesh.vertices_end(); ++viter) {
		auto v = *viter;
		auto p = mesh.point(v);
		for (int i = 0; i < 3; i++) {
			V(v.idx(), i) = p[i];
		}
	}
	for (Mesh::FaceIter fiter = mesh.faces_begin(); fiter != mesh.faces_end(); fiter++) {
		auto f = *fiter;
		int i = 0;
		for (Mesh::FaceVertexIter fviter = mesh.fv_iter(f); fviter.is_valid(); fviter++) {
			auto v = *fviter;
			F(f.idx(), i) = v.idx();
			i++;
		}
	}
	
}

template <class Mesh>
void SurfaceToMatrix(Mesh &mesh, Eigen::MatrixXd &V, Eigen::VectorXi &Di, Eigen::MatrixXi &Fi) {
	int nv = mesh.n_vertices();
	int nf = mesh.n_faces();

	V.setZero();
	V.resize(nv, 3);

	/*load vertex data*/
	for (Mesh::VertexIter viter = mesh.vertices_begin(); viter != mesh.vertices_end(); ++viter) {
		auto v = *viter;

		int idx = v.idx();
		auto point = mesh.point(v);

		for (int i = 0; i < 3; i++) {
			V(idx, i) = point[i];
		}
	}

	Di.resize(nf);
	int max_fnv = 0;
	for (auto fiter = mesh.faces_begin(); fiter != mesh.faces_end(); ++fiter) {
		auto f = *fiter;
		int fnv = 0;
		for (auto fviter = mesh.fv_iter(f); fviter.is_valid(); ++fviter) {
			auto v = *fviter;
			fnv++;
		}
		Di(f.idx()) = fnv;
		max_fnv = fnv > max_fnv ? fnv : max_fnv;
	}

	Fi.resize(nf, max_fnv);
	Fi.setConstant(-1);
	for (auto fiter = mesh.faces_begin(); fiter != mesh.faces_end(); ++fiter) {
		auto f = *fiter;
		int i = 0;
		for (auto fviter = mesh.fv_iter(f); fviter.is_valid(); ++fviter) {
			auto v = *fviter;
			Fi(f.idx(), i) = v.idx();
			i++;
		}
	}
	//std::cout << Fi << std::endl;
}
#endif // !_SURFACEMATRIXCONVERTER
