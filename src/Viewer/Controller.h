#ifndef _CONTROLLER_MAIN_
#define _CONTROLLER_MAIN_

#include "ControllerRigidity.h"
#include "Common.h"
#include "ViewerData.h"
#include "SurfaceMatrixConverter.h"
#include "VolumeMatrixConverter.h"
#include "ViewerFlags.h"
#include "MergeMeshMatrix.h"

Rigidity::Controller controller;


void UpdateMeshData(igl::opengl::glfw::Viewer &viewer);

void LoadMeshFromFile() {
	std::string fname = igl::file_dialog_open();
	if (fname.length() == 0)
		return;
	size_t last_dot = fname.rfind('.');
	if (last_dot == std::string::npos)
	{
		printf("Error: No file extension found in %s\n", fname.c_str());
		return;
	}

	std::string extension = fname.substr(last_dot + 1);
	
	if (extension == "obj" || extension == "OBJ" || extension == "off" || extension == "OFF") {
		//controller.SurfaceMesh() = Surface::MeshSurface();
		controller.LoadMeshFile(fname);
		UpdateMeshData(viewer);
	}
}

void DrawMotion(igl::opengl::glfw::Viewer &viewer) {

	double span = V.col(0).maxCoeff() - V.col(0).minCoeff();

	Surface::MeshSurface &mesh = controller.SurfaceMesh();
	int n_motions = mesh.Motions(*(mesh.vertices_begin())).size();
	if (n_motions == 0) return;
	
	int n_verts = mesh.n_vertices();

	Eigen::MatrixXd lineV, lineTC;
	Eigen::MatrixXi lineT;
	Eigen::MatrixXd P1(n_verts,3), P2(n_verts, 3);
	Eigen::MatrixXd lineColors(n_verts, 3);
	lineColors.setZero();
	lineColors.col(0) = Eigen::VectorXd::Constant(lineColors.rows(), 1);

	for (auto viter = mesh.vertices_begin(); viter != mesh.vertices_end(); ++viter) {
		auto v = *viter;
		auto p = mesh.point(v);
		auto q = mesh.Motions(v)[motion_mode % mesh.Motions(v).size()];
		for (int i = 0; i < 3; i++) {
			P1(v.idx(), i) = p[i];
			P2(v.idx(), i) = p[i] + q[i];
		}
	}

		

	hedra::line_cylinders(
		P1,
		P2,
		span * 0.01,
		lineColors,
		50,
		lineV,
		lineT,
		lineTC);
	Merge::MergeMeshMatrix(bigV, bigT, bigTC, lineV, lineT, lineTC, bigV, bigT, bigTC);

	viewer.data().clear();
	viewer.data().set_mesh(bigV, bigT);
	viewer.data().set_colors(bigTC);

}

void UpdateMeshData(igl::opengl::glfw::Viewer &viewer) {
	
	if (ShowMesh == surface) {
		SurfaceToMatrix(controller.SurfaceMesh(), V, D, F);
		if (F.rows() == 0) return;
		UpdateMeshView(viewer, ShowPolygonalEdges, ShowVertexSpheres);
		if (ShowMotion)
			DrawMotion(viewer);
		
	}
	if (ShowMesh == triangulated_surface) {
		SurfaceToMatrix(controller.TriangulatedSurfaceMesh(), V, D, F);
		if (F.rows() == 0) return;
		UpdateMeshView(viewer, ShowPolygonalEdges, ShowVertexSpheres);
	}
	else if (ShowMesh == volume) {
		VolumeToMatrix(controller.VolumeMesh(), TV, TT);
		igl::barycenter(TV, TT, B);
		if (TT.rows() == 0) return;
		UpdateMeshView(viewer, TetMode);
	}
	
	
}



#endif // !_CONTROLLER

