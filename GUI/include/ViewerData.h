#ifndef _POLYMESH_DATA
#define _POLYMESH_DATA

#include "Common.h"
#include "hedra/polygonal_write_OFF.h"
#include "Triangulate3DPolygon.h"
#include "MergeMeshMatrix.h"

// global data for viewer to visualize

//Input Mesh

	Eigen::MatrixXd V;
	Eigen::VectorXi D;
	Eigen::MatrixXi F;

	// triangulated mesh data
	Eigen::MatrixXd FEs;
	Eigen::MatrixXi EV, EF, FE, T, EFi;
	Eigen::VectorXi TF, innerEdges;
	Eigen::MatrixXd TC;

	Eigen::MatrixXd sphereColors, lineColors;
	Eigen::MatrixXd faceCenters, faceNormals;

	// Visual Data
	Eigen::MatrixXd bigV;
	Eigen::MatrixXi bigT;
	Eigen::MatrixXd bigTC; // color

	Eigen::MatrixXd TV;
	Eigen::MatrixXi TT;
	Eigen::MatrixXd B;

	void UpdateMeshView(igl::viewer::Viewer& viewer, bool ShowPolygonalEdges, bool ShowVertexSpheres)
	{
		if (V.cols() != 3) {
			return;
		}

		Triangulate::TPPLTriangulator triangulator;
		triangulator.TriangulateMesh(V, D, F, T, TF);
		hedra::polygonal_edge_topology(D, F, EV, FE, EF, EFi, FEs, innerEdges);
		hedra::polygonal_face_centers(V, D, F, faceCenters);
		hedra::polyhedral_face_normals(V, D, F, faceNormals);

		lineColors.resize(T.rows(), 3);
		lineColors.col(0) = Eigen::VectorXd::Constant(T.rows(), 0.5);
		lineColors.col(1) = Eigen::VectorXd::Constant(T.rows(), 0.5);
		lineColors.col(2) = Eigen::VectorXd::Constant(T.rows(), 1.0);

		TC.resize(T.rows(), 3);
		TC.col(0) = Eigen::VectorXd::Constant(T.rows(), 0.0);
		TC.col(1) = Eigen::VectorXd::Constant(T.rows(), 0.5);
		TC.col(2) = Eigen::VectorXd::Constant(T.rows(), 0.5);

		viewer.data.clear();
		viewer.data.set_face_based(true);

		Eigen::MatrixXd sphereV, lineV, sphereTC, lineTC;
		Eigen::MatrixXi sphereT, lineT;

		if (ShowVertexSpheres) {
			sphereColors.resize(V.rows(), 3);
			for (int i = 0; i < V.rows(); i++) {
				sphereColors.row(i) << (double)i / (double)V.rows(), 1.0 - (double)i / (double)V.rows(), 0.0;
			}
			hedra::point_spheres(V, 0.02, sphereColors, 10, false, sphereV, sphereT, sphereTC);
		}


		bigV.resize(V.rows() + sphereV.rows() + lineV.rows(), 3);
		bigT.resize(T.rows() + sphereT.rows() + lineT.rows(), 3);
		bigTC.resize(TC.rows() + sphereTC.rows() + lineTC.rows(), 3);

		bigV.block(0, 0, V.rows(), 3) = V;
		bigT.block(0, 0, T.rows(), 3) = T;
		bigTC.block(0, 0, TC.rows(), 3) = TC;
		if (sphereV.rows() != 0) {
			std::cout << "Showing vertex spheres" << std::endl;
			bigV.block(V.rows(), 0, sphereV.rows(), 3) = sphereV;
			bigT.block(T.rows(), 0, sphereT.rows(), 3) = sphereT + Eigen::MatrixXi::Constant(sphereT.rows(), sphereT.cols(), V.rows());
			bigTC.block(TC.rows(), 0, sphereTC.rows(), 3) = sphereTC;
		}

		viewer.data.clear();
		if (ShowPolygonalEdges) {
			viewer.core.show_lines = false;
			std::cout << "Showing polygonal lines" << std::endl;
			Eigen::MatrixXd OrigEdgeColors(EV.rows(), 3);
			OrigEdgeColors.col(0) = Eigen::VectorXd::Constant(EV.rows(), 0.0);
			OrigEdgeColors.col(1) = Eigen::VectorXd::Constant(EV.rows(), 0.0);
			OrigEdgeColors.col(2) = Eigen::VectorXd::Constant(EV.rows(), 0.0);
			viewer.data.set_edges(V, EV, OrigEdgeColors);
		}
		else {
			viewer.core.show_lines = true;
		}

		viewer.data.set_mesh(bigV, bigT);
		viewer.data.set_colors(bigTC);
	}

	void UpdateMeshView(igl::viewer::Viewer& viewer, char key) {
		
		int mode = double(key - '1' + 1);
		if (mode > 9) return;
		double t = 0;
		if (mode == 0)
			t = 1.1;
		else
			t = mode / 9.0;

		Eigen::VectorXd v = B.col(2).array() - B.col(2).minCoeff();
		v /= v.col(0).maxCoeff();

		vector<int> s;

		for (unsigned i = 0; i<v.size(); ++i)
			if (v(i) < t)
				s.push_back(i);

		Eigen::MatrixXd V_temp(s.size() * 4, 3);
		Eigen::MatrixXi F_temp(s.size() * 4, 3);

		for (unsigned i = 0; i<s.size(); ++i)
		{
			V_temp.row(i * 4 + 0) = TV.row(TT(s[i], 0));
			V_temp.row(i * 4 + 1) = TV.row(TT(s[i], 1));
			V_temp.row(i * 4 + 2) = TV.row(TT(s[i], 2));
			V_temp.row(i * 4 + 3) = TV.row(TT(s[i], 3));
			F_temp.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
			F_temp.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
			F_temp.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
			F_temp.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;
		}

		viewer.data.clear();
		viewer.data.set_mesh(V_temp, F_temp);
		viewer.data.set_face_based(true);
	}

#endif // !MeshIO
