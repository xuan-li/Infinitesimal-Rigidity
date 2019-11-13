#ifndef _TETRAHEDRALIZE
#define _TETRAHEDRALIZE

#include <igl/copyleft/tetgen/tetrahedralize.h>
#include "SurfaceMatrixConverter.h"
#include "VolumeMatrixConverter.h"

template <class TriMesh, class VolumeMesh>
void Tetrahedralize(TriMesh &tri_mesh, VolumeMesh &volume_mesh) {

	//volume_mesh = VolumeMesh();
	// Input polygon
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

	// Tetrahedralized interior
	Eigen::MatrixXd TV;
	Eigen::MatrixXi TT;
	Eigen::MatrixXi TF;

	SurfaceToMatrix(tri_mesh, V, F);
	igl::copyleft::tetgen::tetrahedralize(V, F, "pqY", TV, TT, TF);
	MatrixToVolume(volume_mesh, TV, TT);

	//std::cout << std::endl << V << std::endl;
	//std::cout << std::endl << TV << std::endl;

}



#endif // !_TETRAHEDRALIZE
