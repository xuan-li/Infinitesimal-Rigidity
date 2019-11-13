#ifndef _CONTROLLER_RIGIDITY
#define _CONTROLLER_RIGIDITY

#include "Preprocessor.h"
#include "HEMatrixConstructor.h"
#include "KernalSolver.h"
#include "MotionSolver.h"



namespace Rigidity {
	class Controller {
	public:
		Controller();
		Surface::MeshSurface& SurfaceMesh() { return m_surface; }
		Surface::MeshSurface& TriangulatedSurfaceMesh() { return m_triangulated_surface; }
		Volume::MeshVolume& VolumeMesh() { return m_volume; }
		void LoadMeshFile(std::string name);
		void Triangularize();
		void Tetrahedralize();
		void ComputeHE();
		void ComputeKernal();
		void ComputeMotion();
		void Deformation();

		// flag
		bool ComputationDone = false;

	protected:
		Surface::MeshSurface m_surface;
		Surface::MeshSurface m_triangulated_surface;
		Volume::MeshVolume m_volume;
		Eigen::SparseMatrix<double> HE;
		Eigen::SparseMatrix<double> N;
		bool rigidity = false;
		int he_dimension = 0;
		int n_dimension = 0;

	protected:
		Eigen::MatrixXd non_trival_kernal;
	};

	Controller::Controller() {

	}

	inline void Controller::LoadMeshFile(std::string name)
	{
		m_surface = Surface::MeshSurface();
		
		m_surface.ReadMeshFile(name);
		ComputationDone = false;
	}

	void Controller::Tetrahedralize()
	{
		//
		
		Preprocessor preprocessor(m_triangulated_surface, m_volume);
		preprocessor.Tetrahedralize();
	}

	inline void Controller::Triangularize()
	{
		rigidity = false;
		m_triangulated_surface = m_surface;
		Preprocessor preprocessor(m_triangulated_surface, m_volume);
		preprocessor.Triangularize();
		
	}

	inline void Controller::ComputeHE()
	{
		HEMatrixConstructor hecomputer(m_volume);
		hecomputer.ConstructHEMatrix(HE, N);
		//std::cout <<std::endl<< HE << std::endl;
		//std::cout << std::endl << N << std::endl;

		std::cout << std::endl;
		std::cout << m_surface.n_edges() << " Outter Edges\n";
		std::cout << m_triangulated_surface.n_edges() - m_surface.n_edges() << " Added Outter Edges\n";
		std::cout << m_volume.n_vertices() - m_surface.n_vertices() << " Steinner Points\n";
		std::cout << HE.cols() << " Variables\n";
		std::cout << m_volume.n_edges() << " Edges\n";
		std::cout << HE.rows() << " Non-Changable Edges\n";
		std::cout << N.rows() - HE.rows() << " Changable Edges\n";

		Eigen::MatrixXd d_HE(HE);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd1(d_HE, Eigen::ComputeThinU | Eigen::ComputeThinV);
		svd1.setThreshold(EIGEN_THRESHOLD);
		Eigen::VectorXd singualar_values = svd1.singularValues();

		he_dimension = 0;
		for (int i = 0; i < singualar_values.size(); i++) {
			if (singualar_values(i) < EIGEN_THRESHOLD)
				++he_dimension;
		}

		if (he_dimension == 3 *(m_volume.n_vertices() - m_surface.n_vertices()))
			rigidity = true;

		Eigen::MatrixXd d_N(N);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd2(d_N, Eigen::ComputeThinU | Eigen::ComputeThinV);
		svd2.setThreshold(EIGEN_THRESHOLD);
		singualar_values = svd2.singularValues();
		
		n_dimension = 0;
		for (int i = 0; i < singualar_values.size(); i++) {
			if (singualar_values(i) < EIGEN_THRESHOLD)
				++n_dimension;
		}

		std::cout << "Dimension of infinititesimal motion: " << he_dimension << std::endl;
		std::cout << "Dimension of trival infinititesimal motion: " << n_dimension << std::endl;
		std::cout << "Whether Infinititesimally Rigid: " << rigidity << std::endl;
		
	}

	inline void Controller::ComputeKernal()
	{
		if (rigidity) return;
		KernalSolver solver(HE, N);
		non_trival_kernal = solver.ComputeNonTrivalKernal();
	}

	inline void Controller::ComputeMotion()
	{
		Triangularize();
		Tetrahedralize();
		ComputeHE();
		if (rigidity) return;
		ComputeKernal();
		MotionSolver motion_solver(m_surface, m_volume, non_trival_kernal);
		motion_solver.ComputeVertexMotions();
		ComputationDone = true;
	}

	inline void Controller::Deformation()
	{
		ComputeMotion();
		if (rigidity) return;
		
		using namespace Surface;
		for (int i = 0; i < 100; ++i) {
			for (MeshSurface::VertexIter viter = m_surface.vertices_begin(); viter != m_surface.vertices_end(); ++viter) {
				VertexHandle v = *viter;
				m_surface.point(v) += 0.0001 * m_surface.Motions(v)[0];
				//std::cout << m_surface.Motions(v)[0] << std::endl;
				
			}
			ComputeMotion();
			if (rigidity) return;
		}
		

	}

} // end of namespace Rigidity

#endif // !_CONTROLLER_RIGIDITY
