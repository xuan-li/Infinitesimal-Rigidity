#ifndef _KERNAL_SOLVER
#define _KERNAL_SOLVER
#include "AlgorithmCommon.h"

namespace Rigidity {
	class KernalSolver {
	public:
		KernalSolver(Eigen::SparseMatrix<double> &_HE, Eigen::SparseMatrix<double> &_N);
		Eigen::MatrixXd ComputeNonTrivalKernal();
	protected:
		Eigen::MatrixXd HE;
		Eigen::MatrixXd N;
		Eigen::MatrixXd infinititesimal_motions;
		Eigen::MatrixXd trival_motions;
		Eigen::MatrixXd non_trival_motions;

	protected:
		void ComputeKernal();
		void Orthogonalize();
		void RemoveProjection(Eigen::VectorXd &v1, Eigen::VectorXd &v2);
		void ComputeNonTrivalMotions();
		
	
	};

	KernalSolver::KernalSolver(Eigen::SparseMatrix<double> &_HE, Eigen::SparseMatrix<double> &_N):HE(_HE), N(_N)
	{

	}

	inline Eigen::MatrixXd KernalSolver::ComputeNonTrivalKernal()
	{
		ComputeKernal();
		Orthogonalize();
		ComputeNonTrivalMotions();
		return non_trival_motions;
	}

	inline void KernalSolver::ComputeKernal()
	{
		Eigen::FullPivLU<Eigen::MatrixXd> lu1(HE);
		lu1.setThreshold(EIGEN_THRESHOLD);
		infinititesimal_motions = lu1.kernel();

		Eigen::FullPivLU<Eigen::MatrixXd> lu2(N);
		lu2.setThreshold(EIGEN_THRESHOLD);
		trival_motions = lu2.kernel();


	}

	inline void KernalSolver::Orthogonalize()
	{
		std::cout << "Orthogonalize Null Space\n";
		for (int i = 0; i < infinititesimal_motions.cols(); ++i) {
			Eigen::VectorXd motion = infinititesimal_motions.col(i);
			for (int j = 0; j < i; j++) {
				Eigen::VectorXd prev = infinititesimal_motions.col(j);
				RemoveProjection(motion, prev);
			}
			infinititesimal_motions.col(i) = motion / motion.norm();
		}
	}

	inline void KernalSolver::RemoveProjection(Eigen::VectorXd & v1, Eigen::VectorXd & v2)
	{
		double coord = double(v1.transpose() * v2) / double(v2.transpose() * v2);
		v1 = v1 - coord * v2;
		assert(double(v1.transpose() * v2) < EIGEN_THRESHOLD);
	}

	inline void KernalSolver::ComputeNonTrivalMotions()
	{
		if (trival_motions.cols() == 1 && trival_motions.col(0).norm() < EIGEN_THRESHOLD) {
			non_trival_motions = infinititesimal_motions;
			//std::cout << non_trival_motions << std::endl;
			return;
		}
		Eigen::FullPivLU<Eigen::MatrixXd> lu(infinititesimal_motions);
		lu.setThreshold(EIGEN_THRESHOLD);
		Eigen::MatrixXd coords = lu.solve(trival_motions);

		Eigen::MatrixXd coords_T = coords.transpose();
		//std::cout << coords_T;
		Eigen::FullPivLU<Eigen::MatrixXd> lu2(coords_T);
		lu2.setThreshold(EIGEN_THRESHOLD);
		Eigen::MatrixXd otho_coords = lu2.kernel();
		
		non_trival_motions = infinititesimal_motions * otho_coords;
		
		
	}

	
}

#endif