#ifndef _MOTION_SOLVER
#define _MOTION_SOLVER

#include "AlgorithmCommon.h"

namespace Rigidity {
	class MotionSolver {
	public:
		MotionSolver(Surface::MeshSurface &surface, Volume::MeshVolume &volume, Eigen::MatrixXd &kernal);
		void ComputeVertexMotions();
	protected:
		Surface::MeshSurface *m_surface;
		Volume::MeshVolume* m_volume;
		Eigen::MatrixXd m_kernal;

		Volume::EdgePropertyT<char> touched = m_volume->request_edge_property<char>("", false);
	
	
	protected:
		void ClearMotions();
		void ComputeMotion(Eigen::VectorXd solution);
        void ConstructLinearSystem(std::vector<Volume::VertexHandle> vert_pair, Eigen::VectorXd &solution, Eigen::MatrixXd & A, Eigen::VectorXd & b);
        void ConstructLinearSystem(Eigen::VectorXd solution, Eigen::MatrixXd &A, Eigen::VectorXd &b);
		void WriteToVertices(Eigen::VectorXd motion);
		void CheckMotions();
	
	};

	MotionSolver::MotionSolver(Surface::MeshSurface &surface, Volume::MeshVolume &volume, Eigen::MatrixXd &kernal) :m_surface(&surface), m_volume(&volume), m_kernal(kernal) {

	}
	inline void MotionSolver::ComputeVertexMotions()
	{
		ClearMotions();
		for (int i = 0; i < m_kernal.cols(); ++i) {
			//std::cout << std::endl << m_kernal.col(i) << std::endl;
			ComputeMotion(m_kernal.col(i));
		}
		//CheckMotions();
	}


	inline void MotionSolver::ClearMotions()
	{
		using namespace Surface;
		for (MeshSurface::VertexIter viter = m_surface->vertices_begin(); viter != m_surface->vertices_end(); ++viter) {
			VertexHandle v = *viter;
			m_surface->LastMotions(v) = m_surface->Motions(v);
			m_surface->Motions(v).clear();
		}
	}

	inline void MotionSolver::ComputeMotion(Eigen::VectorXd solution)
	{
		Eigen::MatrixXd A(m_volume->n_edges() + 9, 3 * m_volume->n_vertices());
		Eigen::VectorXd b(m_volume->n_edges() + 9);
		A.setZero();
		b.setZero();
		Eigen::VectorXd motion;
		ConstructLinearSystem(solution, A, b);
		//if (b.norm() < EIGEN_THRESHOLD) {
		//	Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
		//	lu.setThreshold(EIGEN_THRESHOLD);
		//	Eigen::MatrixXd solution = lu.kernel();
		//	motion = solution.col(0);
		//}
		//else
		//{
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
			//svd.setThreshold(EIGEN_THRESHOLD);
			motion = svd.solve(b);
		/*}*/
		
		double error = (A * motion - b).norm();
		assert(error <EIGEN_THRESHOLD);
		//std::cout << std::endl << motion << std::endl;
		//std::cout << std::endl<< motion<<std::endl;
		WriteToVertices(motion);
	}

    inline void MotionSolver::ConstructLinearSystem(std::vector<Volume::VertexHandle> vert_pair, Eigen::VectorXd &solution, Eigen::MatrixXd & A, Eigen::VectorXd & b)
    {
        using namespace Volume;
        HalfEdgeHandle he = m_volume->halfedge(vert_pair[0], vert_pair[1]);
        EdgeHandle e = m_volume->edge_handle(he);
        if (touched[e]) return;
        else touched[e] = true;
        double dl;
        if (m_volume->IsVariable(e))
            dl = solution(m_volume->VariableIndex(e));
        else
            dl = 0;

        Vec3d dpoint = m_volume->vertex(vert_pair[1]) - m_volume->vertex(vert_pair[0]);
        dpoint = dpoint / dpoint.norm();

        b(e.idx()) = dl;

        for (int i = 0; i < 3; i++) {
            A(e.idx(), 3 * vert_pair[1].idx() + i) = dpoint[i];
            A(e.idx(), 3 * vert_pair[0].idx() + i) = -dpoint[i];
        }
    }

	inline void MotionSolver::ConstructLinearSystem(Eigen::VectorXd solution, Eigen::MatrixXd & A, Eigen::VectorXd & b)
	{
		/*                       _  
		                         /| q2 - q1
		                        /
			o----- p2 - p1 ---->
		
		The equation for one edge is:
			<q2 - q1, normalize(p2 - p1)> = dl
		
		*/
		
		using namespace Volume;

		for (EdgeIter eiter = m_volume->edges_begin(); eiter != m_volume->edges_end(); ++eiter) {
			EdgeHandle e = *eiter;
			touched[e] = false;
		}

		FaceHandle f;
		//for (HalfFaceIter hfiter = m_volume->halffaces_begin(); hfiter != m_volume->halffaces_end(); ++hfiter) {
		//	auto hs = m_volume->HalfFaceHalfEdges(*hfiter);
		//	for (int i = 0; i < 3; i++) {
		//		if(hs)
		//	}
		//}
		for (FaceIter fiter = m_volume->faces_begin(); fiter != m_volume->faces_end(); ++fiter) {
			if (m_volume->is_boundary(*fiter)) {
				f = *fiter;
				break;
			}

		}
		
		std::vector<VertexHandle> root_verts;
		HalfFaceHandle hf = m_volume->halfface_handle(f, 0);
		//Vec3d normal = m_volume->Normal(hf);

		for (HalfFaceVertexIter hfviter = m_volume->hfv_iter(hf); hfviter.valid(); ++hfviter) {
			root_verts.push_back(*hfviter);
		}
		//EdgeHandle e = m_volume->edge_handle(m_volume->halfedge(root_verts[0], root_verts[1]));
		
		//Vec3d dpoint1 = m_volume->vertex(root_verts[1]) - m_volume->vertex(root_verts[0]);
		//dpoint1 = dpoint1 / dpoint1.norm();
		//double dl;
		//if (m_volume->IsVariable(e))
		//	dl = solution(m_volume->VariableIndex(e));
		//else
		//	dl = 0;

		A(A.rows() - 9, 3 * root_verts[0].idx()) = 1;
		b(A.rows() - 9) = 0;
		A(A.rows() - 8, 3 * root_verts[0].idx() + 1) = 1;
		b(A.rows() - 8) = 0;
		A(A.rows() - 7, 3 * root_verts[0].idx() + 2) = 1;
		b(A.rows() - 7) = 0;

		A(A.rows() - 6, 3 * root_verts[1].idx()) = 1;
		b(A.rows() - 6) = 0;
		A(A.rows() - 5, 3 * root_verts[1].idx() + 1) = 1;
		b(A.rows() - 5) = 0;
		A(A.rows() - 4, 3 * root_verts[1].idx() + 2) = 1;
		b(A.rows() - 4) = 0;

		A(A.rows() - 3, 3 * root_verts[2].idx()) = 1;
		b(A.rows() - 3) = 0;
		A(A.rows() - 2, 3 * root_verts[2].idx() + 1) = 1;
		b(A.rows() - 2) = 0;
		A(A.rows() - 1, 3 * root_verts[2].idx() + 2) = 1;
		b(A.rows() - 1) = 0;
		
		/*A(A.rows() - 4, 3 * root_verts[1].idx()) = 1;
		b(A.rows() - 4) = dl * dpoint1[0];
		A(A.rows() - 3, 3 * root_verts[1].idx() + 1) = 1;
		b(A.rows() - 3) = dl * dpoint1[1];
		A(A.rows() - 2, 3 * root_verts[1].idx() + 2) = 1;
		b(A.rows() - 2) = dl * dpoint1[2];

		A(A.rows() - 1, 3 * root_verts[2].idx()) = normal[0];
		A(A.rows() - 1, 3 * root_verts[2].idx() + 1) = normal[1];
		A(A.rows() - 1, 3 * root_verts[2].idx() + 2) = normal[2];
		b(A.rows() - 1) = 0;*/

		//std::cout << std::endl<<b << std::endl;
		for (CellIter citer = m_volume->cells_begin(); citer != m_volume->cells_end(); ++citer) {
			CellHandle c = *citer;
			auto verts = m_volume->CellVertices(c);
			ConstructLinearSystem({ verts[0], verts[1] }, solution, A, b);
			ConstructLinearSystem({ verts[0], verts[2] }, solution, A, b);
			ConstructLinearSystem({ verts[0], verts[3] }, solution, A, b);
			ConstructLinearSystem({ verts[1], verts[2] }, solution, A, b);
			ConstructLinearSystem({ verts[1], verts[3] }, solution, A, b);
			ConstructLinearSystem({ verts[2], verts[3] }, solution, A, b);
		}

		//std::cout << std::endl << A << std::endl;
		//std::cout << std::endl << b << std::endl;

	}



	inline void MotionSolver::WriteToVertices(Eigen::VectorXd motion)
	{
		using namespace Surface;
		for (MeshSurface::VertexIter viter = m_surface->vertices_begin(); viter != m_surface->vertices_end(); ++viter) {
			VertexHandle v = *viter;
			Vec3d direction;
			direction[0] = motion(3 * v.idx());
			direction[1] = motion(3 * v.idx() + 1);
			direction[2] = motion(3 * v.idx() + 2);

			/*Vec3d last_direction;
			if (m_surface->LastMotions(v).size() == 0)
				last_direction = Vec3d(0, 0, 0);
			else
				last_direction = m_surface->LastMotions(v)[m_surface->Motions(v).size()];
			double diff1 = (direction - last_direction).norm();
			double diff2 = (direction + last_direction).norm();
			m_surface->Motions(v).push_back((diff1 < diff2 ? direction : -direction));*/
				m_surface->Motions(v).push_back(direction);
		}
	}

	inline void MotionSolver::CheckMotions()
	{
		using namespace Surface;
		for (int i = 0; i < m_kernal.cols(); i++) {
			for (MeshSurface::EdgeIter eiter = m_surface->edges_begin(); eiter != m_surface->edges_end(); ++eiter) {
				EdgeHandle e = *eiter;
				HalfedgeHandle h = m_surface->halfedge_handle(e, 0);
				VertexHandle v1 = m_surface->from_vertex_handle(h);
				VertexHandle v2 = m_surface->to_vertex_handle(h);
				Vec3d p1 = m_surface->point(v1);
				Vec3d p2 = m_surface->point(v2);
				Vec3d q1 = m_surface->Motions(v1)[i];
				Vec3d q2 = m_surface->Motions(v2)[i];
				double test = (q2 - q1) | (p2 - p1);
				assert(test < 10 * EIGEN_THRESHOLD);
			}
		}
	}	
}

#endif // !_MOTION_SOLVER
