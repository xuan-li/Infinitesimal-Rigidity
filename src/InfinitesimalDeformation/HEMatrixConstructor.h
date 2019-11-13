#ifndef _HE_MATRIX_CONSTRUCTOR
#define _HE_MATRIX_CONSTRUCTOR

#include "AlgorithmCommon.h"

namespace Rigidity {
	
	class HEMatrixConstructor
	{
	public:
		HEMatrixConstructor(Volume::MeshVolume &volume);
		void ConstructHEMatrix(Eigen::SparseMatrix<double> &HE, Eigen::SparseMatrix<double> &N);
		
	protected: 
		Volume::MeshVolume* m_volume;
		std::vector<Eigen::Triplet<double>> HE_coefficients;
		std::vector<Eigen::Triplet<double>> N_coefficients;

	protected: // compute angles;
		
		void ComputeCornerAngle();
		void ComputeDihedralAngle();
		void ComputeDihedralAngleAroundVertex(Volume::CellHandle c, std::vector<Volume::VertexHandle> verts);
		double EuclideanCosineLaw(double a, double b, double c);
		double SphericalCosineLaw(double a, double b, double c);

	protected: // Compute derivatives.

		void ComputeDerivativesAdjacent();
		void ComputeDerivativesAdjacent(Volume::FaceHandle f);
		
		void ComputeDerivativesNonAdjacent();
		void ComputeDerivativesNonAdjacent(Volume::CellHandle c, std::vector<Volume::HalfEdgeHandle> edge_pair);
		
		void ComputeDerivativesSelf();
		double ComputeDerivativesSelf(Volume::HalfEdgeHandle h);
		double ComputeDerivativesSelfSingle(Volume::HalfFaceHandle f, Volume::HalfEdgeHandle h);

	protected: // Construct Matrix

		void ReIndexEdge();
		int n_variable;
		int n_non_changable;
		int n_changable;

	};

	HEMatrixConstructor::HEMatrixConstructor(Volume::MeshVolume &volume) : m_volume(&volume)
	{
		
	}

	inline void HEMatrixConstructor::ConstructHEMatrix(Eigen::SparseMatrix<double> &HE, Eigen::SparseMatrix<double> &N)
	{
		ReIndexEdge();
		ComputeCornerAngle();
		ComputeDihedralAngle();
		ComputeDerivativesAdjacent();
		ComputeDerivativesSelf();
		ComputeDerivativesNonAdjacent();
		//HE.resize(n_non_changable, n_variable);
		HE.resize(n_variable, n_variable);
		HE.setZero();
		N.resize(m_volume->n_edges(), n_variable);
		N.setZero();
		HE.setFromTriplets(HE_coefficients.begin(), HE_coefficients.end());
		N.setFromTriplets(N_coefficients.begin(), N_coefficients.end());
		
	}
	inline void HEMatrixConstructor::ComputeCornerAngle()
	{
		using namespace Volume;
		for (HalfFaceIter hfiter = m_volume->halffaces_begin(); hfiter != m_volume->halffaces_end(); ++hfiter) {
			HalfFaceHandle hf = *hfiter;
			CellHandle c = m_volume->incident_cell(hf);
			if (!c.is_valid()) continue;
			std::vector<VertexHandle> verts;
			for (HalfFaceVertexIter hfviter = m_volume->hfv_iter(hf); hfviter.valid(); ++hfviter) {
				verts.push_back(*hfviter);
			}
			
			for (int i = 0; i < 3; i++) {
				/*        i
				
				
 				      i+1     i+2
				*/ 
				HalfEdgeHandle he[3];
				he[0] = m_volume->halfedge(verts[(i+2) % 3], verts[i]);
				he[1] = m_volume->halfedge(verts[i], verts[(i + 1) % 3]);
				he[2] = m_volume->halfedge(verts[(i + 1) % 3], verts[(i + 2) % 3]);

				double l[3];
				l[0] = m_volume->length(m_volume->edge_handle(he[2]));
				l[1] = m_volume->length(m_volume->edge_handle(he[0]));
				l[2] = m_volume->length(m_volume->edge_handle(he[1]));
				double cs = EuclideanCosineLaw(l[0], l[1], l[2]);
				m_volume->CornerAngle(c, he[0]) =  cs;
				//std::cout << m_volume->CornerAngle(c, he[0]) << std::endl;
			}

		}

	}
	inline void HEMatrixConstructor::ComputeDihedralAngle()
	{
		using namespace Volume;

		for (CellIter citer = m_volume->cells_begin(); citer != m_volume->cells_end(); ++citer) {
			CellHandle c = *citer;
			std::vector<VertexHandle> verts = m_volume->CellVertices(c);
			/*                     v3
	                             / | \            
                                /  |  \
		                      v0---|---v2	
			                    \  |  /
			                     \ | /
								   v1
                             			
			*/ 
			std::vector<std::vector<VertexHandle>> cell_vertses;
			std::vector<VertexHandle> cell_verts;
			
			cell_verts.clear();
			cell_verts.push_back(verts[0]);
			cell_verts.push_back(verts[1]);
			cell_verts.push_back(verts[2]);
			cell_verts.push_back(verts[3]);
			cell_vertses.push_back(cell_verts);

			cell_verts.clear();
			cell_verts.push_back(verts[1]);
			cell_verts.push_back(verts[3]);
			cell_verts.push_back(verts[2]);
			cell_verts.push_back(verts[0]);
			cell_vertses.push_back(cell_verts);

			cell_verts.clear();
			cell_verts.push_back(verts[1]);
			cell_verts.push_back(verts[0]);
			cell_verts.push_back(verts[3]);
			cell_verts.push_back(verts[2]);
			cell_vertses.push_back(cell_verts);

			for (int i = 0; i < 3; i++) {
				ComputeDihedralAngleAroundVertex(c, cell_vertses[i]);
			}

		}

	}
	inline void HEMatrixConstructor::ComputeDihedralAngleAroundVertex(Volume::CellHandle c, std::vector<Volume::VertexHandle> verts)
	{
		using namespace Volume;

			/*                     v3
	                             / | \            
                                /  |  \
		                      v0---|---v2	
			                    \  |  /
			                     \ | /
								   v1

								  a_{01}
							   0---------1
                             	\	    /	
						  a_{20} \     / a_{12}
                                  \   /
								   \ /
								    2
			*/ 
		VertexHandle v0 = verts[0];
		VertexHandle v1 = verts[1];
		VertexHandle v2 = verts[2];
		VertexHandle v3 = verts[3];

		HalfEdgeHandle h0 = m_volume->halfedge(v0, v3);
		HalfEdgeHandle h1 = m_volume->halfedge(v1, v3);
		HalfEdgeHandle h2 = m_volume->halfedge(v2, v3);
		
		EdgeHandle e0 = m_volume->edge_handle(h0);
		EdgeHandle e1 = m_volume->edge_handle(h1);
		EdgeHandle e2 = m_volume->edge_handle(h2);

		double a01 = m_volume->CornerAngle(c, h0);
		double a12 = m_volume->CornerAngle(c, h1);
		double a20 = m_volume->CornerAngle(c, h2);

		double d0 = SphericalCosineLaw(a12, a01, a20);
		double d1 = SphericalCosineLaw(a20, a12, a01);
		double d2 = SphericalCosineLaw(a01, a20, a12);

		if (m_volume->CellEdgeTouched(c, e0)) {
			double ce0 = m_volume->DihedralAngle(c, e0);
			assert(abs(ce0 - d0) < 1e-7);
		}
		else
			m_volume->DihedralAngle(c, e0) = d0;
		
		if (m_volume->CellEdgeTouched(c, e1)) {
			double ce1 = m_volume->DihedralAngle(c, e1);
			assert(abs(ce1 - d1) < 1e-7);
		}
		else
			m_volume->DihedralAngle(c, e1) = d1;
		
		if (m_volume->CellEdgeTouched(c, e2)) {
			double ce2 = m_volume->DihedralAngle(c, e2);
			assert(abs(ce2 - d2) < 1e-7);
		}
		else
			m_volume->DihedralAngle(c, e2) = d2;
		
		
	}

	inline double HEMatrixConstructor::EuclideanCosineLaw(double a, double b, double c)
	{
		double cs = (b * b + c * c - a * a) / (2.0 * b * c);
		if (cs <= 1.0 && cs >= -1.0) {
			return acos(cs);
		}
		else {
			assert(false);
			return 0;
		}
	}
	inline double HEMatrixConstructor::SphericalCosineLaw(double a, double b, double c)
	{
		double cs = (cos(a) - cos(b) * cos(c)) / (sin(b) * sin(c));
		if (cs <= 1.0 && cs >= -1.0) {
			return acos(cs);
		}
		else {
			assert(false);
			return 0;
		}
	}

	inline void HEMatrixConstructor::ComputeDerivativesAdjacent()
	{
		using namespace Volume;

		for (FaceIter fiter = m_volume->faces_begin(); fiter != m_volume->faces_end(); ++fiter) {
			FaceHandle f = *fiter;	
			ComputeDerivativesAdjacent(f);
		}
	}
	inline void HEMatrixConstructor::ComputeDerivativesAdjacent(Volume::FaceHandle f)
	{
		

		/*          h1
		        <-------_
		         \  hf1 /|
				h2\    /h0  
				  _\| /
				     / 
		*/
		using namespace Volume;
		HalfFaceHandle hf1 = m_volume->halfface_handle(f, 0);
		HalfFaceHandle hf2 = m_volume->halfface_handle(f, 1);
		if (m_volume->is_boundary(hf1)) {
			hf1 = m_volume->halfface_handle(f, 1);
			hf2 = m_volume->halfface_handle(f, 0);
		}
		CellHandle c1 = m_volume->incident_cell(hf1);
		CellHandle c2;
		if (hf2.is_valid()) {
			c2 = m_volume->incident_cell(hf2);
		}

		auto halfedges = m_volume->HalfFaceHalfEdges(hf1);

		for (int i = 0; i < 3; i++) {
			HalfEdgeHandle h0 = halfedges[i];
			HalfEdgeHandle h1 = halfedges[(i + 1) % 3];
			HalfEdgeHandle h2 = halfedges[(i + 2) % 3];

			EdgeHandle e = m_volume->edge_handle(h1);
			double rho0 = m_volume->CornerAngle(c1, h0);
			double rho1 = m_volume->CornerAngle(c1, h1);
			double alpha1 = m_volume->DihedralAngle(c1, e);
			double alpha2 = 0;
			if(c2.is_valid())
				alpha2 = m_volume->DihedralAngle(c2, e);
			double l = m_volume->length(e);

			double d = 0;
			if(c2.is_valid())
				d =  (cos(alpha1) / sin(alpha1) + cos(alpha2) / sin(alpha2)) / (l * sin(rho0) * sin(rho1));
			else
				d = (cos(alpha1) / sin(alpha1)) / (l * sin(rho0) * sin(rho1));
			EdgeHandle e1 = m_volume->edge_handle(h0);
			EdgeHandle e2 = m_volume->edge_handle(h2);
			
			m_volume->Derivative(e1, e2) = d;
			m_volume->Derivative(e2, e1) = d;

			if (m_volume->IsVariable(e1)) {
				//if(!m_volume->IsChangable(e2))
					//HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->FunctionIndex(e2), m_volume->VariableIndex(e1), d));
				if (m_volume->IsVariable(e2))
					HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->VariableIndex(e2), m_volume->VariableIndex(e1), d));
				N_coefficients.push_back(Eigen::Triplet<double>(e2.idx(), m_volume->VariableIndex(e1), d));
			}

			if (m_volume->IsVariable(e2)) {
				/*if (!m_volume->IsChangable(e1))
					HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->FunctionIndex(e1), m_volume->VariableIndex(e2), d));
*/
				if (m_volume->IsVariable(e1))
					HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->VariableIndex(e1), m_volume->VariableIndex(e2), d));
				N_coefficients.push_back(Eigen::Triplet<double>(e1.idx(), m_volume->VariableIndex(e2), d));
			}

		}
	}
	inline void HEMatrixConstructor::ComputeDerivativesNonAdjacent()
	{
		using namespace Volume;
		for (CellIter citer = m_volume->cells_begin(); citer != m_volume->cells_end(); ++citer) {
			CellHandle c = *citer;
			auto verts = m_volume->CellVertices(c);
			ComputeDerivativesNonAdjacent(c, std::vector<HalfEdgeHandle>({ m_volume->halfedge(verts[1], verts[3]),
																		   m_volume->halfedge(verts[3], verts[2]),
																		   m_volume->halfedge(verts[2], verts[0]) }));
			ComputeDerivativesNonAdjacent(c, std::vector<HalfEdgeHandle>({ m_volume->halfedge(verts[2], verts[3]),
																		   m_volume->halfedge(verts[3], verts[0]), 
																		   m_volume->halfedge(verts[0], verts[1]) }));
			ComputeDerivativesNonAdjacent(c, std::vector<HalfEdgeHandle>({ m_volume->halfedge(verts[0], verts[3]),
																		   m_volume->halfedge(verts[3], verts[1]),
																		   m_volume->halfedge(verts[1], verts[2]) }));
		}
	}
	inline void HEMatrixConstructor::ComputeDerivativesNonAdjacent(Volume::CellHandle c, std::vector<Volume::HalfEdgeHandle> halfedge_path)
	{
		using namespace Volume;
		HalfEdgeHandle start = halfedge_path[0];
		HalfEdgeHandle bridge = halfedge_path[1];
		HalfEdgeHandle end = halfedge_path[2];
		
		double sb = m_volume->CornerAngle(c, start);
		double sgamma = m_volume->DihedralAngle(c, m_volume->edge_handle(bridge));
		double d1 = 1 / (sin(sb) * sin(sgamma));
		
		double eb = m_volume->length(m_volume->edge_handle(bridge));
		double egamma = m_volume->CornerAngle(c, m_volume->opposite_halfedge_handle(end));
		double d2 = 1 / (eb * sin(egamma));

		double d = - d1 * d2;

		EdgeHandle e1 = m_volume->edge_handle(start);
		EdgeHandle e2 = m_volume->edge_handle(end);
		m_volume->Derivative(e1, e2) = d;
		m_volume->Derivative(e2, e1) = d;

		if (m_volume->IsVariable(e1)) {
			/*if (!m_volume->IsChangable(e2))
				HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->FunctionIndex(e2), m_volume->VariableIndex(e1), d));*/
			if (m_volume->IsVariable(e2))
				HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->VariableIndex(e2), m_volume->VariableIndex(e1), d));
			N_coefficients.push_back(Eigen::Triplet<double>(e2.idx(), m_volume->VariableIndex(e1), d));
		}

		if (m_volume->IsVariable(e2)) {
			/*if (!m_volume->IsChangable(e1))
				HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->FunctionIndex(e1), m_volume->VariableIndex(e2), d));*/
			if (m_volume->IsVariable(e1))
				HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->VariableIndex(e1), m_volume->VariableIndex(e2), d));
			N_coefficients.push_back(Eigen::Triplet<double>(e1.idx(), m_volume->VariableIndex(e2), d));
		}
	}

	inline void HEMatrixConstructor::ComputeDerivativesSelf()
	{
		using namespace Volume;
		for (EdgeIter eiter = m_volume->edges_begin(); eiter != m_volume->edges_end(); ++eiter) {
			EdgeHandle e = *eiter;
			HalfEdgeHandle h1 = m_volume->halfedge_handle(e, 0);
			HalfEdgeHandle h2 = m_volume->halfedge_handle(e, 1);
			double d1 = ComputeDerivativesSelf(h1);
			double d2 = ComputeDerivativesSelf(h2);
			assert(abs(d1 - d2) < 1e-7);
			m_volume->Derivative(e, e) = d1;

			if (m_volume->IsVariable(e)) {
				//HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->FunctionIndex(e), m_volume->VariableIndex(e), d1));
				HE_coefficients.push_back(Eigen::Triplet<double>(m_volume->VariableIndex(e), m_volume->VariableIndex(e), d1));
				N_coefficients.push_back(Eigen::Triplet<double>(e.idx(), m_volume->VariableIndex(e), d1));
			}
		}
	}
	inline double HEMatrixConstructor::ComputeDerivativesSelf(Volume::HalfEdgeHandle h)
	{
		using namespace Volume;
		double d = 0;
		for (HalfEdgeHalfFaceIter hehfiter = m_volume->hehf_iter(h); hehfiter.valid(); ++hehfiter) {
			HalfFaceHandle hf = *hehfiter;
			d += ComputeDerivativesSelfSingle(hf, h);
		}
		return d;
	}
	inline double HEMatrixConstructor::ComputeDerivativesSelfSingle(Volume::HalfFaceHandle hf, Volume::HalfEdgeHandle he)
	{
		using namespace Volume;
		if (!m_volume->is_boundary(hf)) {
			HalfEdgeHandle next_he = m_volume->next_halfedge_in_halfface(he, hf);
			CellHandle c = m_volume->incident_cell(hf);
			double phi = m_volume->CornerAngle(c, he);
			return -cos(phi) * m_volume->Derivative(m_volume->edge_handle(he), m_volume->edge_handle(next_he));
		}
		else {
			hf = m_volume->opposite_halfface_handle(hf);
			HalfEdgeHandle next_he = m_volume->opposite_halfedge_handle(he);
			he = m_volume->prev_halfedge_in_halfface(next_he, hf);
			CellHandle c = m_volume->incident_cell(hf);
			double phi = m_volume->CornerAngle(c, he);
			return -cos(phi) * m_volume->Derivative(m_volume->edge_handle(he), m_volume->edge_handle(next_he)); 
		}
	}
	
	inline void HEMatrixConstructor::ReIndexEdge()
	{
		using namespace Volume;
		int variable_index = 0;
		int changable_edge_index = 0;
		int not_changable_edge_index = 0;
		for (EdgeIter eiter = m_volume->edges_begin(); eiter != m_volume->edges_end(); ++eiter) {
			EdgeHandle e = *eiter;
			if (m_volume->IsVariable(e)) {
				m_volume->VariableIndex(e) = variable_index;
				++variable_index;
			}


			if (m_volume->IsChangable(e)) {
				m_volume->FunctionIndex(e) = changable_edge_index;
				++changable_edge_index;
			}
			else{
				m_volume->FunctionIndex(e) = not_changable_edge_index;
				++not_changable_edge_index;
			}
		}
		n_variable = variable_index;
		n_non_changable = not_changable_edge_index;
		n_changable = changable_edge_index;
	}
} // end of namespace Rigidity

#endif // !_HE_MATRIX_CONSTRUCTOR
