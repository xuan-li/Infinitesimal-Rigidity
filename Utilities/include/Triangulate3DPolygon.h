/*
This tool is based on polypartition
*/

#ifndef _TRIANGULATE3DPOLYGON
#define _TRIANGULATE3DPOLYGON
 
#include <polypartition.h>
#include <Eigen\Dense>
#include <vector>
#include <map>
#include "assert.h"
#include <list>

namespace Triangulate {
	struct PolyVertex {
		PolyVertex(Eigen::Vector3d _p, int _id): point(_p), id(_id){ }
		int id;
		Eigen::Vector3d point;
		Eigen::Vector2d uv;
	};

	typedef std::vector<PolyVertex> PolyChain;

	class TPPLTriangulator {
	public:
		TPPLTriangulator();
		~TPPLTriangulator();

		void TriangulateMesh(const Eigen::MatrixXd& V,
							 const Eigen::VectorXi& D,
							 const Eigen::MatrixXi& F,
							 Eigen::MatrixXi& T,
							 Eigen::VectorXi& TF);

		std::vector<Eigen::Vector3i> TriangulateFace(const Eigen::MatrixXd &V, const Eigen::VectorXi &F);

		template <class Mesh, class FaceHandle>
		std::vector<Eigen::Vector3i> TriangulateFaceHandle(Mesh &mesh, FaceHandle f);

	protected:

		std::vector<PolyChain> chains;
		std::vector<bool> holes;
		
		void AddPolyChain(PolyChain polychain, bool hole);
		
		template <class Mesh, class FaceHandle>
		void AddPolyChain(Mesh & mesh, FaceHandle f, bool hole);

		std::vector<Eigen::VectorXi> Triangulate();


	protected: // following functions are used to rotate polygon to x-y plane
		Eigen::Vector3d translation;
		Eigen::Vector3d rotate_axis;
		double rotate_angle;

		void ComputeMap();
		
		Eigen::Vector2d Map(Eigen::Vector3d p);
		void ComputeUV();

		void ConvertToTPPLPolys(std::list<TPPLPoly> &polys);
		
		bool CheckPolygonNormal(TPPLPoly &poly);

		std::vector<Eigen::VectorXi> ConvertToEigen(list<TPPLPoly> &result);

		Eigen::Vector3d ComputeNormal(PolyChain &poly);

	};

	

	TPPLTriangulator::TPPLTriangulator() {

	}

	inline TPPLTriangulator::~TPPLTriangulator()
	{
	}

	inline void TPPLTriangulator::AddPolyChain(PolyChain polychain, bool hole)
	{
		chains.push_back(polychain);
		holes.push_back(hole);
	}

	inline std::vector<Eigen::VectorXi> TPPLTriangulator::Triangulate()
	{
		if (chains.size() == 1 && chains[0].size() == 3) {
			std::vector<Eigen::VectorXi> tri_result;
			tri_result.push_back(Eigen::Vector3i(chains[0][0].id, chains[0][1].id, chains[0][2].id));
			return tri_result;
		}
		ComputeMap();
		
		ComputeUV();

		TPPLPartition pp;

		list<TPPLPoly> testpolys, result;

		ConvertToTPPLPolys(testpolys);

		if (!pp.Triangulate_OPT(&(testpolys.front()), &result)) printf("Error\n");

		return ConvertToEigen(result);
	}
	 
	inline void TPPLTriangulator::TriangulateMesh(const Eigen::MatrixXd &V, const Eigen::VectorXi & D, const Eigen::MatrixXi & F, Eigen::MatrixXi & T, Eigen::VectorXi & TF)
	{
		vector<Eigen::Vector3i> NewTriangles;
		vector<int> RawTF;
		
		for (int i = 0; i < D.rows(); i++) {
			vector<Eigen::Vector3i> face_triangles = TriangulateFace(V, F.row(i).segment(0,D(i)));
			for (int j = 0; j < face_triangles.size(); j++) {
				RawTF.push_back(i);
				NewTriangles.push_back(face_triangles[j]);
			}
		}

		T.resize(NewTriangles.size(), 3);
		TF.resize(RawTF.size());
		for (int i = 0; i<NewTriangles.size(); i++) {
			T.row(i) = NewTriangles[i];
			TF(i) = RawTF[i];
		}
	}

	inline std::vector<Eigen::Vector3i> TPPLTriangulator::TriangulateFace(const Eigen::MatrixXd & V, const Eigen::VectorXi & F)
	{
		if (F.size() == 3)
			return std::vector<Eigen::Vector3i>({ F });
		chains.clear();
		holes.clear();

		vector<Eigen::Vector3i> NewTriangles;

		PolyChain poly;
		for (int CurrIndex = 0; CurrIndex < F.size(); CurrIndex++) {
			int id = F(CurrIndex);
			poly.push_back(PolyVertex(V.row(id), id));
		}
		AddPolyChain(poly, false);
		vector<Eigen::VectorXi> face_triangles = Triangulate();
		for (int j = 0; j < face_triangles.size(); j++) {
			Eigen::Vector3i face(face_triangles[j](0), face_triangles[j](1), face_triangles[j](2));
			NewTriangles.push_back(face);
		}
		return NewTriangles;
	}

	inline Eigen::Vector2d TPPLTriangulator::Map(Eigen::Vector3d p)
	{
		p = p + translation;
		Eigen::Vector3d img;
		double u = rotate_axis[0];
		double v = rotate_axis[1];
		double w = rotate_axis[2];
		double x = p[0];
		double y = p[1];
		double z = p[2];

		img[0] = u * (rotate_axis.dot(p)) * (1 - cos(rotate_angle)) + x * cos(rotate_angle) + (v * z - w * y) * sin(rotate_angle);
		img[1] = v * (rotate_axis.dot(p)) * (1 - cos(rotate_angle)) + y * cos(rotate_angle) + (w * x - u * z) * sin(rotate_angle);
		img[2] = w * (rotate_axis.dot(p)) * (1 - cos(rotate_angle)) + z * cos(rotate_angle) + (u * y - v * x) * sin(rotate_angle);

		//assert(abs(img[2]) < 1e-2);
		return Eigen::Vector2d(img[0], img[1]);
	}

	inline void TPPLTriangulator::ComputeUV()
	{
		// compute uv;
		for (int i = 0; i < chains.size(); i++) {
			for (int j = 0; j < chains[i].size(); j++) {
				PolyVertex &v = chains[i][j];
				v.uv = Map(v.point);
			}
		}
	}

	inline void TPPLTriangulator::ConvertToTPPLPolys(std::list<TPPLPoly>& polys)
	{
		for (int i = 0; i < chains.size(); i++) {
			TPPLPoly poly;
			poly.Init(chains[i].size());
			poly.SetHole(holes[i]);
			for (int j = 0; j < chains[i].size(); j++) {
				poly[j].x = chains[i][j].uv[0];
				poly[j].y = chains[i][j].uv[1];
				poly[j].id = chains[i][j].id;
			}

			if (i == 0) {
				if (!CheckPolygonNormal(poly)) {
					for (int i = 0; i < poly.GetNumPoints(); i++) {
						poly.GetPoint(i).x = -poly.GetPoint(i).x;
					}
				}
				assert(CheckPolygonNormal(poly));
			}
			else {
				if (CheckPolygonNormal(poly)) {
					for (int i = 0; i < poly.GetNumPoints(); i++) {
						poly.GetPoint(i).x = -poly.GetPoint(i).x;
					}
				}
			}

			polys.push_back(poly);
		}
		
		
	}

	inline bool TPPLTriangulator::CheckPolygonNormal(TPPLPoly & poly)
	{
		Eigen::Vector3d N(0, 0, 0);
		for (int i = 0; i < poly.GetNumPoints(); i++) {
			Eigen::Vector3d Vi(poly.GetPoint(i).x, poly.GetPoint(i).y, 0);
			Eigen::Vector3d Vj(poly.GetPoint((i + 1) % poly.GetNumPoints()).x, poly.GetPoint((i + 1) % poly.GetNumPoints()).y, 0 );
			N[0] += (Vi[1] - Vj[1]) * (Vi[2] + Vi[2]);
			N[1] += (Vi[2] - Vj[2]) * (Vi[0] + Vi[0]);
			N[2] += (Vi[0] - Vj[0]) * (Vi[1] + Vi[1]);
		}
		N /= N.norm();
		return N[2] > 0;
	}

	inline std::vector<Eigen::VectorXi> TPPLTriangulator::ConvertToEigen(std::list<TPPLPoly>& result)
	{
		std::vector<Eigen::VectorXi> result_partition;
		for (std::list<TPPLPoly>::iterator it = result.begin(); it != result.end(); ++it) {
			TPPLPoly &poly = *it;
			Eigen::VectorXi face(poly.GetNumPoints());
			for (int i = 0; i < face.size(); i++) {
				face[i] = poly[i].id;
			}
			result_partition.push_back(face);
		}
		return result_partition;
	}

	inline Eigen::Vector3d TPPLTriangulator::ComputeNormal(PolyChain & poly)
	{
		// based on https://groups.google.com/forum/#!topic/comp.graphics.algorithms/zS-15TF3Op8
		Eigen::Vector3d N(0,0,0);
		for (int i = 0; i < poly.size(); i++) {
			Eigen::Vector3d Vi = poly[i].point;
			Eigen::Vector3d Vj = poly[(i + 1) % poly.size()].point;
			N[0] += (Vi[1] - Vj[1]) * (Vi[2] + Vi[2]);
			N[1] += (Vi[2] - Vj[2]) * (Vi[0] + Vi[0]);
			N[2] += (Vi[0] - Vj[0]) * (Vi[1] + Vi[1]);
		}
		N /= N.norm();
		return N;
	}

	inline void TPPLTriangulator::ComputeMap()
	{
		Eigen::Vector3d p0 = chains[0][0].point;
		translation = -p0;

		Eigen::Vector3d normal = ComputeNormal(chains[0]);
		if (abs(normal[2] - 1.) < 1e-3) {
			rotate_axis = Eigen::Vector3d(0, 0, 1);
			rotate_angle = 0.;
			return;
		}

		else if (abs(normal[2] + 1.) < 1e-3) {
			rotate_axis = Eigen::Vector3d(1, 0, 0);
			rotate_angle = 3.141592653;
			return;
		}

		Eigen::Vector3d target = Eigen::Vector3d(0, 0, 1);

		rotate_axis = normal.cross(target);
		rotate_angle = acos(normal.dot(target));


	}
	
	template<class Mesh, class FaceHandle>
	inline std::vector<Eigen::Vector3i> TPPLTriangulator::TriangulateFaceHandle(Mesh & mesh, FaceHandle f)
	{
		chains.clear();
		holes.clear();
		AddPolyChain(mesh, f, false);
		vector<Eigen::Vector3i> NewTriangles;
		vector<Eigen::VectorXi> face_triangles = Triangulate();
		for (int j = 0; j < face_triangles.size(); j++) {
			Eigen::Vector3i face(face_triangles[j](0), face_triangles[j](1), face_triangles[j](2));
			NewTriangles.push_back(face);
		}
		return NewTriangles;
	}

	template<class Mesh, class FaceHandle>
	inline void TPPLTriangulator::AddPolyChain(Mesh & mesh, FaceHandle f, bool hole)
	{
		PolyChain poly;
		for (Mesh::FaceVertexIter fviter = mesh.fv_iter(f); fviter.is_valid(); ++fviter) {
			auto v = *fviter;
			auto p = mesh.point(v);
			int id = v.idx();
			poly.push_back(PolyVertex(Eigen::Vector3d(p[0], p[1], p[2]), id));
		}
		AddPolyChain(poly, hole);
	}

}

#endif // !_TRIANGULATE3DPOLYGON
