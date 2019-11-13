#ifndef _MESH_VOLUME
#define _MESH_VOLUME
#define INCLUDE_TEMPLATES
#include "OpenVolumeHeader.h"
#include <map>

namespace Volume{
	using namespace OpenVolumeMesh;
	class MeshVolume : public BaseMeshVolume
	{
	public:
		MeshVolume();
		~MeshVolume();
	
		std::vector<HalfEdgeHandle> HalfFaceHalfEdges(HalfFaceHandle f);

		Vec3d Normal(HalfFaceHandle hf);

		char& IsVariable(EdgeHandle e) { return request_edge_property<char>("is_variable")[e]; }
		char& IsFunction(EdgeHandle e) { return request_edge_property<char>("is_function")[e]; }
		char& IsChangable(EdgeHandle e) { return request_edge_property<char>("is_changable")[e]; }
		double& CornerAngle(CellHandle c, HalfEdgeHandle h);
		double& DihedralAngle(CellHandle c, EdgeHandle e);
		double& Derivative(EdgeHandle e1, EdgeHandle e2);
		bool CellCornerTouched(CellHandle c, HalfEdgeHandle h);
		bool CellEdgeTouched(CellHandle c, EdgeHandle e);
		bool EdgeEdgeTouched(EdgeHandle e1, EdgeHandle e2);
		void ClearAngles() { corner_angle.clear(); dihedral_angle.clear(); derivative.clear(); }
		std::vector<VertexHandle> CellVertices(CellHandle c);

		int & VariableIndex(EdgeHandle e) { return request_edge_property<int>("variable_index")[e]; }
		int & FunctionIndex(EdgeHandle e) { return request_edge_property<int>("function_index")[e]; }

		void clear(bool clear_prop);

	protected:
		// Initiate properties of vertex.
		void InitVertexProperties();
		// Initiate properties of edge.
		void InitEdgeProperties();
		// Initiate properties of hafledge.
		void InitHalfEdgeProperties();
		// Initiate properties of face.
		void InitFaceProperties();
		// Initiate properties of halfface
		void InitHalfFaceProperties();
		//Initiate properties of Cell
		void InitCellProperties();

	protected:
		std::map<unsigned long long, double> corner_angle;
		std::map<unsigned long long, double> dihedral_angle;
		std::map<unsigned long long, double> derivative;

		template<class Handle1, class Handle2>
		unsigned long long IndexMultipleHandle(Handle1 handle1, Handle2 handle2);


	};
	
	MeshVolume::MeshVolume() {
		 InitVertexProperties();
		// Initiate properties of edge.
		 InitEdgeProperties();
		// Initiate properties of hafledge.
		 InitHalfEdgeProperties();
		// Initiate properties of face.
		 InitFaceProperties();
		 // initiate properties of halfface
		 InitHalfFaceProperties();
	}

	inline MeshVolume::~MeshVolume()
	{
	}

	inline std::vector<HalfEdgeHandle> MeshVolume::HalfFaceHalfEdges(HalfFaceHandle f)
	{
		std::vector<VertexHandle> verts;
		for (HalfFaceVertexIter hfviter = hfv_iter(f); hfviter.valid(); ++hfviter) {
			VertexHandle v = *hfviter;
			verts.push_back(v);
		}
		std::vector<HalfEdgeHandle> halfedges;
		for (int i = 0; i < 3; ++i) {
			VertexHandle v1 = verts[i];
			VertexHandle v2 = verts[(i + 1) % 3];
			halfedges.push_back(halfedge(v1, v2));
		}
		return halfedges;
	}

	inline Vec3d MeshVolume::Normal(HalfFaceHandle hf)
	{
		std::vector<VertexHandle> verts;
		for (HalfFaceVertexIter hfviter = hfv_iter(hf); hfviter.valid(); ++hfviter) {
			verts.push_back(*hfviter);
		}
		Vec3d d1 = vertex(verts[1]) - vertex(verts[0]);
		Vec3d d2 = vertex(verts[2]) - vertex(verts[0]);
		Vec3d normal = cross(d1, d2);
		return normal / normal.norm();
	}
	

	inline double & MeshVolume::CornerAngle(CellHandle c, HalfEdgeHandle h)
	{
		unsigned long long index = IndexMultipleHandle(c, h);
		return corner_angle[index];

	}

	inline double & MeshVolume::DihedralAngle(CellHandle c, EdgeHandle e)
	{
		auto index = IndexMultipleHandle(c, e);
		return dihedral_angle[index];
	}

	inline double & MeshVolume::Derivative(EdgeHandle e1, EdgeHandle e2)
	{
		auto index = IndexMultipleHandle(e1, e2);
		return derivative[index];
	}

	inline bool MeshVolume::CellCornerTouched(CellHandle c, HalfEdgeHandle h)
	{
		auto index = IndexMultipleHandle(c, h);
		return corner_angle.count(index) != 0;
	}

	inline bool MeshVolume::CellEdgeTouched(CellHandle c, EdgeHandle e)
	{
		auto index = IndexMultipleHandle(c, e);
		return dihedral_angle.count(index) != 0;
	}

	inline bool MeshVolume::EdgeEdgeTouched(EdgeHandle e1, EdgeHandle e2)
	{
		auto index = IndexMultipleHandle(e1, e2);
		return derivative.count(index) != 0;
	}

	inline std::vector<VertexHandle> MeshVolume::CellVertices(CellHandle c)
	{
		std::vector<VertexHandle> verts;
		for (CellVertexIter cviter = cv_iter(c); cviter.valid(); ++cviter) {
			VertexHandle v = *cviter;
			verts.push_back(v);
		}
		Vec3d center = vertex(verts[0]);
		Vec3d p1 = vertex(verts[1]);
		Vec3d p2 = vertex(verts[2]);
		Vec3d d1 = (p1 - center);
		Vec3d d2 = (p2 - center);
		Vec3d normal = cross(d1 , d2);
		normal = normal / normal.norm();
		Vec3d up = vertex(verts[3]) - center;
		up = up / up.norm();
		if (dot(up, normal) < 0) {
			auto tem = verts[0];
			verts[0] = verts[1];
			verts[1] = tem;
		}
		return verts;
	}

	inline void MeshVolume::clear(bool clear_prop)
	{
		BaseMeshVolume::clear(clear_prop);
		ClearAngles();
	}

	inline void MeshVolume::InitVertexProperties()
	{

	}
	inline void MeshVolume::InitEdgeProperties()
	{
		auto is_variable = request_edge_property<char>("is_variable");
		set_persistent(is_variable, true);

		auto is_function = request_edge_property<char>("is_function");
		set_persistent(is_function, true);

		auto is_changable = request_edge_property<char>("is_changable");
		set_persistent(is_changable, true);

		auto variable_index = request_edge_property<int>("variable_index");
		set_persistent(variable_index, true);

		auto function_index = request_edge_property<int>("function_index");
		set_persistent(function_index, true);
	}
	inline void MeshVolume::InitHalfEdgeProperties()
	{
		
	}
	inline void MeshVolume::InitFaceProperties()
	{
		
	}
	inline void MeshVolume::InitHalfFaceProperties()
	{
		
	}
	inline void MeshVolume::InitCellProperties()
	{
		
	}
	template<class Handle1, class Handle2>
	inline unsigned long long MeshVolume::IndexMultipleHandle(Handle1 handle1, Handle2 handle2)
	{
		return handle1.idx() * 100000 + handle2.idx();
	}
}

#endif // !_MESH_VOLUME
