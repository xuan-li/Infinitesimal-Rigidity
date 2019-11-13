#ifndef _MESHSURFACE
#define _MESHSURFACE

#include "OpenMeshHeader.h"

namespace Surface {
	using namespace OpenMesh;
	class MeshSurface : public BaseMeshSurface {
	public:
		MeshSurface();
		~MeshSurface();

		// Read .obj file
		void ReadMeshFile(std::string  input);

		// Write .obj file
		void WriteMeshFile(std::string output);


		// ===== Common Properties =====

		// Return whether item is touched.
		template<class ItemHandle>
		char& Touched(ItemHandle item) { return GetProperty<char>("touched", item); }


		// ===== Vertex Properties =====
		std::vector<Vec3d>& Motions(VertexHandle v) { return GetProperty<std::vector<Vec3d>>("motions", v); }
		std::vector<Vec3d> & LastMotions(VertexHandle v) { return GetProperty<std::vector<Vec3d>>("last_motions", v); }
		
		// ===== Edge Properties =====
		char & IsChangable(EdgeHandle e) { return GetProperty<char>("is_changable", e); }


	protected:

		// Initiate properties of vertex.
		void InitVertexProperties();
		// Initiate properties of edge.
		void InitEdgeProperties();
		// Initiate properties of hafledge.
		void InitHalfedgeProperties();
		// Initiate properties of face.
		void InitFaceProperties();

		// Register a persistent property that can be found all the time.
		template<class PropHandle>
		void RegisteProperty(PropHandle handle, std::string name)
		{
			add_property(handle, name);
			property(handle).set_persistent(true);
		}


		// Find persistent properties by names.
		template<class T>
		T& GetProperty(std::string name, VertexHandle item_handle) {
			VPropHandleT<T> prop;
			this->get_property_handle(prop, name);
			return property(prop, item_handle);
		}
		// Find persistent properties by names.
		template<class T>
		T& GetProperty(std::string name, FaceHandle item_handle) {
			FPropHandleT<T> prop;
			this->get_property_handle(prop, name);
			return property(prop, item_handle);
		}
		// Find persistent properties by names.
		template<class T>
		T& GetProperty(std::string name, EdgeHandle item_handle) {
			EPropHandleT<T> prop;
			this->get_property_handle(prop, name);
			return property(prop, item_handle);
		}
		// Find persistent properties by names.
		template<class T>
		T& GetProperty(std::string name, HalfedgeHandle item_handle) {
			HPropHandleT<T> prop;
			this->get_property_handle(prop, name);
			return property(prop, item_handle);
		}

	};

	MeshSurface::MeshSurface()
	{

		InitVertexProperties();
		InitEdgeProperties();
		InitHalfedgeProperties();
		InitFaceProperties();

	}


	MeshSurface::~MeshSurface()
	{
	}

	void MeshSurface::ReadMeshFile(std::string input)
	{
		IO::read_mesh(*this, input);
	}

	void MeshSurface::WriteMeshFile(std::string output)
	{
		IO::write_mesh(*this, output);
	}

	void MeshSurface::InitVertexProperties()
	{
		VPropHandleT<char> touched;
		RegisteProperty(touched, "touched");

		VPropHandleT<std::vector<Vec3d>> motions;
		RegisteProperty(motions, "motions");

		VPropHandleT<std::vector<Vec3d>> last_motions;
		RegisteProperty(last_motions, "last_motions");

	}

	void MeshSurface::InitEdgeProperties()
	{

		EPropHandleT<char> touched;
		RegisteProperty(touched, "touched");

		EPropHandleT<char> is_changable;
		RegisteProperty(is_changable, "is_changable");
	}

	void MeshSurface::InitHalfedgeProperties()
	{
		HPropHandleT<char> touched;
		RegisteProperty(touched, "touched");


	}

	void MeshSurface::InitFaceProperties()
	{

		FPropHandleT<char> touched;
		RegisteProperty(touched, "touched");

	}


} // end of namespace Polycube

#endif
