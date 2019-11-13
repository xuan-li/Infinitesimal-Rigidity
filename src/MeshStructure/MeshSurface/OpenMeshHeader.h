#ifndef _OPENMESH_HEADER
#define _OPENMESH_HEADER

#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

namespace Surface {
	struct MyTraits :public OpenMesh::DefaultTraits
	{
		typedef OpenMesh::Vec3d Point;
		typedef OpenMesh::Vec3d Normal;
		VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Status);
		FaceAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Status);
		HalfedgeAttributes(OpenMesh::Attributes::Status);
		EdgeAttributes(OpenMesh::Attributes::Status);
	};

	typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> BaseMeshSurface;

} //end of namespace Polycube

#endif
