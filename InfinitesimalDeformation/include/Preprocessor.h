#ifndef _PREPROCESSOR
#define _PREPROCESSOR

#include "AlgorithmCommon.h"

namespace Rigidity {
	class Preprocessor
	{
	public:
		Preprocessor(Surface::MeshSurface &surface, Volume::MeshVolume &volume);
		void Triangularize();
		void Tetrahedralize();
	protected:
		Surface::MeshSurface *m_surface;
		Volume::MeshVolume *m_volume;

		
		void ModifyFaces(Surface::FaceHandle f, std::vector<Eigen::Vector3i> triangles);
		
	};

	Preprocessor::Preprocessor(Surface::MeshSurface &surface, Volume::MeshVolume &volume) : m_surface(&surface), m_volume(&volume) {

	}

	inline void Preprocessor::Triangularize()
	{
		for (auto eiter = m_surface->edges_begin(); eiter != m_surface->edges_end(); ++eiter) {
			auto e = *eiter;
			m_surface->IsChangable(e) = true;
		}

		std::vector<Surface::FaceHandle> faces;
		for (auto fiter = m_surface->faces_begin(); fiter != m_surface->faces_end(); ++fiter){
			auto f = *fiter;
			faces.push_back(f);
			
		}
		for (int i = 0; i < faces.size(); i++) {
			auto f = faces[i];
			Triangulate::TPPLTriangulator triangulator;
			std::vector<Eigen::Vector3i> triangles = triangulator.TriangulateFaceHandle(*m_surface, f);
			ModifyFaces(f, triangles);
		}
		m_surface->garbage_collection();
		//m_surface->WriteMeshFile("test.obj");
	}

	inline void Preprocessor::ModifyFaces(Surface::FaceHandle f, std::vector<Eigen::Vector3i> triangles)
	{
		using namespace OpenMesh;

		std::vector<std::vector<VertexHandle>> add_vert_pairs;
		for (int i = 0; i < triangles.size(); ++i) {
			Eigen::Vector3i triangle = triangles[i];
			for (int j = 0; j < 3; j++) {
				VertexHandle v0 = m_surface->vertex_handle(triangle[j]);
				VertexHandle v1 = m_surface->vertex_handle(triangle[(j + 1) % 3]);
				HalfedgeHandle h = m_surface->find_halfedge(v0, v1);
				if (!h.is_valid()) {
					std::vector<VertexHandle> verts({ v0, v1 });
					add_vert_pairs.push_back(verts);
				}
			}
		}

		m_surface->delete_face(f);
		

		for (int i = 0; i < triangles.size(); ++i) {
			Eigen::Vector3i triangle = triangles[i];
			std::vector<VertexHandle> verts;
			for (int j = 0; j < 3; j++) {
				verts.push_back(m_surface->vertex_handle(triangle[j]));
			}
			m_surface->add_face(verts);
		}

		for (int i = 0; i < add_vert_pairs.size(); i++) {
			auto vpair = add_vert_pairs[i];
			HalfedgeHandle h = m_surface->find_halfedge(vpair[0], vpair[1]);
			EdgeHandle e = m_surface->edge_handle(h);
			m_surface->IsChangable(e) = false;
		}


		
	}

	inline void Preprocessor::Tetrahedralize()
	{
		m_volume->clear(false);
		//*m_volume = Volume::MeshVolume();
		::Tetrahedralize(*m_surface, *m_volume);

		std::vector<Surface::VertexHandle> verts_surface;
		std::vector<Volume::VertexHandle> verts_volume;

		Surface::MeshSurface::VertexIter viter1 = m_surface->vertices_begin();
		Volume::VertexIter viter2 = m_volume->vertices_begin();
		for (; viter1 != m_surface->vertices_end(); ++viter1, ++viter2) {
			verts_surface.push_back(*viter1);
			verts_volume.push_back(*viter2);
			assert(m_volume->is_boundary(*viter2));
		}

		for (Surface::MeshSurface::EdgeIter eiter = m_surface->edges_begin(); eiter != m_surface->edges_end(); ++eiter) {
			Surface::HalfedgeHandle h = m_surface->halfedge_handle(*eiter, 0);
			Surface::EdgeHandle e_surface = m_surface->edge_handle(h);
			int iv1 = m_surface->from_vertex_handle(h).idx();
			int iv2 = m_surface->to_vertex_handle(h).idx();
			Volume::HalfEdgeHandle h_volume = m_volume->halfedge(verts_volume[iv1], verts_volume[iv2]);
			Volume::EdgeHandle e_volume = m_volume->edge_handle(h_volume);
			m_volume->IsChangable(e_volume) = m_surface->IsChangable(e_surface);
		}

		for (Volume::EdgeIter eiter = m_volume->edges_begin(); eiter != m_volume->edges_end(); ++eiter){
			Volume::EdgeHandle e = *eiter;
			if (m_volume->is_boundary(e)) {
				m_volume->IsVariable(e) = false;
				continue;
			}
			m_volume->IsVariable(e) = true;
			m_volume->IsChangable(e) = false;
		}
	}

}// end of namespace Rigidity;
#endif // !_PREPROCESSOR
