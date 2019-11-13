#ifndef _MENU_MAIN
#define _MENU_MAIN

#include "Common.h"
#include "Controller.h"
#include "ViewerFlags.h"

void InitMenu(igl::viewer::Viewer &viewer) 
{
	viewer.ngui->addWindow(Eigen::Vector2i(220, 10), "Rigidity Lib");
	
	viewer.ngui->addGroup("File IO");
	viewer.ngui->addButton("Load Mesh", [&]() {LoadMeshFromFile(); UpdateMeshData(viewer); });
	
	viewer.ngui->addGroup("Algorithm");
	viewer.ngui->addButton("Compute Motion", [&]() {controller.ComputeMotion(); });
	viewer.ngui->addButton("Deformation", [&]() {controller.Deformation();  UpdateMeshData(viewer); });
	viewer.ngui->addGroup("Viewer Flags");

	viewer.ngui->addVariable<bool>("Show Polygonal Edges", 
		[&](const bool & v) { ShowPolygonalEdges = v; UpdateMeshData(viewer); }, 
		[&]() -> bool { return ShowPolygonalEdges; });

	viewer.ngui->addVariable<bool>("Show Vertex Spheres",
		[&](const bool & v) { ShowVertexSpheres = v; UpdateMeshData(viewer); },
		[&]() -> bool { return ShowVertexSpheres; });
	
	

	viewer.ngui->addVariable<ShowOption>("Show Mesh",
		[&](const ShowOption & v) { ShowMesh = v; UpdateMeshData(viewer); },
		[&]() -> ShowOption { return ShowMesh; })->setItems({"Surface", "Triangulated Surface","Volume"});
	
	viewer.ngui->addVariable<bool>("Show Motion",
		[&](const bool & v) { ShowMotion = v; UpdateMeshData(viewer); },
		[&]() -> bool { return ShowMotion; });

	viewer.screen->performLayout();
}


#endif // !_MENUS
