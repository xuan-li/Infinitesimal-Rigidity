#include "Common.h"
#include "ViewerData.h"
#include "KeyBoard.h"
#include "Controller.h"
#include "ViewerFlags.h"


int main(int argc, char** argv) {
	
	std::cout << R"(
    1 Switch polygonal edges/triangulated edges
    2 Switch vertex spheres on/off
    3 Switch face normals on/off
    )";

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);
    menu.callback_draw_viewer_menu = [&]()
    {
        if (ImGui::CollapsingHeader("File IO", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("LoadMesh", ImVec2(-1,0)))
            {
                LoadMeshFromFile();
                UpdateMeshData(viewer);
            }
        }
        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Compute Motion", ImVec2(-1,0)))
            {
                controller.ComputeMotion();
            }
            if (ImGui::Button("Deformation", ImVec2(-1,0)))
            {
                controller.Deformation();
                UpdateMeshData(viewer);
            }

        }
        if (ImGui::CollapsingHeader("Viewer Flags", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Checkbox("Show Polygonal Edges", &ShowPolygonalEdges))
            {
                UpdateMeshData(viewer);
            }
            if (ImGui::Checkbox("Show Vertex Spheres", &ShowVertexSpheres))
            {
                UpdateMeshData(viewer);
            }

            ImGui::Combo("Show Mesh", (int *)(&ShowMesh), "Surface\0Triangulated Surface\0Volume\0\0");

            if (ImGui::Checkbox("Show Motion", &ShowMotion))
            {
                UpdateMeshData(viewer);
            }
        }

    };
		viewer.callback_key_down = &key_down;
	    viewer.launch();

}