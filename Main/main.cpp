#include "Common.h"
#include "ViewerData.h"
#include "Menu.h"
#include "KeyBoard.h"
#include "Controller.h"

int main(int argc, char** argv) {
	
	std::cout << R"(
    1 Switch polygonal edges/triangulated edges
    2 Switch vertex spheres on/off
    3 Switch face normals on/off
    )";

	viewer.callback_init = [&](igl::viewer::Viewer& viewer)
	{
		InitMenu(viewer);
		viewer.callback_key_down = key_down;
		return false;
	};
	
	viewer.launch();
}