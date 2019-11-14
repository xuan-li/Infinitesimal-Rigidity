#ifndef _KEYBOARD
#define _KEYBOARD

#include "Common.h"
#include "ViewerFlags.h"
#include "ViewerData.h"
#include "Controller.h"

// This function is called every time a keyboard button is pressed
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
	if (ShowMesh == volume) {
		UpdateMeshView(viewer, key);
	}

	if (key == 'N' && ShowMotion) {
		motion_mode++;
		UpdateMeshData(viewer);
	}
	return false;
}

#endif // !_KEYBOARD
