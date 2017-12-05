#ifndef _VIEWER_FLAGS
#define _VIEWER_FLAGS


enum ShowOption {surface, triangulated_surface, volume};


ShowOption ShowMesh = surface;

bool ShowPolygonalEdges = true;
bool ShowVertexSpheres = false;


// only available wehn ShowGraph is true;
bool ShowEdgeColor = false;
bool FillGraphFace = true;
bool ShowVertexColor = false;

bool ShowMotion = false;
int motion_mode = 0;

char TetMode = '5';


#endif // !_VIEWER_FLAGS
