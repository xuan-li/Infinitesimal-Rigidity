#ifndef _VIEWER_FLAGS
#define _VIEWER_FLAGS


enum ShowOption {surface, triangulated_surface, volume};


static ShowOption ShowMesh = surface;

static bool ShowPolygonalEdges = true;
static bool ShowVertexSpheres = false;


// only available wehn ShowGraph is true;
static bool ShowEdgeColor = false;
static bool FillGraphFace = true;
static bool ShowVertexColor = false;

static bool ShowMotion = false;
static int motion_mode = 0;

static char TetMode = '5';


#endif // !_VIEWER_FLAGS
