#pragma once
#include <ctime>
#include <cmath>

#include <thread>
#include <mutex>

#include <windows.h>

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#define FREEGLUT_LIB_PRAGMAS 0 // on x64, this tries to link the wrong lib file
#include <GL/freeglut.h>
#include "DataCaptureThread.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

#include "CameraView.h"
#include "UtilMatrix.h"
#include "utility.h"
#include "CSurface.h"
#include "CPointCloud.h"
#include "surface_misc.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <cuda_gl_interop.h>

#include "FusionConfig.h"
#include "TofinoMeshServer.h"
#include "GlobalDataStatic.h"

#include "../Common/cuda/PinnedMemory.h"
#include "../Common/cuda/CudaHelpers.h"

#include "ShadowMap.h"
#include "FusionControlPanelConnector.h"

#include "TemporalFusionThread.h"

#include <json/json.h>

#define FORCE_NO_RENDER 0

#define AUTO_CALIBRATION_PORT 43002

const int vt_buf_size_per_view = VTS_NUM_MAX;
const int vt_buf_size = vt_buf_size_per_view * 3;

const int ele_buffer_size = vt_buf_size_per_view * 2;

int surfaces_num = 3;
GLuint vboBuf_vts;
GLuint elementbuffer; //vertex indices of triangles
int vts_num_accu = 0;
int vts_num_accu_t = 0;
int vts_num_curr = 0;
int tris_num = 0;
struct cudaGraphicsResource* cuda_res_vts;
struct cudaGraphicsResource* cuda_res_tris;
int ed_nodes_num = 0;
DeformGraphNodeCuda* ed_nodes = NULL;
RigidTransformCuda rigid_transf;

S3DPointMatchSet match_set_3d;

int splitted_window_num = 3;
int show_config_values = false;

int idleCycles = 0;
//threads
DataCaptureFileMultiView *thread_capture = NULL;
DataCaptureNetworkMultiView *thread_network_capture = NULL;

FusionControlPanelConnector* FusionCPC = NULL;

TemporalFusion *thread_cuda_fusion = NULL;

std::thread *t_capture = NULL;
std::thread *t_cuda = NULL;

unsigned short		g_mesh_server_port = TOFINO_MESH_SERVER_PORT;
TofinoMeshServer* g_mesh_server = NULL;
std::thread* t_mesh_server = NULL;
std::thread* t_control_panel_connector = NULL;

const int keyListLength = 35;
std::map<int, string> keyListBase;
std::map<int, string> keyListFull;
float FOVY = 70.0; // the verticle angle of view 
float dx = 0;
float dy = 0;
float dz = 0;
float scale = 1.0;
float angle_updown = 0;
float angle_leftright = 0;
int rotate_z = 0; 

double lastRotationMat[16];
int lastPos[2];
double rot_cen[3];

bool bCentralize = false;
int rows = 1;

int windowHeight = 1000; // the height of the buffer image, which should equal to the height of the window
int windowWidth = 1000;

char name[500];

GLfloat light_direction1[] = { -1.0, 0.01, 0.01, 0.0 };
GLfloat light_direction2[] = { 1, 0, 0.5, 0.0 };
GLfloat light_direction3[] = { -1, 0, 0.5, 0.0 };
GLfloat light_direction4[] = { 0, 0, 1, 0.0 };

GLfloat light_diffuse1[] = { 0.18, 0.18, 0.18, 1.0 };
GLfloat light_ambient1[] = { 0.07, 0.07, 0.07, 1.0 };
GLfloat light_specular1[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat light_diffuse2[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat light_ambient2[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat light_specular2[] = { 0, 0, 0, 1.0 };
GLfloat light_diffuse3[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat light_ambient3[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat light_specular3[] = { 0, 0, 0, 1.0 };
GLfloat light_diffuse4[] = { 0.4, 0.4, 0.4, 1.0 };
GLfloat light_ambient4[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat light_specular4[] = { 1.0, 1.0, 1.0, 1.0 };

enum TextureMode
{
	SHOW_GREY = 0,
	SHOW_TEXTURE = 1,
	SHOW_NORMAL = 2,
	SHOW_COLOR = 3
};

TextureMode show_color_mode = SHOW_GREY;
bool show_cameras = true;
bool show_axis = false;
bool show_box = true;
bool show_graph = false;
bool bGraphTransfed = true;
bool bShowParallel = true;
bool bRotating = false;
bool bSaving = false;
bool bBlackBackground = true;
bool bLighting = true;
bool show_all_surfaces = false;
bool bDrawMatches3D = false;

vector<bool> show_surfaces_vec = vector<bool>(3, false);

bool in_the_first_few_frames = true;

#define SAVE_IMAGE
cv::Mat *img = NULL;
int save_count = 0;

vector<vector<GLfloat>> colorMats;

char display_str[5000];
char frameIdx_input_str[100];

//click a point
bool bNewClicked = false;
bool bPtSelected = false; //point selected
bool bSearchVert = false;//make sure the clicked point is a vertex
int surf_idx;
int vert_idx;
int x_clicked = -1;
int y_clicked = -1;
double pt_clicked[3]; //the point clicked

#define BUFFER_OFFSET(i) ((void*)(i))

ShadowMap* shadow_map;

bool renderingEnabled();