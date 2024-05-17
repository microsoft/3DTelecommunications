#include "LiveFusionDemo.h"
#include "Peabody.h"
#include "3dtm_version.h"
#include <strsafe.h>

using namespace std;

int DEPTH_CAMERAS_NUM = MAX_NUM_DEPTH_CAMERAS;
int FrameCountKeyVolume = 1;

/// <summary> 
/// Enable the rendering procedure, if and only if:
/// 1) The fusion has run through the first few frames (e.g., 7 frames), 
/// since the fusion does not work well with only very few frames.
/// 2) The reconstructed surfaces are non-empty. 
/// </summary>
bool renderingEnabled()
{
	return (!in_the_first_few_frames) && (show_surfaces_vec.size() >= 0) && (show_surfaces_vec[0] || show_surfaces_vec[1] || show_surfaces_vec[2]);
}

void init(void)
{
	glewInit();

	const GLubyte* version;

	if (version = glGetString(GL_VERSION))
	{
		printf("OpenGL Version: %s\n", version);
	}
	//VBO buffer
	//init vertex VBO
	glGenBuffers(1, &vboBuf_vts);
	glBindBuffer(GL_ARRAY_BUFFER, vboBuf_vts);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vt_buf_size * 9, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &elementbuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ele_buffer_size * sizeof(int) * 3, NULL, GL_STATIC_DRAW);

	cudaSetDevice(0);
	cudaGLSetGLDevice(0);

	checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_res_vts, vboBuf_vts, cudaGraphicsMapFlagsWriteDiscard));
	checkCudaErrors(cudaGraphicsGLRegisterBuffer(&cuda_res_tris, elementbuffer, cudaGraphicsMapFlagsWriteDiscard));

	ed_nodes = new DeformGraphNodeCuda[ED_NODES_NUM_MAX];

	//start threads
	if (data_use_network_loader) {
		thread_network_capture = new DataCaptureNetworkMultiView();
	}
	else {
		thread_capture = new DataCaptureFileMultiView();
	}

	thread_cuda_fusion = new TemporalFusion();

	if (data_use_network_loader) {

		thread_cuda_fusion->set_network_capture_view(thread_network_capture);
		t_capture = new std::thread([=]() {

			thread_network_capture->operator()(); });
	}
	else {
		t_capture = new std::thread((*thread_capture));
	}

	if (run_mesh_server)
	{
		g_mesh_server = new TofinoMeshServer();
		g_mesh_server->initialize_server(g_mesh_server_port);  // mock server is started without gui

#ifdef TOFINO_MESH_SERVER_THREAD_SYNCHRONOUS
		g_mesh_server->set_run_asynchronous(false);
#endif
		//run this anyways
		t_mesh_server = new std::thread([]() {
			fprintf(stderr, "starting mesh server loop\n");
			g_mesh_server->server_mainloop();
			fprintf(stderr, "exiting mesh server loop\n");
			});

		thread_cuda_fusion->set_mesh_server(g_mesh_server);
	}

	t_cuda = new std::thread(&TemporalFusion::operator(), thread_cuda_fusion);

	if (FORCE_NO_RENDER)
		show_surfaces_vec.resize(surfaces_num, false);
	else
	{
		show_surfaces_vec.resize(surfaces_num, false);
		show_surfaces_vec[1] = true;
	}

	colorMats.resize(surfaces_num, vector<GLfloat>(3, 1.0));
	for (int i = 0; i < surfaces_num; i++)
	{
		colorMats[i][0] = RANDOM;
		colorMats[i][1] = RANDOM;
		colorMats[i][2] = RANDOM;
	}

	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_COLOR_MATERIAL);
	GLfloat shininess = 50;
	GLfloat mat_specular[] = { 0.35, 0.35, 0.3, 0.0 };
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

	/* set lighting parameters  */
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse1);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient1);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse2);
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient2);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular2);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, light_diffuse3);
	glLightfv(GL_LIGHT2, GL_AMBIENT, light_ambient3);
	glLightfv(GL_LIGHT2, GL_SPECULAR, light_specular3);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, light_diffuse4);
	glLightfv(GL_LIGHT3, GL_AMBIENT, light_ambient4);
	glLightfv(GL_LIGHT3, GL_SPECULAR, light_specular4);

	glLightfv(GL_LIGHT1, GL_POSITION, light_direction2);
	glLightfv(GL_LIGHT2, GL_POSITION, light_direction3);
	glLightfv(GL_LIGHT3, GL_POSITION, light_direction4);


	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
	//glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHTING);

	rot_cen[0] = 0.0;
	rot_cen[1] = -100.0;
	rot_cen[2] = 0.0;
	memset(lastRotationMat, 0, sizeof(double) * 16);
	lastRotationMat[0] = 1.0;
	lastRotationMat[5] = 1.0;
	lastRotationMat[10] = 1.0;
	lastRotationMat[15] = 1.0;

	shadow_map = new ShadowMap();

	keyListBase = {
	{27, "Escape - Exit"},
	{'=', "+/= - Zoom In/Out 1%"},
	{'z', "z - Rotate in Z by 90 degrees"},
	{'i', "i/I - Translate in Y +/-10 pixels"},
	{'l', "l/L - Translate in X +/-10 pixels"},
	{'k', "k/K - Translate in Z +/-10 pixels"},
	{'r', "r - Reset Key Frame"},
	{'R', "R - Reset viewport"},
	{'t', "t - Change color mode [Grey, Color, Texture, Normals]"},
	{'T', "T - Toggle black background"},
	{'g',"g - Toggle ED Node Graphs"},
	{'q',"q - Toggle graph transfed (requires G)"},
	{'b',"b - Toggle bounding box"},
	{'e',"e - Stop Fusion"},
	{'c',"c - Capture screenshot"},
	{'a',"a - Select camera for toggling [Fusion,1,2,3,4....]"},
	{'o',"o - Toggle current camera view"},
	{'m',"m - Toggle all views in paralell (requires w)"},
	{'M',"M - Draw match set"},
	{'n',"n - Toggle normals"},
	{' ',"Space - Toggle rotate"},
	{'v',"v - Toggle mesh server stats"},
	{'V',"V - Toggle logfile verbosity"},
	{'d',"d - Toggle hotkey/config display"},
	{'D',"D - Toggle mesh file capture"},
	{';',"; - Toggle lighting"},
	{'\'',"\' - Toggle high quality frame"},
	{'y',"y/Y - Increase/Decrease depth offset"},
	{'h',"h - Toggle send right view through mesh server"},
	{'u',"u - Reinit background frame with next frame"},
	{'w',"w - Toggle show all surfaces [Fusion, All, Freeze Update]"},
	{'0',"0 + Enter - Show surface 0"},
	{'1',"1 + Enter - Show surface 1"},
	{'2',"2 + Enter - Show surface 2"},
	};
	keyListFull = std::map(keyListBase);
}

void drawPoints(vector< vnl_vector_fixed<float, 3> > const& points, GLfloat color[], int step = 1)
{
	glBegin(GL_POINTS);
	glColor3fv(color);
	for (int i = 0; i < points.size(); i += step)
	{
		glVertex3f(points[i][0], points[i][1], points[i][2]);
	}
	glEnd();
}

void drawPoints(vector< vnl_vector_fixed<double, 3> > const& points, GLfloat color[], int step = 1)
{
	glBegin(GL_POINTS);
	glColor3fv(color);
	for (int i = 0; i < points.size(); i += step)
	{
		glVertex3d(points[i][0], points[i][1], points[i][2]);
	}
	glEnd();
}


void RenderString_2D(float x, float y, void* font, const char* string, float color[3])
{
	glColor3f(color[0], color[1], color[2]);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2f(x, y);
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glutBitmapString(font, (const unsigned char*)string);
}
void printAtPos(float x, float y, void* font, const char* string, float color[3])
{
	glColor3f(color[0], color[1], color[2]);
	glRasterPos2f(x, y);
	glutBitmapString(font, reinterpret_cast<const unsigned char*>(string));
}

void showHotKeyList()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	float valcolor[3] = { 0.1, 0.2, 0.1 };
	double linePixHeight = 14;
	double yLineSpacing = 2.0 / (windowHeight / linePixHeight);
	double yPos = 0.95;
	for (std::map<int, string>::iterator it = keyListFull.begin(); it != keyListFull.end(); ++it)
	{
		yPos -= yLineSpacing;
		printAtPos(-0.95, yPos, GLUT_BITMAP_HELVETICA_10, it->second.c_str(), valcolor);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void showConfigList()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	float valcolor[3] = { 0.1, 0.2, 0.1 };
	float keycolor[3] = { 0.0, 0.0, 0.0 };

	auto conf = FusionConfig::current();
	if (!conf)
	{
		printAtPos(-0.95, 0.95, GLUT_BITMAP_HELVETICA_10, "no config specified", valcolor);
		return;
	}

	auto title = string("config file: ") + FusionConfig::get_config_path();
	printAtPos(-0.95, 0.95, GLUT_BITMAP_HELVETICA_10, title.c_str(), valcolor);

	double linePixHeight = 14;
	double yLineSpacing = 2.0 / (windowHeight / linePixHeight);
	auto keys = conf->GetKeyList("Fusion");

	double yPos = 0.95;

	for (auto k : keys)
	{
		yPos -= yLineSpacing;
		auto kvstr = k + ":  " + conf->GetValue<std::string>("Fusion", k);
		printAtPos(-0.95, yPos, GLUT_BITMAP_HELVETICA_10, k.c_str(), keycolor);
		printAtPos(-0.95, yPos, GLUT_BITMAP_HELVETICA_10, kvstr.c_str(), valcolor);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void drawDeformGraph(DeformGraphNodeCuda* ed_nodes, int ed_nodes_num, RigidTransformCuda rigid_transf, bool bDrawTransformed, bool bDrawLabel)
{
	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);

	vector< vnl_vector_fixed<float, 3> > graph_points;
	vnl_matrix_fixed<float, 3, 3> R(rigid_transf.R.data_block());
	vnl_vector_fixed<float, 3> T(rigid_transf.T.data_block());
	for (int i = 0; i < ed_nodes_num; i++)
	{
		vnl_vector_fixed<float, 3> g(ed_nodes[i].g.data_block());
		vnl_vector_fixed<float, 3> t(ed_nodes[i].t.data_block());
		if (bDrawTransformed)
			graph_points.push_back(R * (g + t) + T);
		else
			graph_points.push_back(g);
	}
	GLfloat color_p[] = { 1.0, 0.8, 0.2 };
	glPointSize(10);
	drawPoints(graph_points, color_p);

	//draw label
	if (bDrawLabel)
	{
		GLdouble modelViewMatrix[16];
		GLdouble projMatrix[16];
		GLint viewport[4];
		GLdouble winx, winy, winz;
		glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
		glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
		glGetIntegerv(GL_VIEWPORT, viewport);
		for (int i = 0; i < ed_nodes_num; i++)
		{
			vnl_vector_fixed<float, 3> const& g = graph_points[i];
			gluProject(g[0], g[1], g[2],
				modelViewMatrix,
				projMatrix,
				viewport,
				&winx,
				&winy,
				&winz);
			GLfloat color[3];
			color[0] = 1.0; color[1] = 1.0; color[2] = 0.0;
			char name[500];
			sprintf(name, "%d", i);
			RenderString_2D(winx / windowWidth * 2.0 - 1.0, winy / windowHeight * 2.0 - 1.0, GLUT_BITMAP_TIMES_ROMAN_10, name, color);
		}
	}

	GLfloat color_l[] = { 0.1, 0.3, 0.9 };
	glColor3fv(color_l);
	glLineWidth(1);
	glBegin(GL_LINES);
	for (int i = 0; i < ed_nodes_num; i++)
	{
		for (int idx = 0; idx < EDNODE_NN; idx++)
		{
			int j = ed_nodes[i].neighbors[idx];
			if (0 <= j && j < ed_nodes_num)
			{
				glVertex3f(graph_points[i][0], graph_points[i][1], graph_points[i][2]);
				glVertex3f(graph_points[j][0], graph_points[j][1], graph_points[j][2]);
			}
		}
	}
	glEnd();


	GLfloat color_n[] = { 1.0, 0.1, 0.5 };
	glColor3fv(color_n);
	glBegin(GL_LINES);
	for (int i = 0; i < ed_nodes_num; i++)
	{
		cuda_vector_fixed<float, 3> const& n = ed_nodes[i].n;
		glVertex3f(graph_points[i][0], graph_points[i][1], graph_points[i][2]);
		glVertex3f(graph_points[i][0] - n[0] * 2.0, graph_points[i][1] - n[1] * 2.0, graph_points[i][2] - n[2] * 2.0);
	}
	glEnd();

	glPopAttrib();
}

void drawMatchSet3D(S3DPointMatchSet const& match_set_3d, int windowId_when_parallel = 0)
{
	glPushAttrib(GL_LIGHTING);
	if (bShowParallel && splitted_window_num > 0)
		glViewport(windowWidth * windowId_when_parallel / splitted_window_num, 0, (GLsizei)windowWidth / splitted_window_num, (GLsizei)windowHeight);
	else
		glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);

	glDisable(GL_LIGHTING);
	GLfloat color_p[] = { 1.0, 1.0, 0.0 };
	glPointSize(5.0);
	drawPoints(match_set_3d.points_1, color_p, 1);
	GLfloat color_p2[] = { 0.0, 1.0, 1.0 };
	drawPoints(match_set_3d.points_2, color_p2, 1);

	GLfloat color_l[] = { 0.0, 1.0, 0.0 };
	glColor3fv(color_l);
	glBegin(GL_LINES);
	for (int i = 0; i < match_set_3d.size(); i++)
	{
		if ((match_set_3d.points_1[i][0] == 0 && match_set_3d.points_1[i][1] == 0 && match_set_3d.points_1[i][2] == 0) ||
			(match_set_3d.points_2[i][0] == 0 && match_set_3d.points_2[i][1] == 0 && match_set_3d.points_2[i][2] == 0)
			)
			continue;
		glVertex3d(match_set_3d.points_1[i][0], match_set_3d.points_1[i][1], match_set_3d.points_1[i][2]);
		glVertex3d(match_set_3d.points_2[i][0], match_set_3d.points_2[i][1], match_set_3d.points_2[i][2]);
	}
	glEnd();
	glEnable(GL_LIGHTING);

	glPopAttrib();
}

void renderSurface()
{
	glBindBuffer(GL_ARRAY_BUFFER, vboBuf_vts);
	glEnableClientState(GL_VERTEX_ARRAY); // activate vertex coords array
	glVertexPointer(3, GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view));   // last param is offset, not ptr
	glEnableClientState(GL_NORMAL_ARRAY);
	glNormalPointer(GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view + 12));
	if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
	{
		glEnableClientState(GL_COLOR_ARRAY); // activate vertex coords array
		glColorPointer(3, GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view + 24));   // last param is offset, not ptr
	}

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
	glDrawElements(GL_TRIANGLES, tris_num * 3, GL_UNSIGNED_INT, BUFFER_OFFSET(0));

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
		glDisableClientState(GL_COLOR_ARRAY);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void display(void)
{
	if (!renderingEnabled())
		return;

	if (bBlackBackground)
		glClearColor(0.2, 0.2, 0.2, 0.0);
	else
		glClearColor(1.0, 1.0, 1.0, 0.0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	glEnable(GL_MULTISAMPLE);
	glHint(GL_MULTISAMPLE_FILTER_HINT_NV, GL_NICEST);

	int cols = std::ceil(double(surfaces_num) / rows);

	glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double aspect_ratio = (double)windowWidth / windowHeight;
	double zfar = 5000;
	if (bShowParallel)
		aspect_ratio = aspect_ratio * double(rows) / double(cols);
	gluPerspective(FOVY, aspect_ratio, 30.0, zfar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, -170, -300, 0, -100, 0, 0, -1, 0);

	glTranslatef(dx, -dy, -dz);
	glTranslatef(rot_cen[0], rot_cen[1], rot_cen[2]);
	glRotatef(-angle_leftright, 0, 1, 0);
	glRotatef(angle_updown, 1, 0, 0);
	glRotatef(rotate_z * 90.0, 0, 0, -1.0);
	glScalef(scale, scale, scale);
	glTranslatef(-rot_cen[0], -rot_cen[1], -rot_cen[2]);
	glMultMatrixd(lastRotationMat);

	float color[3] = { 0.3, 1.0, 0.3 };
	RenderString_2D(1.0 - 400.0 / windowWidth * 2.0, 1.0 - 30.0 / windowHeight * 2.0, GLUT_BITMAP_TIMES_ROMAN_24, display_str, color);

	// draw version string
	std::string szBuildDescrip = VERSION_DESCRIPTION;
	RenderString_2D(10, windowHeight - 30, GLUT_BITMAP_TIMES_ROMAN_24, szBuildDescrip.c_str(), color);

	glPushAttrib(GL_LIGHTING_BIT);
	glDisable(GL_LIGHTING);

	if (show_axis)
	{
		glLineWidth(2.0);
		glBegin(GL_LINES);
		glColor4f(1, 0, 0, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(200, 0, 0);

		glColor4f(0, 1, 0, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 200, 0);

		glColor4f(0, 0, 1, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 200);
		glEnd();
	}
	glPopAttrib();

	glPointSize(3.0);
	glColor3f(0.8, 0.8, 0.8);
	//show vertex buffer accuVt
	if (show_surfaces_vec[0])
	{
		int subWinId = 0;
		int colIdx = subWinId / rows;
		int rowIdx = rows - 1 - subWinId % rows;
		if (bShowParallel)
			glViewport(double(windowWidth * colIdx) / cols, double(windowHeight * rowIdx) / rows, (GLsizei)windowWidth / cols, (GLsizei)windowHeight / rows);
		else
			glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);

		switch (show_color_mode)
		{
		case SHOW_GREY:
			glColor3f(0.8, 0.8, 0.8);
			break;
		case SHOW_COLOR:
			glColor3f(colorMats[0][0], colorMats[0][1], colorMats[0][2]);
			break;
		case SHOW_TEXTURE:
		case SHOW_NORMAL:
			break;
		}

		glBindBuffer(GL_ARRAY_BUFFER, vboBuf_vts);
		glEnableClientState(GL_VERTEX_ARRAY); // activate vertex coords array
		glVertexPointer(3, GL_FLOAT, 9 * 4, 0);   // last param is offset, not ptr
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_FLOAT, 9 * 4, BUFFER_OFFSET(12));
		if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
		{
			glEnableClientState(GL_COLOR_ARRAY); // activate vertex coords array
			glColorPointer(3, GL_FLOAT, 9 * 4, BUFFER_OFFSET(24));   // last param is offset, not ptr
		}
		glDrawArrays(GL_POINTS, 0, vts_num_accu);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
			glDisableClientState(GL_COLOR_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

	}

	//draw vts_t
	if (show_surfaces_vec[1])
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		double aspect_ratio = (double)shadow_map->shadow_map_size / shadow_map->shadow_map_size;
		double zfar = 5000;
		gluPerspective(FOVY, aspect_ratio, 30.0, zfar);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		gluLookAt(0, -170, -300, 0, -100, 0, 0, -1, 0);

		glTranslatef(rot_cen[0], rot_cen[1], rot_cen[2]);
		glRotatef(-0, 0, 1, 0);
		glRotatef(45, 1, 0, 0);
		glRotatef(0.0, 0, 0, -1.0);
		glScalef(scale, scale, scale);
		glTranslatef(-rot_cen[0], -rot_cen[1], -rot_cen[2]);

		shadow_map->shadow_pass_begin();
		renderSurface();
		shadow_map->shadow_pass_end();

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		int subWinId = 1;
		int colIdx = subWinId / rows;
		int rowIdx = rows - 1 - subWinId % rows;

		int viewport_x;
		int viewport_y;
		int viewport_width;
		int viewport_height;

		if (bShowParallel)
		{
			viewport_x = double(windowWidth * colIdx) / cols;
			viewport_y = double(windowHeight * rowIdx) / rows;
			viewport_width = (GLsizei)windowWidth / cols;
			viewport_height = (GLsizei)windowHeight / rows;
		}
		else
		{
			viewport_x = 0;
			viewport_y = 0;
			viewport_width = (GLsizei)windowWidth;
			viewport_height = (GLsizei)windowHeight;
		}

		glViewport(viewport_x, viewport_y, viewport_width, viewport_height);

		switch (show_color_mode)
		{
		case SHOW_GREY:
			glColor3f(0.8, 0.8, 0.8);
			break;
		case SHOW_COLOR:
			glColor3f(colorMats[1][0], colorMats[1][1], colorMats[1][2]);
			break;
		case SHOW_TEXTURE:
		case SHOW_NORMAL:
			break;
		}

		renderSurface();

		shadow_map->snap_screen_depth(viewport_x, viewport_y, viewport_width, viewport_height);
		shadow_map->apply_deferred_shadows();

		shadow_map->draw_textures();
	}



	//draw vts_cur
	if (show_surfaces_vec[2])
	{
		int subWinId = 2;
		int colIdx = subWinId / rows;
		int rowIdx = rows - 1 - subWinId % rows;
		if (bShowParallel)
			glViewport(double(windowWidth * colIdx) / cols, double(windowHeight * rowIdx) / rows, (GLsizei)windowWidth / cols, (GLsizei)windowHeight / rows);
		else
			glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);

		switch (show_color_mode)
		{
		case SHOW_GREY:
			glColor3f(0.8, 0.8, 0.8);
			break;
		case SHOW_COLOR:
			glColor3f(colorMats[2][0], colorMats[2][1], colorMats[2][2]);
			break;
		case SHOW_NORMAL:
		case SHOW_TEXTURE:
			break;
		}

		glBindBuffer(GL_ARRAY_BUFFER, vboBuf_vts);
		glEnableClientState(GL_VERTEX_ARRAY); // activate vertex coords array
		glVertexPointer(3, GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view * 2));   // last param is offset, not ptr
		glEnableClientState(GL_NORMAL_ARRAY);
		glNormalPointer(GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view * 2 + 12));
		if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
		{
			glEnableClientState(GL_COLOR_ARRAY); // activate vertex coords array
			glColorPointer(3, GL_FLOAT, 9 * 4, BUFFER_OFFSET(sizeof(float) * 9 * vt_buf_size_per_view * 2 + 24));   // last param is offset, not ptr
		}
		glDrawArrays(GL_POINTS, 0, vts_num_curr);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		if (show_color_mode == SHOW_NORMAL || show_color_mode == SHOW_TEXTURE)
			glDisableClientState(GL_COLOR_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	if (show_graph)
	{
		if (bShowParallel && !bGraphTransfed)
		{
			int subWinId = 0;
			int colIdx = subWinId / rows;
			int rowIdx = rows - 1 - subWinId % rows;
			glViewport(double(windowWidth * colIdx) / cols, double(windowHeight * rowIdx) / rows, (GLsizei)windowWidth / cols, (GLsizei)windowHeight / rows);
		}
		else if (bShowParallel && bGraphTransfed)
		{
			int subWinId = 1;
			int colIdx = subWinId / rows;
			int rowIdx = rows - 1 - subWinId % rows;
			glViewport(double(windowWidth * colIdx) / cols, double(windowHeight * rowIdx) / rows, (GLsizei)windowWidth / cols, (GLsizei)windowHeight / rows);
		}
		else
			glViewport(0, 0, (GLsizei)windowWidth, (GLsizei)windowHeight);

		drawDeformGraph(ed_nodes, ed_nodes_num, rigid_transf, bGraphTransfed, false);
	}

	if (show_box)
	{
		double x_start = thread_cuda_fusion->bbox.x_s;
		double x_end = thread_cuda_fusion->bbox.x_e;
		double y_start = thread_cuda_fusion->bbox.y_s;
		double y_end = thread_cuda_fusion->bbox.y_e;
		double z_start = thread_cuda_fusion->bbox.z_s;
		double z_end = thread_cuda_fusion->bbox.z_e;

		glColor3f(1.0, 1.0, 0.5);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);

		glColor3f(1.0, 0.0, 0.0);	// x-axis
		glVertex3d(x_start, y_start, z_start);
		glVertex3d(x_end, y_start, z_start);
		glVertex3d(x_start, y_end, z_start);
		glVertex3d(x_end, y_end, z_start);
		glVertex3d(x_start, y_start, z_end);
		glVertex3d(x_end, y_start, z_end);
		glVertex3d(x_start, y_end, z_end);
		glVertex3d(x_end, y_end, z_end);

		glColor3f(0.0, 1.0, 0.0);	// y-axis
		glVertex3d(x_start, y_start, z_start);
		glVertex3d(x_start, y_end, z_start);
		glVertex3d(x_end, y_start, z_start);
		glVertex3d(x_end, y_end, z_start);
		glVertex3d(x_start, y_start, z_end);
		glVertex3d(x_start, y_end, z_end);
		glVertex3d(x_end, y_start, z_end);
		glVertex3d(x_end, y_end, z_end);

		glColor3f(0.0, 0.0, 1.0);	// z-axis
		glVertex3d(x_start, y_start, z_start);
		glVertex3d(x_start, y_start, z_end);
		glVertex3d(x_end, y_start, z_start);
		glVertex3d(x_end, y_start, z_end);
		glVertex3d(x_start, y_end, z_start);
		glVertex3d(x_start, y_end, z_end);
		glVertex3d(x_end, y_end, z_start);
		glVertex3d(x_end, y_end, z_end);
		glEnd();

		glBegin(GL_LINES);

		glColor3f(1.0, 0.0, 0.0);	// x-axis
		glVertex3d(0, 0, 0);
		glVertex3d(100, 0, 0);

		glColor3f(0.0, 1.0, 0.0);	// y-axis
		glVertex3d(0, 0, 0);
		glVertex3d(0, 100, 0);

		glColor3f(0.0, 0.0, 1.0);	// z-axis
		glVertex3d(0, 0, 0);
		glVertex3d(0, 0, 100);

		glEnd();


		glEnable(GL_LIGHTING);
	}

	if (bDrawMatches3D)
	{
		drawMatchSet3D(match_set_3d, 1);
	}


	if (show_config_values)
	{
		glViewport(0, 0, windowWidth, windowHeight);
		showConfigList();
	}
	else
	{
		glViewport(0, 0, windowWidth, windowHeight);
		showHotKeyList();
	}

	if (g_mesh_server && g_mesh_server->is_capturing_to_file()) {
		float captureColor[3] = { 0.5, 0.1, 0.1 };
		RenderString_2D(0, 0.95, GLUT_BITMAP_HELVETICA_12, "CAPTURING TO DISK", captureColor);
	}


	glDisable(GL_DEPTH_TEST);
	glutSwapBuffers();

	if (bSaving)
	{
		glReadBuffer(GL_FRONT);
		if (img != NULL) img->release();
		img = &cv::Mat(windowHeight, windowWidth, CV_MAKETYPE(8, 3));
		glReadPixels(0, 0, windowWidth, windowHeight, GL_RGB, GL_UNSIGNED_BYTE, img->ptr<unsigned char>());
		cv::flip(*img, *img, 0);
		for (int i = 0; i < windowHeight; i++)
		{
			for (int j = 0; j < windowWidth; j++)
			{
				cv::Vec3b dst = img->at<cv::Vec3b>(i, j);
				uchar tmp = dst[0];
				dst[0] = dst[2];
				dst[2] = tmp;
			}
		}
		char name[500];
		sprintf(name, "c:/demo/img_%04d.jpg", save_count);
		save_count++;
		cv::imwrite(name, *img);
	}
}

void reshape(int w, int h)
{
	windowWidth = w;
	windowHeight = h;
}

extern bool g_fusion_draw_current_depth;

void idle()
{
	g_fusion_draw_current_depth = show_surfaces_vec[2];

	if (bRotating)
	{
		angle_leftright += 0.1;
		glutPostRedisplay();
	}
	if (bLighting)
		glEnable(GL_LIGHTING);
	else
		glDisable(GL_LIGHTING);

	int bNewSurface = false;
	{
		lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
		bNewSurface = GlobalDataStatic::bNewSurface;
		GlobalDataStatic::bNewSurface = false;
	}

	if (bNewSurface && renderingEnabled())
	{
		cudaGraphicsResourceSetMapFlags(cuda_res_vts, cudaGraphicsMapFlagsWriteDiscard);
		cudaGraphicsMapResources(1, &cuda_res_vts, 0);
		float* dev_vts = NULL;
		size_t num_bytes;
		cudaGraphicsResourceGetMappedPointer((void**)&dev_vts, &num_bytes, cuda_res_vts);

		float* dev_accu_vts = dev_vts;
		vts_num_accu = vt_buf_size_per_view;
		if (show_surfaces_vec[0])
			thread_cuda_fusion->fusion4d->vol_fusion.sync_copy_vts_prev_to_dev_buf(dev_accu_vts, 9, vts_num_accu, show_color_mode, true);

		float* dev_accu_vts_t = dev_vts + vt_buf_size_per_view * 9;
		vts_num_accu_t = vt_buf_size_per_view;
		thread_cuda_fusion->fusion4d->vol_fusion.sync_copy_vts_t_to_dev_buf(dev_accu_vts_t, 9, vts_num_accu_t, show_color_mode, true);

		float* dev_curr_vts = dev_vts + vt_buf_size_per_view * 9 * 2;
		vts_num_curr = vt_buf_size_per_view;
		if (show_surfaces_vec[2]) // TODO: use key's input
			thread_cuda_fusion->fusion4d->vol_fusion.sync_copy_vts_cur_to_dev_buf(dev_curr_vts, 9, vts_num_curr, show_color_mode, true);
		cudaGraphicsUnmapResources(1, &cuda_res_vts, 0);

		cudaGraphicsResourceSetMapFlags(cuda_res_tris, cudaGraphicsMapFlagsWriteDiscard);
		cudaGraphicsMapResources(1, &cuda_res_tris, 0);
		int* dev_triangles = NULL;
		size_t num_bytes_tris;
		cudaGraphicsResourceGetMappedPointer((void**)&dev_triangles, &num_bytes, cuda_res_tris);

		tris_num = ele_buffer_size;
		thread_cuda_fusion->fusion4d->vol_fusion.sync_copy_tris_to_dev_buf(dev_triangles, tris_num);

		cudaGraphicsUnmapResources(1, &cuda_res_tris, 0);

		//TODO: no gpu-->cpu
		if (show_graph)
		{
			DeformGraphNodeCuda const* dev_ed_nodes = thread_cuda_fusion->fusion4d->vol_fusion.dev_ed_nodes();
			ed_nodes_num = thread_cuda_fusion->fusion4d->vol_fusion.ed_nodes_num();
			checkCudaErrors(cudaMemcpy(ed_nodes, dev_ed_nodes, sizeof(DeformGraphNodeCuda) * ed_nodes_num, cudaMemcpyDeviceToHost));
			RigidTransformCuda const* dev_rigid_transf = thread_cuda_fusion->fusion4d->vol_fusion.dev_rigid_transf();
			checkCudaErrors(cudaMemcpy(&rigid_transf, dev_rigid_transf, sizeof(RigidTransformCuda), cudaMemcpyDeviceToHost));
		}
		else
		{
			ed_nodes_num = 0;
		}

		if (bDrawMatches3D)
		{
			match_set_3d = thread_cuda_fusion->fusion4d->matches_3d;
		}

		glutPostRedisplay();
	}

	//only start sending FPS once system is actually running
	if (!in_the_first_few_frames)
	{
		if (idleCycles % 10 == 0)
		{
			double prevFPS = thread_cuda_fusion->GetPreviousFrameFPS();
			FusionCPC->SendStatusUpdate(CPC_STATUS::FPS, &prevFPS, sizeof(prevFPS));
		}
		idleCycles = idleCycles > 10 ? 0 : idleCycles++;
	}


	// The launcher service handles receiving new calibration data, so we don't want to re-save here
	if (FusionCPC->JSONCalibrationData != NULL)
	{
		thread_cuda_fusion->LoadCameras();
		// We have to delete this data once we're done, because the EventReceived that created it has no way to know when we are finished
		FusionCPC->DeleteCalibrationData();
	}

}


int level = 0;

void key(unsigned char key, int x, int y)
{
	int mod = glutGetModifiers(); //0--normal, 1--shift, 2--ctrl, 4--alt
	int depth_offset_delta = 1;

	switch (key)
	{
	case 27: // Escape key
		exit(0);
		break;
	case '+':
		FOVY *= 0.9;
		keyListFull['='] = keyListBase['='] + "[" + std::to_string(FOVY) + "]";
		break;
	case '=':
		FOVY *= 1.1;
		keyListFull['='] = keyListBase['='] + "[" + std::to_string(FOVY) + "]";
		break;
	case 'z':
		rotate_z++;
		if (rotate_z >= 4)
			rotate_z = 0;
		keyListFull[key] = keyListBase[key] + "[" + std::to_string(rotate_z * 90) + "]";
		break;
	case 'i':
		dy += 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dy) + "]";
		break;
	case 'I':
		dy -= 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dy) + "]";
		break;
	case 'l':
		dx += 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dx) + "]";
		break;
	case 'L':
		dx -= 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dx) + "]";
		break;
	case 'k':
		dz += 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dz) + "]";
		break;
	case 'K':
		dz -= 10;
		keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + std::to_string(dz) + "]";
		break;
	case 'r':
	{
		lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
		GlobalDataStatic::bResetKeyFrame = true;
	}
	break;
	case 'R':
		dx = 0;
		dy = 0;
		dz = 0;
		FOVY = 40.0;
		scale = 1.0;
		angle_updown = 0;
		angle_leftright = 0;
		break;
	case 't':
		switch (show_color_mode)
		{
		case SHOW_GREY:
			show_color_mode = SHOW_COLOR;
			break;
		case SHOW_COLOR:
			show_color_mode = SHOW_TEXTURE;
			break;
		case SHOW_TEXTURE:
			show_color_mode = SHOW_NORMAL;
			break;
		case SHOW_NORMAL:
			show_color_mode = SHOW_GREY;
			break;
		}
		keyListFull[key] = keyListBase[key] + "[" + std::to_string(show_color_mode) + "]";
		break;
	case 'T':
		bBlackBackground = !bBlackBackground;
		break;
	case 'g':
		show_graph = !show_graph;
		break;
	case 'q':
		bGraphTransfed = !bGraphTransfed;
		break;
	case 'b':
		show_box = !show_box;
	case 'e':
	{
		lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
		GlobalDataStatic::bStopFusion = !GlobalDataStatic::bStopFusion;
	}
	break;
	case 'c':
		bSaving = !bSaving;
		break;
	case 'a':
		g_select_fusion_input_view = (g_select_fusion_input_view + 1) % (DEPTH_CAMERAS_NUM + 1);

		if (g_select_fusion_input_view == 0)
		{
			keyListFull[key] = keyListBase[key] + "[ALL]";

			std::string depth_offsets;
			for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				depth_offsets += std::to_string(i + 1) + ": " + std::to_string(GlobalDataStatic::DepthOffset[i]) + ",";
			}
			keyListFull['y'] = keyListBase['y'] + "[" + depth_offsets + "]";
		}
		else
		{
			keyListFull[key] = keyListBase[key] + "[" + std::to_string(g_select_fusion_input_view) + "]";
			keyListFull['y'] = keyListBase['y'] + " [" + std::to_string(GlobalDataStatic::DepthOffset[g_select_fusion_input_view - 1]) + "]";
		}
		{
			std::string selected_views;
			for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				selected_views += std::to_string(i + 1) + ": " + std::to_string(g_select_fusion_input_views[i]) + ",";
			}
			keyListFull['o'] = keyListBase['o'] + "[" + selected_views + "]";
		}
		break;
	case 'o':

		if (g_select_fusion_input_view == 0) {
			for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				g_select_fusion_input_views[i] = 1;
			}
		}
		else {
			g_select_fusion_input_views[g_select_fusion_input_view - 1] = (g_select_fusion_input_views[g_select_fusion_input_view - 1] != 0) ? 0 : 1;
		}
		{
			std::string selected_views;
			for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				selected_views += std::to_string(i + 1) + ":" + std::to_string(g_select_fusion_input_views[i]) + ",";
			}
			keyListFull[key] = keyListBase[key] + "[" + selected_views + "]";
		}
		break;
	case 'm':
		bShowParallel = !bShowParallel;
		break;
	case 'M':
		bDrawMatches3D = !bDrawMatches3D;
		break;
	case ' ':
		bRotating = !bRotating;
		break;
	case 'v':
		if (g_mesh_server)
		{
			g_mesh_server->m_show_stats = !g_mesh_server->m_show_stats;
		}
		break;
	case 'V':
		if (g_mesh_server)
		{
			if (g_mesh_server->is_logging_to_file()) {
				keyListFull[key] = keyListBase[key] + " [OFF]";
				g_mesh_server->set_log_file_level(0);
			}
			else {
				keyListFull[key] = keyListBase[key] + " [ON]";
				g_mesh_server->set_log_file_level(2);
			}
		}
		break;
	case 'd':
		show_config_values = !show_config_values;
		break;
	case 'D':
		if (g_mesh_server)
		{
			if (g_mesh_server->is_capturing_to_file()) {
				keyListFull[key] = keyListBase[key] + " [OFF]";
				g_mesh_server->set_file_capture_location("");
			}
			else {
				keyListFull[key] = keyListBase[key] + " [ON]";
				g_mesh_server->set_file_capture_location("./meshpacket%04d.pkt");
			}
		}
		break;
	case ';':
		bLighting = !bLighting;
		break;
	case '\'':
		if (GlobalDataStatic::HighQualityFrameNumber != -1)
		{
			GlobalDataStatic::bStopFusion = false;
			GlobalDataStatic::bNextFrameHighQuality = false;
			GlobalDataStatic::HighQualityFrameNumber = -1; 
		}
		else
		{
			GlobalDataStatic::bNextFrameHighQuality = true;
		}
		break;
	case 'y':
	case 'Y':
		depth_offset_delta = (key == 'y') ? 1 : -1;
		if (g_select_fusion_input_view > 0) {
			GlobalDataStatic::DepthOffset[g_select_fusion_input_view - 1] += depth_offset_delta;
			keyListFull[tolower(key)] = keyListBase[tolower(key)] + " [" + std::to_string(GlobalDataStatic::DepthOffset[g_select_fusion_input_view - 1]) + "]";
			*LOGGER() << Logger::Debug << "DepthOffset[" << g_select_fusion_input_view << "]: " << GlobalDataStatic::DepthOffset[g_select_fusion_input_view - 1] << Logger::endl;
		} else {
			std::string depth_offsets;
			for (int i = 0; i < DEPTH_CAMERAS_NUM; i++) {
				GlobalDataStatic::DepthOffset[i] += depth_offset_delta;
				depth_offsets += std::to_string(i + 1) + ": " + std::to_string(GlobalDataStatic::DepthOffset[i]) + ",";
			}
			*LOGGER() << Logger::Debug << "DepthOffset " << depth_offsets << Logger::endl;
			keyListFull[tolower(key)] = keyListBase[tolower(key)] + "[" + depth_offsets + "]";
		}
		break;
	case 'h':
	{
		lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
		GlobalDataStatic::bSendRightViewThroughMeshServer = !GlobalDataStatic::bSendRightViewThroughMeshServer;
	}
	break;
	case 'u':
	{
		lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
		GlobalDataStatic::bInitbackgroundNextFrame = true;
	}
	break;
	case 'w':
		show_all_surfaces = !show_all_surfaces;
		if (show_all_surfaces)
		{
			for (int i = 0; i < surfaces_num; i++)
				show_surfaces_vec[i] = true;
		}
		else
		{
			for (int i = 0; i < surfaces_num; i++)
				show_surfaces_vec[i] = false;
		}
		break;
	case '0':
	case '1':
	case '2':
	{
		int len = strlen(display_str);
		if (len == 0)
			strcpy(display_str, "Surface: ");
		len = strlen(display_str);
		display_str[len] = key;
		len = strlen(frameIdx_input_str);
		frameIdx_input_str[len] = key;
		break;
	}
	case 13://return
	{
		int surfaceIdx = atoi(frameIdx_input_str);
		if (surfaceIdx < surfaces_num)
			show_surfaces_vec[surfaceIdx] = !show_surfaces_vec[surfaceIdx];
		memset(frameIdx_input_str, 0, 100);
		memset(display_str, 0, 5000);
		strcpy(display_str, "Surface: ");

		break;
	}
	case 8: //backspace
	{
		int len = strlen(frameIdx_input_str);
		if (len > 0)
		{
			frameIdx_input_str[len - 1] = '\0';
			len = strlen(display_str);
			display_str[len - 1] = '\0';
		}
		break;
	}
	}

	glutPostRedisplay();
}

int button_cur = -1;
int button_state_cur = -1;
int last_dx = 0;
int last_dy = 0;
bool bDrag = false; //differentiate click from drag
void mouse(int button, int state, int x, int y)
{
	int mod = glutGetModifiers(); //0--normal, 1--shift, 2--ctrl, 4--alt
	printf("<key, state, mod>: %d, %d, %d\r", button, state, mod);

	//update lastRotationMat
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslatef(rot_cen[0], rot_cen[1], rot_cen[2]);
	glRotatef(-angle_leftright, 0, 1, 0);
	glRotatef(angle_updown, 1, 0, 0);
	glTranslatef(-rot_cen[0], -rot_cen[1], -rot_cen[2]);
	glMultMatrixd(lastRotationMat);
	glGetDoublev(GL_MODELVIEW_MATRIX, lastRotationMat);
	glPopMatrix();
	angle_leftright = 0.0;
	angle_updown = 0.0;

	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			button_cur = GLUT_LEFT_BUTTON;
			button_state_cur = GLUT_DOWN;
			bDrag = false;

			lastPos[0] = x;
			lastPos[1] = y;
		}
		else if (state == GLUT_UP)
		{
			if (!bDrag)
			{
				button_cur = -1;
				button_state_cur = -1;
				x_clicked = x;
				y_clicked = y;
				bNewClicked = true;
				if (mod == 2)
					bSearchVert = true;
				else
					bSearchVert = false;
				glutPostRedisplay();
			}
		}
		break;
	case GLUT_MIDDLE_BUTTON:
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			button_cur = GLUT_MIDDLE_BUTTON;
			button_state_cur = GLUT_DOWN;

			lastPos[0] = x;
			lastPos[1] = y;
			last_dx = dx;
			last_dy = dy;
		}
		break;
	case 3: //scroll up
		dz += 10;
		glutPostRedisplay();
		break;
	case 4: //scroll down
		dz -= 10;
		glutPostRedisplay();
		break;
	case 19: //CTRL+scroll up
		dz += 0.5;
		glutPostRedisplay();
		break;
	case 20: //CTRL+scroll down
		dz -= 0.5;
		glutPostRedisplay();
		break;
	default:
		break;
	}
}

void mouseTrack(int x, int y)
{
	int dx_cur = x - lastPos[0];
	int dy_cur = y - lastPos[1];
	switch (button_cur)
	{
	case GLUT_LEFT_BUTTON:
		angle_updown = (float(dy_cur) * 1.8 * FOVY / windowHeight);
		angle_leftright = (float(dx_cur) * 1.8 * FOVY / windowHeight);
		bDrag = true;
		break;
	case GLUT_MIDDLE_BUTTON:
	case GLUT_RIGHT_BUTTON:
		dx = last_dx + dx_cur * 0.2 * FOVY / 40.0;
		dy = last_dy - dy_cur * 0.2 * FOVY / 40.0;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}

void on_window_closed()
{
	FusionCPC->SendStatusUpdate(CPC_STATUS::STOPPED);
	delete FusionCPC;
	//let's clean things up!
}

int main(int argc, char** argv)
{
	// setup file logging right away
	// route stdout and stderr to our files
	char* documentsPath = std::getenv("APPDATA");
	std::string logFilePath, errorFilePath;
	bool failedToCreate = false;

	if (documentsPath != nullptr)
	{
		//create dir if not exist
		std::string folderPath = std::string(documentsPath) + "\\..\\LocalLow\\Microsoft Research\\FusionLogs";
		std::error_code ec;
		std::filesystem::create_directories(folderPath, ec);
		if (!ec)
		{
			std::time_t currTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

			std::stringstream ss;
			ss << std::put_time(std::localtime(&currTime), "%Y-%m-%d_%X");
			std::string timeString = ss.str();
			//cleanup into appropriate file name
			timeString.erase(std::remove(timeString.begin(), timeString.end(), '\n'), timeString.end());
			std::replace(timeString.begin(), timeString.end(), ':', '-');

			logFilePath = folderPath + "\\Fusion_Output_" + timeString + ".txt";
			errorFilePath = folderPath + "\\Fusion_Error_" + timeString + ".txt";

			std::cout << "Fusion stdout/stderr redirected to " << logFilePath << " AND " << errorFilePath << std::endl;
			if (std::freopen(logFilePath.c_str(), "w", stdout) != nullptr && std::freopen(errorFilePath.c_str(), "w", stderr) != nullptr)
			{
				std::cout << "Fusion Output Log start - " << timeString << std::endl;
				std::cerr << "Fusion Error Log start - " << timeString << std::endl;
			}
			else
			{
				std::cout << "ERROR: Fusion Log FILE creation Failed" << std::endl;
			}
		}
		else
		{
			std::cout << "ERROR: Fusion Log FOLDER creation Failed for path " << folderPath << " : " << ec.message() << std::endl;
		}
	}
	else
	{
		std::cout << "ERROR: Fusion Log APPDATA lookup Failed" << std::endl;
	}

	rows = 1;
	bCentralize = false;

	std::string fusion_config_file;
	for (int i = 1; i < argc; ++i)
	{
		if (_stricmp(argv[i], "--fusion_config") == 0 && i + 1 < argc) fusion_config_file = argv[i + 1];
		if (_stricmp(argv[i], "--livecapture") == 0) data_use_network_loader = true;
		if (_stricmp(argv[i], "--runmeshserver") == 0) run_mesh_server = true;
		if (_stricmp(argv[i], "--meshserverport") == 0 && i + 1 < argc)
		{
			run_mesh_server = true;
			g_mesh_server_port = atoi(argv[i + 1]); ++i;
		}
		if (_stricmp(argv[i], "--runmockserver") == 0 && i + 3 < argc)
		{
			run_mesh_server = true;
			run_mock_server = true;
			server_mock_file = argv[i + 1];
			server_mock_begin = atoi(argv[i + 2]);
			server_mock_end = atoi(argv[i + 3]);
			i += 3;
		}
	}

	// if a config wasn't specified on the command line
	if (fusion_config_file.empty())
	{
		std::cout << "No config file specified on command line.  Using 3DTelemedicine_dir environment variable." << std::endl;
		// Try the System directory
		string tmdir = string(getenv("3DTelemedicine_dir"));
		if (!tmdir.empty())
		{
			tmdir += "/3DTelemedicine.cfg";
			if (std::filesystem::exists(std::filesystem::path(tmdir)))
			{
				std::cout << "Using " << tmdir << std::endl;
				fusion_config_file = tmdir;
			}
			// if a config doesn't exist there, look for the basic .default version to create one
			else if (std::filesystem::exists(std::filesystem::path(tmdir + ".default")))
			{
				cerr << "!!! No config file found.  Copying the default in place for use." << std::endl;
				std::filesystem::copy_file(std::filesystem::path(tmdir + ".default"), std::filesystem::path(tmdir));
				fusion_config_file = tmdir;
			}
		}
	}
	if (!fusion_config_file.empty())
	{
		cerr << "!!! using configuration file : " << fusion_config_file << std::endl;
		auto loaded = FusionConfig::initialize_config(fusion_config_file);
		if (!loaded)
		{
			cerr << "attempted to load config file " << fusion_config_file << ", but failed" << endl;
			//exit(0);
		}
		else
		{
			cerr << "  config file " << fusion_config_file << " loaded" << endl;
			auto conf = FusionConfig::current();
			auto keys = conf->GetKeyList("Fusion");
			for (auto k : keys) { cerr << "key " << k << ":" << conf->GetValue<std::string>("Fusion", k) << endl; }
			cerr << endl;
		}
	}

	auto conf = FusionConfig::current();
	if (!conf)
	{
		cerr << "Could not find a config file.  Please create a config file in order to run." << endl;
		return -1;
	}
	
	//debug
	int verbosity = conf->GetValueWithDefault<int>("Fusion", "Verbosity", 0);
	std::string logger_output_filename = conf->GetValueWithDefault<std::string>("Fusion", "LoggerStandardOutput", "");
	std::string logger_error_output_filename = conf->GetValueWithDefault<std::string>("Fusion", "LoggerErrorOutput", "");

	LOGGER()->set_verbosity((Logger::Verbosity)verbosity);
	LOGGER()->set_filename(logger_output_filename.c_str());
	LOGGER()->set_error_filename(logger_error_output_filename.c_str());

	DEPTH_CAMERAS_NUM = conf->GetValueWithDefault<int>("DepthGeneration", "DepthCameraCount", DEPTH_CAMERAS_NUM);
	FrameCountKeyVolume = conf->GetValueWithDefault<int>("Fusion", "FrameCountKeyVolume", FrameCountKeyVolume);

	if (DEPTH_CAMERAS_NUM > MAX_NUM_DEPTH_CAMERAS) {
		LOGGER()->fatal("LiveFusionDemo::main", "The maximum number of pods supported by Fusion4D with the current GPU hardware is %d. Cannot run system with %d pods.", MAX_NUM_DEPTH_CAMERAS, DEPTH_CAMERAS_NUM);
		LOGGER()->close();

		return -1;
	}

	data_use_network_loader = conf->GetValueWithDefault<bool>("Fusion", "livecapture", true);
	//mesh
	run_mesh_server = conf->GetValueWithDefault<bool>("Fusion", "runmeshserver", run_mesh_server);
	g_mesh_server_port = conf->GetValueWithDefault<int>("Ports", "DataStreamPort", g_mesh_server_port);

	//mock
	run_mock_server = conf->GetValueWithDefault<bool>("Fusion", "runmockserver", run_mock_server);
	if (run_mock_server)
	{
		server_mock_file = conf->GetValue<std::string>("Fusion", "mockserver_file");
		server_mock_begin = conf->GetValue<int>("Fusion", "mockserver_begin");
		server_mock_end = conf->GetValue<int>("Fusion", "mockserver_end");
	}

	controlPanelIP = conf->GetValueWithDefault<std::string>("Network", "ControlPanelIPAddress", controlPanelIP);
	myInterfaceIP = conf->GetValueWithDefault<std::string>("Network", "FusionIPAddress", myInterfaceIP);
	eventPort = conf->GetValueWithDefault<std::string>("Ports", "EventPort", eventPort);
	statusPort = conf->GetValueWithDefault<std::string>("Ports", "StatusPort", statusPort);


	FusionCPC = new FusionControlPanelConnector(controlPanelIP, eventPort, myInterfaceIP, statusPort);
	FusionCPC->SetVersionInformation(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_COMMITS, std::string(VERSION_BRANCH_NAME), std::string(VERSION_DESCRIPTION), std::string(VERSION_SHA1));
	GlobalDataStatic::bNextFrameHighQuality = conf->GetValueWithDefault<bool>("Fusion", "HighQualityEnabled", GlobalDataStatic::bNextFrameHighQuality);

	// Make a thread to monitor the CPC for start and stop events and effectuate a shutdown if necessary
	t_control_panel_connector = new std::thread([]() {
		while (true)
		{
			if (FusionCPC->GotStopCommand)
			{
				std::cout << "~~~~~~~~Got a control panel stop command.  Shutting down fusion.~~~~~~~~~~" << std::endl;
				if (g_mesh_server != NULL)
				{
					g_mesh_server->stop_server();
				}
				lock_guard<std::mutex> lock_cuda_res(GlobalDataStatic::mutex_cuda);
				GlobalDataStatic::bStopFusion = !GlobalDataStatic::bStopFusion;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		});

	if (run_mock_server)
	{
		g_mesh_server = new TofinoMeshServer();

		g_mesh_server->initialize_mock_server(server_mock_file.c_str(), server_mock_begin, server_mock_end, g_mesh_server_port);
		g_mesh_server->set_run_asynchronous(false);

		//run this anyways
		t_mesh_server = new std::thread([]() {
			fprintf(stderr, "starting mesh server loop\n");
			g_mesh_server->server_mainloop();
			fprintf(stderr, "exiting mesh server loop\n");
			});

		while (true)
		{
			fprintf(stderr, "running mock...\n");
			Sleep(3000);
		}
		exit(0);
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_MULTISAMPLE);
	glutInitWindowSize(windowWidth, windowHeight);
	glutInitWindowPosition(100, 100);
	char windowname[500];
	sprintf(windowname, "3DTelemedicine | Live Fusion");
	glutCreateWindow(windowname);
	glutCloseFunc(on_window_closed);

	init();
	FusionCPC->SendStatusUpdate(CPC_STATUS::READY);
	FusionCPC->SendBuildVersion();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(key);
	glutMouseFunc(mouse);
	glutMotionFunc(mouseTrack);
	glutIdleFunc(idle);

	glutMainLoop();
	FusionCPC->SendStatusUpdate(CPC_STATUS::STOPPED);

	LOGGER()->close();

	return 0;
}

