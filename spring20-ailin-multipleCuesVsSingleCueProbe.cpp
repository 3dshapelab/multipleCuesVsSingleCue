// visual angel 6.5 + textureNormalizerbase = 77 -> 3mm
// 

#pragma once
// The following macros define the minimum required platform.  The minimum required platform
// is the earliest version of Windows, Internet Explorer etc. that has the necessary features to run 
// your application.  The macros work by enabling all features available on platform versions up to and 
// including the version specified.

// Modify the following defines if you have to target a platform prior to the ones specified below.
// Refer to MSDN for the latest info on corresponding values for different platforms.
#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif


#include <stdio.h>
#include <tchar.h>


// TODO: reference additional headers your program requires here
#include <cstdlib>
#include <cmath>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <queue>
//Library for texture mapping
#include "SOIL.h"

/**** BOOOST MULTITHREADED LIBRARY *********/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"


#ifdef _WIN32
#include <windows.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>
#endif

/************ INCLUDE CNCSVISION LIBRARY HEADERS ****************/
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "CylinderPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "TrialGenerator.h"
#include "ParametersLoader.h"
#include "Util.h"
#include "VRCamera.h"
/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;


	#include <direct.h>
	#include "Optotrak2.h"
	#include "Marker.h"
	#include "BrownMotorFunctions.h"
	using namespace BrownMotorFunctions;
/********* #DEFINE DIRECTIVES **************************/
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
static const double SCREEN_WIDE_SIZE = 383;//306;    // millimeters

/*************** Variable Declarations ********************/

/********* November 2019   CALIBRATION ON CHIN REST *****/
static const Vector3d calibration(160,179,-75);
//static const Vector3d objCalibration(199.1, -149.2, -319.6);
// Alignment between optotrak z axis and screen z axis
double alignmentX = 33.5;
double alignmentY = 33;// - 49 - 55.0/2.0;
double focalDistance= -270.0, homeFocalDistance=-270.0;
static const Vector3d center(0,0,focalDistance);
double mirrorAlignment=0.0, screenAlignmentY=0.0, screenAlignmentZ=0.0;
Screen screen;
static const Vector3d centercal(29.75,-133.94,-296.16); //updated 9/25/14


/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode=true;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords, thumbCoords,indexCoords;
/********** STREAMS **************/
ofstream responseFile;
/********** EYES AND MARKERS **********************/

Vector3d eyeLeft, eyeRight, eyeMiddle;
vector <Marker> markers;
static double interoculardistance=60.0;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;

/********** VISUALIZATION AND STIMULI ***************/
Timer timer;
Timer globalTimer;
bool visibleInfo = true;
//TEXTURE LOADING VARIABLE
GLuint texture0[51];
GLuint texture1[51];
GLuint texture2[51];
GLuint texture3[51];
GLuint* texture[51];

// Cylinder Buffers (drawing one wave per trial)
//GLfloat *vertices = new GLfloat[1];
GLfloat vertices[10000000];
GLfloat vertices_projected[10000000];
GLfloat normals[10000000];
GLuint indices[10000000];
GLfloat texcoors[10000000];
int vertex_index, tex_index = 0; int iFace; int nr_triangles; int nr_vertices;

//color arrays, where texture is defined
GLfloat colors[5000000];

/********* LIGHTING ***************/
float ambientRed = 0.25;
GLfloat LightAmbient[] = { ambientRed, 0.f, 0.f, 1.0f };

float diffuseRed = 0.95;
GLfloat LightDiffuse[] = { diffuseRed, 0.0f, 0.0f, 1.0f };

//float LightDist = 120.0;
float lightPos_X = 0.0;
float lightPos_Y = 80.0;
float lightPos_Z = 120.0;
GLfloat LightPosition[] = {lightPos_X, lightPos_Y, lightPos_Z, 1.0f};

float specularValue = 1.0;
GLfloat LightSpecular[] = { specularValue, specularValue, specularValue, 0.0f };

float specularMaterialValue = 0.2;
GLfloat specularMaterial[] = { specularMaterialValue, specularMaterialValue, specularMaterialValue, 1.0 };

GLfloat shininessMaterial = 70.0f;

/********** TRIAL SPECIFIC PARAMETERS ***************/
//Vector3d dot_placement[100000];
ParametersLoader parameters;
BalanceFactor<double> trial;
const float DEG2RAD = M_PI / 180;
static const int THREERODS = 0;

int blkNum = 1;
int trialNumBlk = 0;
int texnum = 0;

//controls the factors 
bool checkInfo = false;
bool training = true;
bool screenBack = false; //has the screen been placed at projection distance
bool finished = false;
double ElapsedTime;
int trialNumber = 1;
int num_depth = 5;
int bin = 0;
double totalTrial = num_depth * 20.0; // 30 bins

// Timer variable, set for each trial 
Timer trial_time;
double trialTime; // will be set for each trial
double drawStimulusTime = 1000; //ms, time for fixation to be present



/********** STIMULUS SPECIFIC PARAMETERS ***************/
// stimulus 
double display_distance;
double visual_angle = 6; // stim diangonal size
double min_depth = 1; // set in the subject file
double max_depth = 50; // set in the subject file
double edge = 70; 
double edge_apparent = edge;//tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double cylinder_width = 1.2 * edge;

// estimated 2D probe
double estimated_depth = 0; // variable that subjects will populate with the 2D sine wave
double twoD_starting_depth = 0; // variable to note what the random starting 2D depth is
double depth = 30;
int texture_type = 0;
double probe_edge = 100; // height of 2D sine wave to include only 1 period of 3D sine wave
double circle_radius = 1;
double probe_x_offset = 80;
double probe_y_offset = 70;

// For texture:
double normalizer_base = 70; // maximum distance along the side on the sine wave fluctuating in depth
double normalizer = normalizer_base; //will vary based on display distance
double minimum_distance = -270.;// closest distance sine wave could be presented
double maximum_distance = -470.;// furthest distance sine wave could be presented
double normalizer_scalefactor = (minimum_distance/minimum_distance); // scale the normalizer by distance to make sure dots are the same VA

// panel size
double panel_w = 30;
double panel_h = 40;
double panel_separation = 45;
int apertureState = 1; // 0 - no aperture; 1 - aperture black; 2 - aperture dark red

// stimulus depth
double depth_text = 40;
double depth_disp = 40;

bool text_disp_conflict = true;
/*************************** EXPERIMENT SPECS ****************************/
string subjectName;


// experiment directory

string experiment_directory = "C:/Users/visionlab/Documents/data/ailin/spring20-ailin-cueCompProbe";

// paramters file directory and name
string parametersFile_directory = experiment_directory + "/parameters_spring20-ailin-cueCompProbe.txt";

// response file headers
string responseFile_headers = "subjName\tIOD\ttrialN\tbin\ttextureType\ttexnum\tdisplayDistance\tvisualAngle\tprobeStart\tdisparityDepth\ttextureDepth\tperceivedDepth\tambientRed\tdiffuseRed\tlightPos_Y\tlightPos_Z\ttime";

/*************************** FUNCTIONS ***********************************/
void initOptotrak();
void initMotors();
int LoadGLTextures();
void initRendering();
void initVariables();
void initStreams();
void handleResize(int w, int h);
void drawStimulus();
void drawCylinder();
void drawProbe(double probe_depth);
double calculateDepth(double depth, double y);
void drawMarkerPosition();
void initTrial();
void advanceTrial();
void updateTheMarkers();
void drawInfo();
void beepOk(int tone);
void cleanup();
void initProjectionScreen(double _focalDist, const Affine3d &_transformation=Affine3d::Identity(),bool synchronous=true);
void online_apparatus_alignment();
void buildCylinder(double textureDepth, double dispDepth);
void drawAperture();
void lighting();
// This function seems to be used to shut down the system after use
void shutdown(){
	cout << "shutting down" << endl;
	responseFile.close(); // close this object
	homeEverything(5000,4500);
	cleanup();
	exit(0);
}
void cleanup() 
{
// Stop the optotrak
    optotrak->stopCollection();
    delete optotrak;
}
void initProjectionScreen(double _focalDist, const Affine3d &_transformation, bool synchronous)
{
	focalDistance = _focalDist;	
    screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH);
    screen.setOffset(alignmentX,alignmentY);
    screen.setFocalDistance(_focalDist);
    screen.transform(_transformation);
    cam.init(screen);
	if ( synchronous )
		moveScreenAbsolute(_focalDist,homeFocalDistance,4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,4500);
}
// Initialize Optotrak for use in the experiment
void initOptotrak()
{
    optotrak=new Optotrak2(); //intiailize the Optotrak object
    optotrak->setTranslation(calibration);

	//define Optotrak-specific variables
    int numMarkers=22;
    float frameRate=85.0f;
    float markerFreq=4600.0f;
    float dutyCycle=0.4f;
    float voltage = 7.0f;

	// run the intiailization method for the Optotrak, checking to see if ever (if == 0) and catch the error if so
    if ( optotrak->init("C:/cncsvisiondata/camerafiles/Aligned20111014",numMarkers, frameRate, markerFreq, dutyCycle,voltage) != 0)
    {   cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
        cin.ignore(1E6,'\n');
        exit(0);
    }

    // Read 10 frames of coordinates and fill the markers vector
    for (int i=0; i<10; i++)
    {
        updateTheMarkers();
    }
}
// run a method to define a vector that holds marker positions 
void updateTheMarkers()
{
	optotrak->updateMarkers();
	markers = optotrak->getAllMarkers();

}
// Initialize motors for moving screen around
void initMotors()
{
	//specify the speed for (objects,screen)
	homeEverything(5000,4500);
}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{   
glClearColor(0.0,0.0,0.0,1.0);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
/* Set depth buffer clear value */
glClearDepth(1.0);
/* Enable depth test */
glEnable(GL_DEPTH_TEST);
/* Set depth function */
glDepthFunc(GL_LEQUAL);
// scommenta solo se vuoi attivare lo shading degli stimoli

glMatrixMode(GL_MODELVIEW);
glLoadIdentity();
// Tieni questa riga per evitare che con l'antialiasing attivo le linee siano piu' sottili di un pixel e quindi
// ballerine (le vedi vibrare)
glLineWidth(1.5);
lighting();
}
//Extracts constants from parameters file. Used in the method defined below
void initVariables()
{
	// initialize the trial matrix
	trial.init(parameters);
	trial.next();
	interoculardistance = atof(parameters.find("IOD").c_str());
	// eye coordinates
	eyeRight = Vector3d(interoculardistance/2,0,0);
	eyeLeft = Vector3d(-interoculardistance/2,0,0);
	eyeMiddle = Vector3d(0,0,0);
	display_distance = str2num<double>(parameters.find("dispDepth"));
	edge_apparent = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
	probe_edge = edge_apparent;

}
// Initialize the streams, open the file and write to it
void initStreams()
{
	// Initialize the parameter file starting from the file parameters.txt, if the file does not exist, it tells you
	ifstream parametersFile;
	parametersFile.open(parametersFile_directory.c_str());
	parameters.loadParameterFile(parametersFile);
	// Subject name
	subjectName = parameters.find("SubjectName");

	string responseFileName = experiment_directory +"/"+ subjectName  + ".txt";
	// Principal streams files
	if (util::fileExists(experiment_directory +"/"+subjectName+".txt") && subjectName != "junk")
	{
		string error_on_file_io = experiment_directory+"/"+subjectName+".txt" + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.",NULL, NULL);
		shutdown();
	}
	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;
	
	globalTimer.start();
}
//Central function for projecting image onto the screen
void drawGLScene()
{
	// Note that in this experiment, there is no difference between drawing left and right because
	// we are displaying to the middle (there is no stereo)
	// Draw left eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeLeft);//cam.setEye(eyeMiddle); 
	drawStimulus();
	drawInfo();


	// Draw right eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0,0.0,0.0,1.0);
	cam.setEye(eyeRight); //cam.setEye(eyeMiddle);
	drawStimulus();
	drawInfo();
	
	glutSwapBuffers();
}
void update(int value)
{
    glutPostRedisplay();
    glutTimerFunc(TIMER_MS, update, 0);
}

void drawInfo()
{
	// displays relevant information to the screen
	if ( visibleInfo )
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);

		GLText text;

		if ( gameMode )
			text.init(SCREEN_WIDTH,SCREEN_HEIGHT,glWhite,GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640,480,glWhite,GLUT_BITMAP_HELVETICA_12);

		text.enterTextInputMode();
		if (finished) {
			GLText text;

			if (gameMode)
				text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
			else
				text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);
			text.enterTextInputMode();
			glColor3fv(glWhite);
			text.draw("The experiment is over. Thank you! :)");
			text.leaveTextInputMode();
		} else {
		
			// check if IOD has been input
			if(interoculardistance>45.0)
			{
				text.draw("####### SUBJECT #######");
				text.draw("#");
				text.draw("# Name: " + parameters.find("SubjectName"));
				
			} else {
				text.draw(" "); text.draw(" "); text.draw(" ");
			}
			text.draw("# IOD: " +stringify<double>(interoculardistance));

			
			// check if mirror is calibrated
			if ( abs(mirrorAlignment - 45.0) < 0.2 )
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("# Mirror Alignment = " +stringify<double>(mirrorAlignment));

			// check if monitor is calibrated
			if ( screenAlignmentY < 89.0 )
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("# Screen Alignment Y = " +stringify<double>(screenAlignmentY));
			if ( abs(screenAlignmentZ) < 89.0 )
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("# Screen Alignment Z = " +stringify<double>(screenAlignmentZ));

			text.draw(" ");

			// if IOD has been input
			if(checkInfo)
			{
				glColor3fv(glWhite);
				//text.draw("# trial: " +stringify<int>(trialNumber));
				//text.draw("# disparity depth: " +stringify<int>(depth_disp));
				//text.draw("# texture depth: " +stringify<int>(depth_text));
				//text.draw("# estimated depth: " +stringify<int>(estimated_depth));
				//text.draw("# Texture type: " + stringify<int>(texture_type));
				//text.draw("# Texture base: " + stringify<double>(normalizer_base));
				//text.draw("# texnum: " + stringify<int>(texnum));
				//text.draw("# apertureState: " + stringify<int>(apertureState));
		
		text.draw("# diffuseRed: " + stringify<float>(diffuseRed));
		text.draw("# ambientRed: " + stringify<float>(ambientRed));

		text.draw("# lightPos_Y: " + stringify<float>(lightPos_Y));
		text.draw("# lightPos_Z: " + stringify<float>(lightPos_Z));
		text.draw("# depth: " + stringify<double>(depth));

				text.leaveTextInputMode();
				glEnable(GL_COLOR_MATERIAL);
				glEnable(GL_BLEND);
			}
		}
	}
}

void drawFixation() {
	// draws a small fixation cross at the center of the display
	glDisable(GL_TEXTURE_2D);
	glColor3f(0.1f, 0.0f, 0.0f);
	glLineWidth(2.f);
	glLoadIdentity();
	glTranslated(0, 0, display_distance - depth_disp + 2);
	double cross_length = 5;
	glBegin(GL_LINES);
	glVertex3d(cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(-cross_length / 2. + circle_radius / 2., 0, 0);
	glVertex3d(circle_radius / 2, -cross_length / 2. , 0);
	glVertex3d(circle_radius / 2, cross_length / 2. , 0);
	glEnd();
}

void drawAperture(){
	//double panel_separation = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance) - max_depth); 
	//panel_separation = edge * (display_distance + max_depth - depth_disp)/(display_distance - depth_disp);
	panel_separation = edge_apparent;

	//glDisable(GL_TEXTURE_2D);
	glLoadIdentity();
	//glTranslated(0,0,display_distance + max_depth - depth_disp);
	glTranslated(0,0,display_distance);

	if(apertureState == 2){
		glColor3f(0.8f, 0.0f, 0.0f);
	}else{
		glColor3f(0.0f, 0.0f, 0.0f);
	}
	glBegin(GL_QUADS); 
	glVertex3f(-panel_separation/2 - panel_w, panel_h, 0.0f);
	glVertex3f(-panel_separation/2 , panel_h, 0.0f);
	glVertex3f(-panel_separation/2 , -panel_h, 0.0f);
	glVertex3f(-panel_separation/2 - panel_w, -panel_h, 0.0f);
	glEnd();

	glBegin(GL_QUADS); 
	glVertex3f(panel_separation/2 , panel_h, 0.0f);
	glVertex3f(panel_separation/2 + panel_w, panel_h, 0.0f);
	glVertex3f(panel_separation/2  + panel_w, -panel_h, 0.0f);
	glVertex3f(panel_separation/2, -panel_h, 0.0f);
	glEnd();
}
// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}

void drawStimulus()
{
	// conditional to compare trialTime to determine what to present
	if (trialTime < drawStimulusTime) {
	// here, draw a fixation cross for 1000 ms 
		drawFixation();
	} else if (trialTime > drawStimulusTime) {
	// Then draw the experimental stimulus	
		drawCylinder();
		drawProbe(estimated_depth);
		 
		//draw the 2D probe line
		if(apertureState > 0){
			//drawAperture();
		}
	}
}


// calculate the depth for a given y coordinate value
double calculateDepth(double depth, double y){
	//use the equation for an ellipse to get the depth
	double z = depth * sqrt(1 - pow(y/(edge/2.),2));	
	return (z);
}


void buildCylinder(double textureDepth, double dispDepth) {

	// edge is the long axis of the ellipse (along y axis), the short axis of the ellipse is along z axis
	edge = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance) + depth);
	// the x dimension, does not change the shape of the ellipse, but it is the length of the cylinder bar
	cylinder_width = 1.2 * edge;
	if (textureDepth == dispDepth) {
		text_disp_conflict = false;
	}
	else {
		text_disp_conflict = true;
	}
	cout << "Building Texture" << endl;

	double nr_points = 500; // nr of points in x and y
	double step_size = (cylinder_width / (nr_points - 1));

	// build the meshgrid point by point
	// indices for buffers
	vertex_index = 0; tex_index = 0;
	Vector3d prev_side_point;

	//preallocate the first x,y,z coordinates for previous side point
	prev_side_point[0] = -cylinder_width / 2; prev_side_point[1] = -edge / 2;
	prev_side_point[2] = calculateDepth(textureDepth, (-edge / 2.));
	int nr_vertices_per_row = nr_points;

	double total_distance = 0; //tracks the distance along y/z axis, approximate the "diameter" of the ellipse
	double y; double x; double z;// preallocate variables to loop through the meshgrid

	for (y = -edge / 2; y <= edge / 2 - step_size; y = y + step_size) {  // 

		z = calculateDepth(textureDepth, y); // Get depth for this point

		for (x = -cylinder_width / 2; x <= cylinder_width / 2 - step_size; x = x + step_size) { //
			// here, we have all of the coordinates needed for this point. 
			// append the appropriate x,y,z for each point into the array storing vertices
			// also do color in the same iteration to be more efficient with only one index counter

			// we also calculate the normal for each vertex for shading
			// the normal of the ellipse is the point on the axis (x, 0, 0) to (x, y, z),
			// which is (0, y, z) before normalization
			vertices[vertex_index] = x;  // this is x
			colors[vertex_index] = 1;    //R is this value
			normals[vertex_index] = 0; // this is x
			vertex_index++;
			vertices[vertex_index] = y; // this is y
			colors[vertex_index] = 0;  // G is this value
			normals[vertex_index] = y * textureDepth * textureDepth; // this is y
			vertex_index++;
			vertices[vertex_index] = z; // this is z
			colors[vertex_index] = 0; // B is this value
			normals[vertex_index] = z * textureDepth * textureDepth / 4; // this is z
			vertex_index++;

			texcoors[tex_index] = (x + edge / 2) / normalizer; tex_index++;//u coordinate
			texcoors[tex_index] = total_distance / normalizer; tex_index++;//v coordinate

			prev_side_point[0] = x;

			if (text_disp_conflict) { // need to backproject the disp vertices locations
				double a = z;
				double b = y;
				double h = edge / 2;
				double l = -display_distance;
				double denom = pow(a * h, 2) - 2 * a * l * pow(h, 2) + pow(b * dispDepth, 2) + pow(h * l, 2);
				double projectedZ =
					(sqrt(pow(dispDepth * h * (a - l), 2) * (denom - pow(b * l, 2))) + pow(b * dispDepth, 2) * l) / denom;
				vertices_projected[vertex_index - 1] = projectedZ;
				vertices_projected[vertex_index - 2] = (b / (a - l)) * (projectedZ - l);
				vertices_projected[vertex_index - 3] = x;
			}

		}
		total_distance = total_distance + sqrt(pow(y - prev_side_point[1], 2) + pow(z - prev_side_point[2], 2));
		prev_side_point[1] = y; prev_side_point[2] = z;
	}

	// total nr of vertices is the index counter for vertices divided by 3
	nr_vertices = vertex_index / 3;

	// having created the meshgrid above, I need to create the associated indices of vertex lcations
	// in the 'vertices' array for drawing.reminder that nr_points is the steps in the y column
	// Modified from Evan's code:
	iFace = 0; // this should end up being 3*numFaces
	for (int vi = 0; vi < (nr_vertices - nr_vertices_per_row); vi++) {
		if ((vi % nr_vertices_per_row) != (nr_vertices_per_row - 1)) {
			indices[iFace] = vi; iFace++;
			indices[iFace] = vi + 1; iFace++;
			indices[iFace] = vi + nr_vertices_per_row; iFace++;

			indices[iFace] = vi + 1; iFace++;
			indices[iFace] = vi + nr_vertices_per_row + 1; iFace++;
			indices[iFace] = vi + nr_vertices_per_row; iFace++;
		}
	}
	nr_triangles = (iFace) / 3; // nr of triangles is the iFace counter divided by 3
	cout << "FINISHED!" << endl;

}


void drawProbe(double probe_depth){

	probe_edge = edge;

	glDisable(GL_TEXTURE_2D);

	double nr_points = 1000; // nr of points in x and y
	double step_size = (probe_edge/(nr_points-1));

	double prev_x; double prev_y; // holder for drawing lines

	glPushMatrix();
	// draw the sine wave in 2D
	glLoadIdentity();
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(4.f);

		glTranslated(0, -probe_y_offset, display_distance); 
		glRotatef(-90.0, 0.0, 0.0 ,1.0); 
	
	
	for (double x = probe_edge/2.; x >= -probe_edge/2. ; x = x - step_size) {
				
		double y = probe_depth * sqrt(1 - pow(x/(probe_edge/2.),2)); // Get depth for this point

		if (x == probe_edge/2.) { // if the first time, don't draw anything but assign this point to previous x and y

				prev_x = x; prev_y = y;

		} else {
				glBegin(GL_LINES);
					glVertex2f(prev_x, prev_y);
					glVertex2f(x, y);
				glEnd();
				
				prev_x = x; prev_y = y; 
		}
	}
	glPopMatrix();
}

void drawCylinder() {

	//glEnable(GL_LIGHT1);
	//glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);
	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5f);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	//glTranslated(0, 0, display_distance - depth);



	//glMaterialfv(GL_FRONT, GL_SPECULAR, specularMaterial);
	//glMaterialf(GL_FRONT, GL_SHININESS, shininessMaterial);
	//glMaterialfv(GL_BACK, GL_SPECULAR, specularMaterial);
	//glMaterialf(GL_BACK, GL_SHININESS, shininessMaterial);
	//glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);

	// enable matrices for use in drawing below
	glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces
	//glColorPointer(3, GL_FLOAT, 0, colors);
	// glColor3f(1.0f, 0.0f, 0.0f);


	glLoadIdentity();
	glTranslated(0, 0, display_distance - depth_disp);


	// bind the texture
	if (texture_type == 0) {
		glBindTexture(GL_TEXTURE_2D, texture0[texnum]);
	}
	else if (texture_type == 1) {
		glBindTexture(GL_TEXTURE_2D, texture1[texnum]);
	}
	else if (texture_type == 2) {
		glBindTexture(GL_TEXTURE_2D, texture2[texnum]);
	}
	else {
		glBindTexture(GL_TEXTURE_2D, texture3[texnum]);
	}

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);



	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	if (text_disp_conflict) {
		glVertexPointer(3, GL_FLOAT, 0, vertices_projected);
		glNormalPointer(GL_FLOAT, 0, normals); // put the normals in 
		glColorPointer(3, GL_FLOAT, 0, colors);
		glTexCoordPointer(2, GL_FLOAT, 0, texcoors);
	}
	else {
		glVertexPointer(3, GL_FLOAT, 0, vertices);
		glNormalPointer(GL_FLOAT, 0, normals);
		glColorPointer(3, GL_FLOAT, 0, colors);
		glTexCoordPointer(2, GL_FLOAT, 0, texcoors);
	}

	

	// Draw the 3D sine wave
	glDrawElements(GL_TRIANGLES, iFace, GL_UNSIGNED_INT, indices);

	// deactivate vertex arrays after drawing
	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

}

void initTrial()
{
	// get the trial parameters for this trial and re-define sine wave parameters dependant on display distance
	initProjectionScreen(display_distance);

	//texture_type = trial.getCurrent()["textureTypes"]; // texture type from the subject parameters file
		bin = ceil( double(trialNumber) / double(num_depth) );

		//if(bin <= 5 || bin > 15){
		if(bin <= 5 || bin > 15){
			texture_type = 3;
			LightAmbient[0] = 0.5;
			LightDiffuse[0] = 0.0;
		}else{
			texture_type = 0;
			LightAmbient[0] = 0.30;
			LightDiffuse[0] = 0.50;
		}

	depth_disp = trial.getCurrent()["testDepths"];
	depth_text = depth_disp;

	// recalculate the edge size based on this tial's parameters

	// here, make the sine wave a bit bigger so that the edges are totally hiddne by the viewing aperture
	//chooswe random generated texture
	texnum = (rand() % 50) + 1;
	
	// scale the texture normalizer to ensure dots are the same visual angle
	if (display_distance == minimum_distance) {
		normalizer_scalefactor = (minimum_distance/minimum_distance);
	} else if (display_distance == maximum_distance) {
		normalizer_scalefactor = (maximum_distance/minimum_distance);
	}
	normalizer = normalizer_base * normalizer_scalefactor; // maximum distance along the sine wave edge


	// preplan the vertices (in a vector) and an array of colors and give to OpenGL
	buildCylinder(depth_text, depth_disp);

	//assign a random depth to 'estimated_depth' variable used for creating the 2D sine wave
	twoD_starting_depth = (rand() % (int) max_depth) + min_depth;
	estimated_depth = twoD_starting_depth;

	// reset TrialTime variables
	trialTime = 0;
	trial_time.start(); // start the timer
}

void advanceTrial()
{
	//"subjName\tIOD\ttrialN\ttextureType\ttexnum\tdisplayDistance\tvisualAngle\tprobeStart\tdisparityDepth\ttextureDepth\tperceivedDepth\ttime\tisRotated";
	responseFile << fixed <<
	parameters.find("SubjectName") << "\t" <<		//subjName
	interoculardistance << "\t" <<
	trialNumber << "\t" <<
	bin << "\t" <<
	texture_type << "\t" <<
	texnum << "\t" <<
	display_distance << "\t" <<
	visual_angle << "\t" <<
	twoD_starting_depth << "\t" <<
	depth_disp << "\t" <<
	depth_text << "\t" <<
	LightAmbient[0] << "\t" << 
	LightDiffuse[0] << "\t" << 
	lightPos_Y << "\t" << 
	lightPos_Z << "\t" << 
	trialTime << endl;	//trialN
	

	finished = trial.isEmpty();
	trialNumber++;
	trial.next();
	if(!finished){
		beepOk(4);
		initTrial();
	}else{
		beepOk(1);
		responseFile.close();
		visibleInfo=true;
		finished = true;
	}
}

// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{   
	//cout << "listening for keypress" << endl;
	switch (key)
    {   
	case 'k':
	case 'K':
		{ 
			depth = depth + 5;
			buildCylinder(depth, depth);

		}
		break;

	case 'l':
	case 'L':
		{  
			depth = depth - 5;
			buildCylinder(depth, depth);
		}
		break;




	case 'd':
	case 'D':
		{
		if(diffuseRed <= 1.0){  
			diffuseRed = diffuseRed + 0.05;		
			LightDiffuse[0] = diffuseRed;
			}
		}
		break;

	case 's':
	case 'S':
		{ 
		if(diffuseRed >= 0){ 
			diffuseRed = diffuseRed - 0.05;		
			LightDiffuse[0] = diffuseRed;
			}
		}
		break;

	case 'o':
	case 'O':
		{  
		lightPos_Y = lightPos_Y - 10;
		LightPosition[1] = lightPos_Y;
			
		}
		break;

	case 'p':
	case 'P':
		{  
		lightPos_Y = lightPos_Y + 10;
		LightPosition[1] = lightPos_Y;
		}
		break;

	case 'm':
	case 'M':
		{  
		lightPos_Z = lightPos_Z + 10;
		LightPosition[2] = lightPos_Z;
		}
		break;

	case 'n':
	case 'N':
		{  
		lightPos_Z = lightPos_Z - 10;
		LightPosition[2] = lightPos_Z;
		}
		break;
		case 'Q':
		case 'q':
		case 27:	//corrisponde al tasto ESC
		{   
			// Ricorda quando chiami una funzione exit() di chiamare prima cleanup cosi
			// spegni l'Optotrak ed i markers (altrimenti loro restano accesi e li rovini) 
			shutdown();
		}
		break;


		case 'f':
		case 'F':
		{
			//Initializes trial without the use of finger calibrations
			//fingersCalibrated=false;
			if(!screenBack){
				beepOk(0);
				initProjectionScreen(display_distance);
				screenBack = true;
			}else{
				beepOk(0);
				visibleInfo=false;
				//trial.next(false);
				initTrial();
			}
			
		}
		break;

		case '4': // 1 decreases the width of the 2D sine wave until it gets really close to 0
		{
			if (estimated_depth > 0.01) {
				estimated_depth = estimated_depth - 4.0; // decease the depth by 1/10th of a millimeter
			} else {
				estimated_depth = estimated_depth;
			}
			//cout << "Depth decreased to : " << estimated_depth << endl;
		}
		break;
		case '5': // 2 increases the depth of the 2D sine wave until it gets really large (arbitrarily set upper bound to 50 mm)
		{
			if (estimated_depth < 66) {
				estimated_depth = estimated_depth + 4.0; // decease the depth by 1/10th of a millimeter
			} else {
				estimated_depth = estimated_depth;
			}
			//cout << "Depth increased to : " << estimated_depth << endl;

		}
		break;

		case '1': // 1 decreases the width of the 2D sine wave until it gets really close to 0
		{
			if (estimated_depth > 0.01) {
				estimated_depth = estimated_depth - 1.0; // decease the depth by 1/10th of a millimeter
			} else {
				estimated_depth = estimated_depth;
			}
			//cout << "Depth decreased to : " << estimated_depth << endl;
		}
		break;
		case '2': // 2 increases the depth of the 2D sine wave until it gets really large (arbitrarily set upper bound to 50 mm)
		{
			if (estimated_depth < 66) {
				estimated_depth = estimated_depth + 1.0; // decease the depth by 1/10th of a millimeter
			} else {
				estimated_depth = estimated_depth;
			}
			//cout << "Depth increased to : " << estimated_depth << endl;

		}
		break;

		case 'r':
		{
			normalizer_base = normalizer_base + 5 ;
			initTrial();
			buildCylinder(depth_text, depth_disp);
			drawCylinder();
			drawAperture();
		}
		break;
		case 'e':
		{
			normalizer_base = normalizer_base - 5 ;
			initTrial();
			buildCylinder(depth_text, depth_disp);
			drawCylinder();
			drawAperture();
		}
		break;

		case 'a':
		{
        apertureState++;
		apertureState = apertureState % 3;
		}

		case '+': // advance to next trial
		{
			if (trialTime > drawStimulusTime){
				advanceTrial();
			}
			
		}
		break;

		case 'i':
		checkInfo = !checkInfo;
		break;
		
	}
}
/***** SOUNDS *****/
void beepOk(int tone)
{

	switch(tone)
	{
	case 0:
    // Remember to put double slash \\ to specify directories!!!
    PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-1.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;
	
	case 1:
    PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-6.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;
	
	//When the participant has either hit the ceiling or floor
	case 3:
    PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-reject.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	//a clicking beat, signals the participant adjusting the stimuli
	case 4:
    PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-lowBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 15:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-rising.wav",
			NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 17:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck.wav",
			NULL, SND_FILENAME | SND_ASYNC);
	break;

	case 18:
	PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck-5below.wav",
		NULL, SND_FILENAME | SND_ASYNC);
	break;

	}
	return;
}
void idle()
{
	//cout << "got to idle function" << endl;
	online_apparatus_alignment();
	trialTime = trial_time.getElapsedTimeInMilliSec();
}

void drawMarkerPosition(){
	// this function displays the marker position for one of the markers set up for use with fingers
	// used to display marker position and test calibration of the Optotrak in the depth plane

	GLText text;

	if (gameMode)
		text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
	else
		text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

	text.enterTextInputMode();

	// Draw the position of the markers
	glColor3fv(glWhite);
	text.draw(" ");
	text.draw("marker position: ");
	if (isVisible(markers[15].p) && isVisible(markers[17].p) && isVisible(markers[18].p))
		glColor3fv(glGreen);
	else
		glColor3fv(glRed);
	text.draw("Marker " + stringify<int>(15) + stringify< Eigen::Matrix<double, 1, 3> >(markers[15].p.transpose()) + " [mm]");
	text.draw("Marker " + stringify<int>(17) + stringify< Eigen::Matrix<double, 1, 3> >(markers[17].p.transpose()) + " [mm]");
	text.draw("Marker " + stringify<int>(18) + stringify< Eigen::Matrix<double, 1, 3> >(markers[18].p.transpose()) + " [mm]");
}

/*** Online operations ***/
void online_apparatus_alignment()
{
	updateTheMarkers();
    // mirror alignment check
    mirrorAlignment = asin(
        abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
        sqrt(
        pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
        pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
        )
        ) * 180 / M_PI;

    // screen Y alignment check
    screenAlignmentY = asin(
		abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
		sqrt(
		pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
		pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
		)
		) * 180 / M_PI;

    // screen Z alignment check
    screenAlignmentZ = asin(
		abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
		sqrt(
		pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
		pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
		)
		) * 180 / M_PI *
		abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
		(markers.at(screen1).p.x() - markers.at(screen2).p.x());

}

void lighting() {

	// Set up the lighting
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	//glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	//glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	//glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);
	//glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5f);

	//glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT1);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

}


// this is run at compilation because it's titled 'main'
int main(int argc, char*argv[])  
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();
	
	// initializes optotrak and velmex motors
	initOptotrak();
	initMotors();
	
	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();
	

	// initializing experiment's parameters
	
	initRendering(); // initializes the openGL parameters needed for creating the stimuli
	
	initStreams(); // streams as in files for writing data

	LoadGLTextures();
	//cout << "Loaded texture in main loop" << endl << endl;

	// glut callback, OpenGL functions that are infinite loops to constantly run 
	glutDisplayFunc(drawGLScene); // keep drawing the stimuli
	glutKeyboardFunc(handleKeypress); // check for keypress
	glutReshapeFunc(handleResize);
	glutIdleFunc(idle);
	glutTimerFunc(TIMER_MS, update, 0);
	glutSetCursor(GLUT_CURSOR_NONE);
	//cout << "Got through the glut stuff in main loop" << endl << endl;

	boost::thread initVariablesThread(&initVariables); 
	// Application main loop
	glutMainLoop();
	// When the program exists, clean up
	cleanup();
	return 0;
}


int LoadGLTextures()  // Load PNG And Convert To Textures
{
    //load an image file directly as a new OpenGL texture 
	for (int texture_file = 0; texture_file <= 3; texture_file++){
		for (int texture_num = 0; texture_num <= 50; texture_num++){
			 ostringstream texPath; 
			 ostringstream texNum; 
  
			// Sending a number as a stream into output 
			// string 
			texPath << texture_file; 
			texNum << texture_num;
			string texturePath = "fall16-ailin-stimulusTest/" + texPath.str() + "/polkadots" + texNum.str() + ".png";
			//cout << texturePath.c_str() <<endl;
			//cout << "fall19-james-textureProbeTask/polkadots.png" <<endl;
			if (texture_file == 0){
				texture0[texture_num] = SOIL_load_OGL_texture
					(
					texturePath.c_str(),
					SOIL_LOAD_AUTO,
					SOIL_CREATE_NEW_ID,
					SOIL_FLAG_MULTIPLY_ALPHA
					);
			} else if(texture_file == 1){
				texture1[texture_num] = SOIL_load_OGL_texture
					(
					texturePath.c_str(),
					SOIL_LOAD_AUTO,
					SOIL_CREATE_NEW_ID,
					SOIL_FLAG_MULTIPLY_ALPHA
					);
			} else if (texture_file == 2){
				texture2[texture_num] = SOIL_load_OGL_texture
					(
					texturePath.c_str(),
					SOIL_LOAD_AUTO,
					SOIL_CREATE_NEW_ID,
					SOIL_FLAG_MULTIPLY_ALPHA
					);
			} else {
				texture3[texture_num] = SOIL_load_OGL_texture
					(
					texturePath.c_str(),
					SOIL_LOAD_AUTO,
					SOIL_CREATE_NEW_ID,
					SOIL_FLAG_MULTIPLY_ALPHA
					);
			}
		}
	}

    
	return true; // Return Success
}

