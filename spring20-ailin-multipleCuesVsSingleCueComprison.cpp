// 8/3/2018 trying to solve the problem of the start
// 7/24/2018 1. add a prractice trial block (block 0) 2. add the spoken command 3. make it spin all the time 
//4. solve the moving when occluded problem 5. change the response key from "4" to "+"

// 7/19/2018
// This script got the 
// record the movement until response made or until hand back home?
// feature to add: add the indicator of "right" or "on target" in intermixed tasks
// feature to add: adding the audio "right" "on target" 
// feature to add: training for location - use dots cloud
// feature to add: a short break for introduction right before training
// feature to add: take break anytime
// feature to add: training again if mistakes

// as for 4/9/2018
// need to show which side is intermixed
// need to get the training grasp going

// add shades 2/5/2020
// To add shades, you'll need to define light_ambient, light_diffuse, light_direction and ojbect_color
// also enable a lot of lighting relevant functions/features? (currently in drawClyinder)
// also define normals for each vertex and creat the normal array (I put it in the buildCylinder together with the vertex array)
// also points to the array glNormalPointer(GL_FLOAT, 0, normals);

#include <cstdlib>
#include <cmath>
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
#include <cstdlib>
#include "SOIL.h"
#include <cstdio>
#include <ctime>
#include <math.h>
/********* BOOST MULTITHREADED LIBRARY ****************/
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>	//include asio in order to avoid the "winsock already declared problem"

#ifdef __APPLE__
#include <OpenGL/OpenGL.h>
#include <GLUT/glut.h>
#endif

#ifdef __linux__
#include <GL/glut.h>
#include <SOIL/SOIL.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <gl\gl.h>            // Header File For The OpenGL32 Library
#include <gl\glu.h>            // Header File For The GLu32 Library
#include "glut.h"            // Header File For The GLu32 Library
#include <MMSystem.h>


#endif
/********* INCLUDE CNCSVISION LIBRARY HEADERS **********/
//#include "Optotrak.h"
#include "Optotrak2.h"
#include "Marker.h"
#include "Mathcommon.h"
#include "GLUtils.h"
#include "VRCamera.h"
#include "CoordinatesExtractor.h"
#include "CylinderPointsStimulus.h"
#include "EllipsoidPointsStimulus.h"
#include "StimulusDrawer.h"
#include "GLText.h"
#include "BalanceFactor.h"
#include "ParStaircase.h"
#include "Staircase.h"
#include "ParametersLoader.h"
#include "TrialGenerator.h"
#include "Util.h"
#include "BrownMotorFunctions.h"
#include "BrownPhidgets.h"

/***** CALIBRATION FILE *****/
#include "LatestCalibration.h"

/***** DEFINE SIMULATION *****/
//#define SIMULATION
#ifndef SIMULATION
#include <direct.h> // mkdir
#endif

/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;
using namespace BrownPhidgets;

/********* #DEFINE DIRECTIVES **************************/
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
//static const double SCREEN_WIDE_SIZE = 383;//306;    // millimeters

static const Vector3d center(0, 0, focalDistance);
static double interoculardistance = 60;
Screen screen;
VRCamera cam;
Optotrak2 optotrak;
CoordinatesExtractor headEyeCoords, thumbCoords, indexCoords, thumbJointCoords, indexJointCoords;
Timer trial_timer;
GLUquadric* qobj;
const int axisZ = 1;
CPhidgetStepperHandle rotTable;
const float DEG2RAD = M_PI / 180;

/********* DISPLAY *******************/
static const bool gameMode = true;
static const bool stereo = true;

vector <Marker> markers;

Vector3d eyeLeft, eyeRight;
Vector3d ind,  thm;
Vector3d indexCalibrationPoint(0, 0, 0), thumbCalibrationPoint(0, 0, 0);

GLfloat fSizes[2];      // Line width range metrics
GLfloat fCurrSize;
GLfloat fCurrColor;
bool light = false;

//__int64 pos;


double mirrorAlignment = 0.0;
double screenAlignmentY = 0.0;
double screenAlignmentZ = 0.0;


/************************* MARKERS **************************************/
int ind1 = 13, ind2 = 14, ind3 = 16;
int thu1 = 15, thu2 = 17, thu3 = 18;
int calibration1 = 1, calibration2 = 2;
int screen1 = 19, screen2 = 20, screen3 = 21;
int mirror1 = 6, mirror2 = 22;
double markerXOffset = 10;
double markerYOffset = 10;

int centercalMarker = 4;

double target_angle = 0.0;
double objSizes[5] = { 30.0, 34.0, 38.0, 42.0 ,26.0 };
double objStepperPosns[5] = { 0, 90, 180, -90, -135 };

double indXNow = 0;
double indYNow = 0;
double indZNow = 0;

double thmXNow = 0;
double thmYNow = 0;
double thmZNow = 0;


/********** VISUAL STIMULUS  ***************/
// visual origin is the center of the visual stimulus (the mid point on the surface), 
// it should be set at a point that can get the edge of the elleptical surface to line up with the haptic stumulus edge
double display_distance = -380;
double lastVisualOriginX = 0, lastVisualOriginY = 0.0, lastVisualOriginZ = display_distance;
double visualOriginX = lastVisualOriginX, visualOriginY = 0.0, visualOriginZ = display_distance; // holders for the current target
double visual_angle = 6; // stim diangonal size
double min_depth = 22; // set in the subject file
//double max_depth = 40; // set in the subject file
double edge = 70;
double edge_apparent = edge;//tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance));
double cylinder_width = 1.2 * edge;
// stimulus depth
double depth = 20;

bool text_disp_conflict = true;

// estimated 2D probe
double estimated_depth = 0; // variable that subjects will populate with the 2D sine wave
double twoD_starting_depth = 0; // variable to note what the random starting 2D depth is
int texture_type = 0;
double probe_edge = 100; // height of 2D sine wave to include only 1 period of 3D sine wave
double rotation_bool = 0; //rotation boolean for the stimuli
double circle_radius = 1;
double probe_x_offset = 80;
double probe_y_offset = 70;

// For texture:
double normalizer_base = 70; // maximum distance along the side on the sine wave fluctuating in depth
double normalizer = normalizer_base; //will vary based on display distance
double minimum_distance = -270.;// closest distance sine wave could be presented
double maximum_distance = -470.;// furthest distance sine wave could be presented
double normalizer_scalefactor = (minimum_distance / minimum_distance); // scale the normalizer by distance to make sure dots are the same VA
int texnum = 2;

// panel size
double panel_w = 30;
double panel_h = 40;
double panel_separation = 45;


/********** VISUALIZATION AND STIMULI ***************/
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


/********* HAPTIC STIMULI *******************/
int num_depth = 5;
int objId = 0;

// the shift is the offset beteen the marker and the visual origin 
double targetMarker_origin_offsetX = -22.5;
double targetMarker_origin_offsetY = 0;
double targetMarker_origin_offsetZ = 5;

// target_thm is the grip point of the thumb
//double target_thm_X = visualOriginX ;
//double target_thm_Y = visualOriginY;
//double target_thm_Z = visualOriginZ;
double target_ind_Z = visualOriginZ - depth;


// centercal is a reference point for moving the motors
// Specifically, we will move the motors with moveObjectAbsolute() commands.
// First argument of moveObjectAbsolute is often called "moveTo"
// Second argument of moveObjectAbsolute is "centercal"
// moveObjectAbsolute will move the motors so that the final position is the difference between moveTo and centercal
// This has the effect of moving the "centercal" point to the "moveTo" point
// So set centercal to be the starting position (after homeEverything) of the point you want to control
// IN THIS EXPERIMENT: centercal is the start position of the middle of the bottom-edge of the phidgets linear apparatus
Vector3d centercal(0, 0, 0);
Vector3d moveTo(68, 0, -385);

// we have three positions: target_thm is the target thumb grip point;
// visualOrigin is the origin of the visual stimulus surface
// targetMarker is the position of the target marker (reference)
// targetMarkerX is set by the apparatus position so we can read it in at the begining, and it will determin the visualOriginX
// visualOriginY and visualOriginZ should be set by us

// target_thm = targetMarker + targetMarker_origin_offset
// visualOrigin = target_thm - edge_apparant/2

/***************** DISTANCE VARIABLES******************/
// The starting position
double startPosX;
double startPosY;
double startPosZ;
double x_dist_home;
double y_dist_home;
double z_dist_home;
double oldDistanceGripCenterToHome;
double distanceGripCenterToHome;
double vel_gripCntrToHome;
double start_dist;

// Position variables for keeping track of finger movement
double grip_Origin_X;
double grip_Origin_Y;
double grip_Origin_Z;
double grip_aperture = 0;
double old_grip_aperture;
double oldDistanceGripCenterToObject;
double distanceGripCenterToObject = 0;
double vel_gripCntrToObj;
double oldGrip_aperture;
double vel_gripAp;
double x_dist = 999;
double y_dist = 999;
double z_dist = 999;
double y_dist_thm = 999;
double z_dist_thm = 999;
double z_dist_ind = 999;

/********* VISIBILITY BOOLEANS ************************/
bool markers_status = false;
bool allVisibleIndex = markers_status;
bool allVisibleThumb = markers_status;
bool allVisibleFingers = markers_status;
//bool fingersOn = false; // Draw Fingers?
bool checkInfo = true;
bool feedbackInfo = false;
bool instructInfo = false;
bool handPositionInfo = false;
bool reachedObject = false;
bool allVisibleCalibration=false;
//bool allVisibleObject = false;

// TRIAL IDENTIFICATION
int blocks = 4;
int blockNum = 0;
//int taskNum = 1; // task==1: training for target location (MSE training) ; task==2: MSE; task==3: train for grasping time (grasp training); task==4: grasping trials
int trialNum = 1;
int attemptNum = 1;
int bin = 0;
int buffer_training_num = 0; // we take a break after the first training session, but we want to do an extra 6 training after this break before the actual experiment start
int repNum = 15;
bool training = true;

double frameNum = 0; // needs to be a double for division
double startTime;
double endTime;
double touchTime;
double drawTime;
double totalTrial = num_depth * 30.0; // 30 bins
/********* STATUS CHECK *********/
int fingerCalibrationDone = 0;
bool fingersCalibrated = false;
bool finished = false;

//bool training = true;
bool initTrainingDone = false;
//bool trialInitialized = false;
bool inBreak = false;
int percentComplete = 0;

// control the procedures
bool PositionSet;
bool goSignalPlayed;
bool returnHomePlayed;
bool IntervalOver = false;
bool responseCompleted = false;
int holdStillCountAtHome;
int holdStillCount;
int fbInfoHoldCount;
bool handAtHome = false;
bool handFarHome = false;
bool handNearObject = false;
bool handNearHome = false;
bool handFarObject = false;
bool handOnObject = false;
bool handTouchingObject = false;
int fingersOccluded = 0;
int framesOccluded = 0;
int startFrame = 0;
int endFrame = 0;
bool movementStarted = false;
bool touchedObject = false;
int goodTrain1_cumCnt = 0;
int goodTrain2_cumCnt = 0;// track consecutive success of training
int NumOfSuc_train = 3; // number of consecutive success before quit training
int failTrial_cumCnt = 0; // track consecutive failures of trials
int NumOfFail_trial = 3; // reaching this number will put trail back to training
bool lastAttempFail = true;
bool lastTrialFail = true;
//int failCause = 0; // failCause==0: success; failCause==1: press the key when fingers occluded; failCause==2: missed the target object;  failCause==3: moved too fast;  failCause==4: moved too slow; failCause==5: occluded movement; 
enum Stages { calibrationStage, preparingTrial, waitForHand, drawVisual, respond, feedback, breaktime, expFinished };
Stages currentState = calibrationStage;
enum TrialFeedback { success, occluded, missed, tooslow };
TrialFeedback failCause = success;
enum trialTypes { train_session, train_buffer, trial_grasp };
trialTypes currentTrial = train_session;


/********* LIGHTING ***************/
GLfloat LightAmbient[] = { 0.10f, 0.f, 0.f, 1.0f };
GLfloat LightDiffuse[] = { .6f, 0.0f, 0.0f, 1.0f };
GLfloat LightSpecular[] = { .8f, 0.8f, 0.8f, 0.0f };
//in the init trial now
GLfloat LightPosition[] = {0.0f, 80.0f, 80.0f, 1.0f};
//GLfloat LightPosition[] = { 0.0f, 0.0f, 0.0f, 1.0f };
GLfloat specularMaterial[] = { .25f, 0.25, 0.25, 1.0 };
GLfloat shininessMaterial = 40.0f;/*GLfloat LightPosition[] = {0.0f, 0.0f, 0.0f, 1.0f};
GLfloat LightDiffuse[] = {.8f, 0.0f, 0.0f, 1.0f};
float lightZ = -200.0;
float lightAmb = 0.70;*/


/********** SET THE LIGHT ***********/
// setting up the light
/*
GLfloat light_ambient[] = {0.2f, 0.2f, 0.2f, 1.f};
GLfloat light_diffuse[] = {1.f, 1.f, 1.f, 1.f};
GLfloat light_dir[] = {0.f, 0.4f, 0.8f, 0.f};
GLfloat cylinder_ambient[] = {1.f, 0.f, 0.f, 1.f};
GLfloat cylinder_color[] = {1.f, 0.f, 0.f, 1.f};
*/

/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void cleanup();
void drawInfo();
void drawStimulus(Stages stage);
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d& _transformation = Affine3d::Identity(), bool synchronous = true);
void update(int value);
void idle();
void initMotors();
void initStreams();
void initOptotrak();
void initRendering();
void advanceTrial();
void initTrial();
void updateTheMarkers();
void online_apparatus_alignment();
void online_fingers();
void online_trial();
void calibration_fingers(int phase);
void drawFingers(double offset);
void drawTrainObject();
void closingTrialFile();
void drawX();
void drawT();
void drawCross();
void drawArrow();
//void drawGripCenter();
//void drawGuidanceBar();
void drawThumb();
void drawProgressBar();
void buildCylinder(double textureDepth, double dispDepth);
void drawCylinder();
void drawAperture();
void initVariables();
int LoadGLTextures();
double calculateDepth(double depth, double y);
void lighting();


/*************************** EXPERIMENT SPECS ****************************/
// experiment directory
string experiment_directory = "C:/Users/visionlab/Documents/data/ailin/fall19-ailin-cueCompGraspAdaptationGraspCarousel/";

// response file headers
string intermixedFile_headers = "subjName\tblockN\tbin\ttrialN\tattemptN\tdepth\ttextureType\ttextNum\tframeN\ttime\tstartTime\tdrawTime\ttouchTime\tendTime\tfingersOccluded\tdistanceToHome\tdistanceToObject\tindexXraw\tindexYraw\tindexZraw\tthumbXraw\tthumbYraw\tthumbZraw\ty_dist_thm\tz_dist_thm\tz_dist_ind\tfailCause\tGA\tvelGripAp\tvelGripCnt\tframesOccluded";
string subjectName;

/********* FILE STREAMS AND PARAMETERS**************************/
//ofstream trialFile;
ParametersLoader parameters;
BalanceFactor<double> trial;

/*************************** FUNCTIONS ***********************************/

/***** SOUNDS *****/
void beepOk(int tone)
{
	switch (tone)
	{
	case 0:
		// Remember to put double slash \\ to specify directories!!!
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-1.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 1:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-6.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 2:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-faster.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 3:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-slower.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 4:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-missed.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 10:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-grasp.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 24:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-look.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 15:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-reject.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 20:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-pinch.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 30:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-440-pluck.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 31:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\beep-lowBubblePop.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 40:
		PlaySound((LPCSTR) "C:\\cygwin\\home\\visionlab\\workspace\\cncsvision\\data\\beep\\spoken-virtual.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	}
	return;
}


void drawProgressBar() {
	
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glColor3f(0.7, 0.7, 0.7);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-50, 5, 0);
	glVertex3f(50, 5, 0);
	glVertex3f(50, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();

	glColor3f(0.1, 0.5, 0.1);
	glBegin(GL_POLYGON);
	glVertex3f(-50, 5, 0);
	glVertex3f(-50 + percentComplete, 5, 0);
	glVertex3f(-50 + percentComplete, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();
}



// First, make sure the filenames in here are correct and that the folders exist. 
// If you mess this up, data may not be recorded!
void initStreams()
{
	string parametersFile_directory_intermixed;

	// parameters file directory and name
	parametersFile_directory_intermixed = experiment_directory + "parameters_fall19-ailin-fall19-ailin-cueCompGraspAdaptationGraspCarousel.txt";

	ifstream parametersFile_intermixed;
	parametersFile_intermixed.open(parametersFile_directory_intermixed.c_str());
	parameters.loadParameterFile(parametersFile_intermixed);

	subjectName = parameters.find("SubjectName");


	// trialFile directory
	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax

	// Throw an error if the subject name is already used. Don't want to overwrite good data!
	if (util::fileExists(dirName + "/" + subjectName + "_blk1_trial1.txt")) // when writing distractor task data to file, use this style of filename
	{
		string error_on_file_io = subjectName + string(" already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
		exit(0);
	}

	//globalTimer.start();
}

void initVariables()
{
	// initialize the trial matrix
	trial.init(parameters);
	trial.next();
	interoculardistance = str2num<double>(parameters.find("IOD"));
	// eye coordinates
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);
	//eyeMiddle = Vector3d(0, 0, 0);
	display_distance = str2num<double>(parameters.find("dispDepth"));
	edge_apparent = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));
	probe_edge = edge_apparent;
	visualOriginZ = display_distance;
	
}


void drawInfo()
{

	GLText text;
	if (gameMode)
		text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_12);
	else
		text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);

	text.enterTextInputMode();

	switch (currentState) {

	case expFinished:
	{
		glColor3fv(glWhite);
		text.draw("The experiment is over. Thank you! :)");
	}
	break;

	case breaktime:
	{
		glColor3fv(glRed);
		text.draw("Break Time ... Press A to continue");
	}
	break;

	case feedback:
	{
		glColor3fv(glWhite);
		switch (failCause) {
		case tooslow:
			text.draw("TOO SLOW");
			break;

		case occluded:
			text.draw("OCCLUDED");
			break;

		case missed:
			text.draw("MISSED");
			break;
		}

	}
	break;

	case calibrationStage:
	{
		switch (fingerCalibrationDone) {

				case 0:
					text.draw("Press F to record calibration points on metal bar.");
					break;
				case 1:
					text.draw("Press F with both fingertips on calibration points.");
					break;
				case 2:
					text.draw("Press F to move the object into place.");
					break;
				case 3:
					text.draw("Take off the marker!!! Press F with hand at start position to begin!");
					break;
		}
		// the following is drawn depending on which calibration stage it is
		if (fingerCalibrationDone <= 2) {

			if (isVisible(markers[calibration1].p) && isVisible(markers[calibration1].p) && isVisible(markers[calibration1].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Calibration Point " + stringify< Eigen::Matrix<double, 1, 3> >(markers[calibration1].p.transpose()));

			// INDEX FINGER ///////
			glColor3fv(glWhite);
			text.draw(" ");
			text.draw("Index");
			if (isVisible(markers[13].p) && isVisible(markers[14].p) && isVisible(markers[16].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Marker " + stringify<int>(13) + stringify< Eigen::Matrix<double, 1, 3> >(markers[13].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(14) + stringify< Eigen::Matrix<double, 1, 3> >(markers[14].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(16) + stringify< Eigen::Matrix<double, 1, 3> >(markers[16].p.transpose()) + " [mm]");

			// THUMB //////
			glColor3fv(glWhite);
			text.draw(" ");
			text.draw("Thumb");
			if (isVisible(markers[15].p) && isVisible(markers[17].p) && isVisible(markers[18].p))
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Marker " + stringify<int>(15) + stringify< Eigen::Matrix<double, 1, 3> >(markers[15].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(17) + stringify< Eigen::Matrix<double, 1, 3> >(markers[17].p.transpose()) + " [mm]");
			text.draw("Marker " + stringify<int>(18) + stringify< Eigen::Matrix<double, 1, 3> >(markers[18].p.transpose()) + " [mm]");
		}

		// Index and Thumb Positions ////////
		if (fingerCalibrationDone > 2) {
			glColor3fv(glWhite);
			text.draw("--------------------");
			if (allVisibleIndex)
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Index= " + stringify< Eigen::Matrix<double, 1, 3> >(ind.transpose()));
			if (allVisibleThumb)
				glColor3fv(glGreen);
			else
				glColor3fv(glRed);
			text.draw("Thumb= " + stringify< Eigen::Matrix<double, 1, 3> >(thm.transpose()));
			glColor3fv(glWhite);
			text.draw("--------------------");
		}
	}

	break;
	}


	if (checkInfo) {

		// the following would be drawn but not shown when visibleInfo = false
		text.draw("#");
		text.draw("#");
		text.draw("#");
		text.draw("#");
		text.draw("####### SUBJECT #######");
		text.draw("#");
		text.draw("# Name: " + subjectName);
		text.draw("# IOD: " + stringify<double>(interoculardistance));
		text.draw("# haptic depth: " + stringify<double>(depth));
		text.draw("centercal marker " + stringify< Eigen::Matrix<double, 1, 3> >(markers[4].p.transpose()));
		text.draw("# stage: " + stringify<int>(currentState));
		text.draw("# time: " + stringify<double>(trial_timer.getElapsedTimeInMilliSec()));
		
		

		// check if mirror is calibrated
		if (abs(mirrorAlignment - 45.0) < 0.2)
			glColor3fv(glGreen);
		else
			glColor3fv(glRed);
		text.draw("# Mirror Alignment = " + stringify<double>(mirrorAlignment));

		// check if monitor is calibrated
		if (screenAlignmentY < 89.0)
			glColor3fv(glRed);
		else
			glColor3fv(glGreen);
		text.draw("# Screen Alignment Y = " + stringify<double>(screenAlignmentY));
		if (abs(screenAlignmentZ) < 89.0)
			glColor3fv(glRed);
		else
			glColor3fv(glGreen);
		text.draw("# Screen Alignment Z = " + stringify<double>(screenAlignmentZ));

		glColor3fv(glWhite);
		// if IOD has been input

		glColor3fv(glWhite);
		text.draw("# Trial: " + stringify<int>(trialNum));
		text.draw("# Hold Still Count: " + stringify<int>(holdStillCount));
		text.draw("# Home Hold Still Count: " + stringify<int>(holdStillCountAtHome));

		glColor3fv(glWhite);
		text.draw("#######################");
		text.draw("Calibration Step= " + stringify<int>(fingerCalibrationDone));

		// the following will be always be drawn
		glColor3fv(glWhite);
		//if (handNearObject)
		//	text.draw("hand near object");
		text.draw("# distance to Ojbect: " + stringify<double>(distanceGripCenterToObject));
		//			text.draw("# progress: " + stringify<int>(percentComplete));
		text.draw("# current state: " + stringify<int>(currentState));
		text.draw("# thm to target y: " + stringify<double>(y_dist_thm));
		text.draw("# thm to target z: " + stringify<double>(z_dist_thm));
		text.draw("# index to target z: " + stringify<double>(z_dist_ind));
		text.draw("# visual origin X: " + stringify<double>(visualOriginX));
		text.draw("# visual origin Y: " + stringify<double>(visualOriginY));
		text.draw("# visual origin Z: " + stringify<double>(visualOriginZ));
		text.draw("# texture: " + stringify<int>(texture_type));
		text.draw("# time: " + stringify<double>(trial_timer.getElapsedTimeInMilliSec()));
		

	}


	text.leaveTextInputMode();

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
	cylinder_width = 1.5 * edge;
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
			normals[vertex_index] = y; // this is y
			vertex_index++;
			vertices[vertex_index] = z; // this is z
			colors[vertex_index] = 0; // B is this value
			normals[vertex_index] = z; // this is z
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


void drawAperture() {
	//double panel_separation = tan((DEG2RAD * visual_angle)/2) * 2 * (abs(display_distance) - max_depth); 
	//panel_separation = edge * (display_distance + max_depth - depth)/(display_distance - depth);
	panel_separation = edge_apparent;

	//glDisable(GL_TEXTURE_2D);
	glLoadIdentity();
	glTranslated(visualOriginX, visualOriginY, visualOriginZ );
	//glTranslated(0, 0, display_distance);
	if (rotation_bool == 1) {
		glRotatef(90.0, 0.0, 0.0, 1.0); // 0, 0, 1
	}

		//glColor3f(0.6f, 0.0f, 0.0f);
	glColor3f(0.0f, 0.0f, 0.0f);
	glBegin(GL_QUADS);
	glVertex3f(-panel_separation / 2 - panel_w, panel_h, 0.0f);
	glVertex3f(-panel_separation / 2, panel_h, 0.0f);
	glVertex3f(-panel_separation / 2, -panel_h, 0.0f);
	glVertex3f(-panel_separation / 2 - panel_w, -panel_h, 0.0f);
	glEnd();

	glBegin(GL_QUADS);
	glVertex3f(panel_separation / 2, panel_h, 0.0f);
	glVertex3f(panel_separation / 2 + panel_w, panel_h, 0.0f);
	glVertex3f(panel_separation / 2 + panel_w, -panel_h, 0.0f);
	glVertex3f(panel_separation / 2, -panel_h, 0.0f);
	glEnd();
}

void drawCylinder() {
/*
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	GLfloat lmodel_ambient[] = {lightAmb, lightAmb, lightAmb, 1.0};
	
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient); 
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);

*/

	/*
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, light_dir);	
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, cylinder_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, cylinder_color);
	
	glEnable(GL_LIGHT1);
	//glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces
	//glEnable(GL_COLOR_MATERIAL);
	//glEnable(GL_BLEND);
	//glEnable(GL_LIGHTING);
*/
	// define our transformation matrix to push back in depth, and do it
	

	// rotation

	//if (rotation_bool == 1) {

	//}

	glLoadIdentity();
	glTranslated(visualOriginX, visualOriginY, visualOriginZ - depth );
	//glTranslated(0, 0, display_distance - depth);

	glMaterialfv(GL_FRONT, GL_SPECULAR, specularMaterial);
	glMaterialf(GL_FRONT, GL_SHININESS, shininessMaterial);
	glMaterialfv(GL_BACK, GL_SPECULAR, specularMaterial);
	glMaterialf(GL_BACK, GL_SHININESS, shininessMaterial);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);

	if (rotation_bool == 1) {
		glRotatef(90.0, 0.0, 0.0, 1.0); // 0, 0, 1
	}
	// enable matrices for use in drawing below
	glEnable(GL_LIGHTING);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_TEXTURE_2D);
	
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces
	//glColorPointer(3, GL_FLOAT, 0, colors);
	// glColor3f(1.0f, 0.0f, 0.0f);

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

void initTrial() {
	// ojbect size
	// file name
	// new file
	// lastFailure
	// set variables
	// set carousel
	//trialInitialized = false;
	
	//texture_type = 3; // texture type from the subject parameters file
		bin = ceil( double(trialNum) / double(num_depth) );
		objId = trial.getCurrent()["objectID"];
		texnum = (rand() % 50) + 1;
		if(bin <= 5 || bin > 25){
			texture_type = 3;
		}else{
			texture_type = 0;
		}

	//target_jitterZ = - rand() % 16; 
	//moveTo[2] = target_thm_Z - targetMarker_origin_offsetZ + target_jitterZ;
	//moveObjectAbsolute(moveTo, centercal, 6200);		
	depth = objSizes[objId];
	target_angle = objStepperPosns[objId];
	
	buildCylinder(depth, depth);

	//if(handFarObject && allVisibleIndex){
	//stepper_rotate(rotTable, target_angle, 238.67);

	target_ind_Z = visualOriginZ - depth;

	frameNum = 0;
	startTime = 0;
	endTime = 0;
	startFrame = 0;
	touchTime = 0;
	drawTime = 0;
	endFrame = 0;
	responseCompleted = false;
	holdStillCountAtHome = 0;
	holdStillCount = 0;
	movementStarted = false;
	instructInfo = true;
	framesOccluded = 0;
	reachedObject = false;
	touchedObject = false;
	// roll on

	initProjectionScreen(display_distance);
	//get the position of the backSurface
	trial_timer.reset();
	trial_timer.stop();
	currentState = drawVisual;

}

void drawGLScene()
{
	online_apparatus_alignment();
	online_fingers();
	online_trial();

	glDrawBuffer(GL_BACK);

	// Draw left eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeLeft);
	drawStimulus(currentState);
	drawInfo();

	// Draw right eye view
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	cam.setEye(eyeRight);
	drawStimulus(currentState);
	drawInfo();

	glutSwapBuffers();
}


void drawStimulus(Stages stage)//drawing fingers and stiumulus
{//Stages {calibrationStage, preparingTrial, waitForHand, respond, feedback, breaktime, expFinished };

	//stepperRotate(depth);


	switch (stage) {
		//case waitForHand:

	case drawVisual:
			drawCylinder();
			//drawAperture();
		break;


	}

}



void drawX() {
	glPushMatrix();
	glLoadIdentity();
	glTranslated(visualOriginX, visualOriginY+60, visualOriginZ);
	glColor3fv(glRed);
	glLineWidth(3.0);
	//slash 1
	glBegin(GL_LINE_LOOP);
	glVertex3d(-10, -10, 0);
	glVertex3d(10, 10, 0);
	glEnd();
	//slash 2
	glBegin(GL_LINE_LOOP);
	glVertex3d(-10, 10, 0);
	glVertex3d(10, -10, 0);
	glEnd();
	glPopMatrix();
	glLineWidth(1.0);
}

void drawCross() {
	glPushMatrix();
	glLoadIdentity();
	//glTranslated(visualOriginX, visualOriginY, visualOriginZ);
	glTranslated(visualOriginX, 0, visualOriginZ );
	glLineWidth(3.0);
	//slash 1
	glBegin(GL_LINE_LOOP);
	glVertex3d(-10, 0, 0);
	glVertex3d(10, 0, 0);
	glEnd();
	//slash 2
	glBegin(GL_LINE_LOOP);
	glVertex3d(0, -10, 0);
	glVertex3d(0, 10, 0);
	if (currentTrial == train_session && handOnObject)
		glColor3f(0.0f, 1.0f, 0.0f); //green
	else
		glColor3f(1.0f, 0.0f, 0.0f); //red
	glEnd();
	glPopMatrix();
	glLineWidth(1.0);
}

void drawT() {
	glPushMatrix();
	glLoadIdentity();
	glTranslated(visualOriginX, visualOriginY+50, visualOriginZ);
	glColor3fv(glRed);
	glLineWidth(3.0);
	//slash 1
	glBegin(GL_LINE_LOOP);
	glVertex3d(-10, 10, 0);
	glVertex3d(10, 10, 0);
	glEnd();
	//slash 2
	glBegin(GL_LINE_LOOP);
	glVertex3d(0, 10, 0);
	glVertex3d(0, -10, 0);
	glEnd();
	glPopMatrix();
	glLineWidth(1.0);
}





void drawThumb() {
		// Thumb Sphere
		glColor3f(0.8f,0.3f,0.3f);
		glPushMatrix();
		glLoadIdentity();
		GLUquadricObj* thumbSphere = gluNewQuadric();
		gluQuadricDrawStyle(thumbSphere, GLU_FILL);
		glTranslated(thmXNow,thmYNow,thmZNow);
		gluSphere(thumbSphere, 2, 6, 6);
		gluDeleteQuadric(thumbSphere);
		glPopMatrix();
}




void advanceTrial()
{
	if (training){
		//closingTrialFile();
		if (currentTrial == train_buffer){
			buffer_training_num++;
			if (buffer_training_num == 13) {
				training = false;
				trial.init(parameters);
				currentTrial = trial_grasp;
			    blockNum = 1;
				trialNum = 1;
				attemptNum = 0;
			}
		}
		attemptNum++;
		trial.next();
		initTrial();
	}else{

		if(failCause == success){

			if (!trial.isEmpty()) { // if  there are more trials in this block
				//closingTrialFile();
				trialNum++;
				attemptNum = 1;
				trial.next(); // pull the next trial from the "deck"
				initTrial();
			} else {
				beepOk(0);
				//closingTrialFile();
				currentState = expFinished;
			}

		} else {
			//closingTrialFile();
			attemptNum++;
			initTrial();
		}
	}

}

void handleKeypress(unsigned char key, int x, int y)
{
	switch (key)
	{

	case 'a':
	case 'A':
		{  
		trial.next();
		initTrial();
		}
		break;

	case 't':
	case 'T':
		{   
		currentState = breaktime;
		closingTrialFile();
		//blockNum = 1;
		//training = false;
		currentTrial = train_buffer;
		}
		break;


	case 'i':
		checkInfo = !checkInfo;
		break;

	case 'm':
		{
			interoculardistance += 0.5;
			headEyeCoords.setInterOcularDistance(interoculardistance);
		}
		break;
	case 'n':
		{
			interoculardistance -= 0.5;
			headEyeCoords.setInterOcularDistance(interoculardistance);
		}
		break;


	case 27:	//corrisponde al tasto ESC
		{

			stepper_rotate(rotTable, 0, 238.67);
			//homeEverything(5000, 4500);
			stepper_close(rotTable);

			//homeEverything(5000, 4500);

			cleanup();
			exit(0);
		}
		break;


	case 'f':
	case 'F':
	
				beepOk(0);
				fingersCalibrated = true;
				checkInfo = false;
				fingerCalibrationDone=4;
				visibleInfo = false;
				
			
				blockNum = 0;
				trialNum = 0;
				attemptNum = 1;
				training = false;
				//initTrainingDone = false;
				currentTrial = trial_grasp;
				currentState = preparingTrial;

				initTrial();

			
		
		break;

	}
}

void calibration_fingers(int phase)
{
	switch (phase)
	{
	case 1:
		{
			if(isVisible(markers[calibration1].p))
			{
				// Specific to the one-marker calibration procedure on the metal posts
				indexCalibrationPoint=markers.at(calibration1).p;
				indexCalibrationPoint[0] = indexCalibrationPoint[0] + 3;
				indexCalibrationPoint[1] = indexCalibrationPoint[1] - 13;
				indexCalibrationPoint[2] = indexCalibrationPoint[2] - 10;
				thumbCalibrationPoint=markers.at(calibration1).p;
				thumbCalibrationPoint[0] = thumbCalibrationPoint[0] + 3;
				thumbCalibrationPoint[1] = thumbCalibrationPoint[1] - 13;
				thumbCalibrationPoint[2] = thumbCalibrationPoint[2] + 10;
			}
		} break;

	case 2:
		{
			indexCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p );
			thumbCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p );
		} break;

		//case 3:
		//{
		//	indexJointCoords.init(indexCalibrationPoint, markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p );
		//	thumbJointCoords.init(thumbCalibrationPoint, markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p );
		//} break;

	}

}


// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}

void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_HIGH_SIZE * SCREEN_WIDTH / SCREEN_HEIGHT, SCREEN_HIGH_SIZE);//(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE*SCREEN_HEIGHT/SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 4500);
}

void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

void closingTrialFile()
{

}

/*
string graspFile_headers = "subjName\tblockN\ttaskN\ttrialN\tattemptN\tvisualHeight\thapticHeight\tframeN\ttime\tstartTime\tendTime\tfingersOccluded\tdistanceToHome\tdistanceToObject\tindexXraw\tindexYraw\tindexZraw\tthumbXraw\tthumbYraw\tthumbZraw\ttargetX\ttargetY\ttargetZ\tfailCause";*/
void idle()
{

	// get new marker positions from optotrak
	updateTheMarkers();

	// eye coordinates
	eyeRight = Vector3d(interoculardistance / 2, 0, 0);//0
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);//0


}


void updateTheMarkers()
{
	// this gets called every frame in idle()
	optotrak.updateMarkers();
	markers = optotrak.getAllMarkers();
}

/*** Online operations ***/
void online_apparatus_alignment()
{

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

void online_fingers()
{
	// Visibility check
	if (fingersCalibrated) {
		allVisibleIndex = isVisible(markers.at(ind1).p) && isVisible(markers.at(ind2).p) && isVisible(markers.at(ind3).p);
		allVisibleThumb = isVisible(markers.at(thu1).p) && isVisible(markers.at(thu2).p) && isVisible(markers.at(thu3).p);

		if (fingerCalibrationDone < 3)
			allVisibleCalibration = isVisible(markers.at(calibration1).p);// && isVisible(markers.at(calibration2).p);

		allVisibleFingers = allVisibleIndex && allVisibleThumb;

		if (fingerCalibrationDone >= 2)
		{
			// index coordinates
			if (allVisibleIndex) {
				indexCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p);
				//indexJointCoords.update(markers.at(ind1).p, markers.at(ind2).p, markers.at(ind3).p );
				ind = indexCoords.getP1();
				//if(fingerCalibrationDone>=3)
				//indJoint = indexJointCoords.getP1();
			}
			// thumb coordinates
			if (allVisibleThumb) {
				thumbCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p);
				//thumbJointCoords.update(markers.at(thu1).p, markers.at(thu2).p, markers.at(thu3).p );
				thm = thumbCoords.getP1();
				//if(fingerCalibrationDone>=3)
				//thmJoint = thumbJointCoords.getP1();
			}
		}
		if (!inBreak) {
			if (!allVisibleFingers)
			{
				fingersOccluded = 1;
			}
			else {
				fingersOccluded = 0;
			}

			indXNow = ind.x();
			indYNow = ind.y();
			indZNow = ind.z();

			thmXNow = thm.x();
			thmYNow = thm.y();
			thmZNow = thm.z();

			grip_Origin_X = (indXNow + thmXNow) / 2;
			grip_Origin_Y = (indYNow + thmYNow) / 2;
			grip_Origin_Z = (indZNow + thmZNow) / 2;
			oldGrip_aperture = grip_aperture;
			// compute grip aperture
			grip_aperture = sqrt(
				(indXNow - thmXNow) * (indXNow - thmXNow) +
				(indYNow - thmYNow) * (indYNow - thmYNow) +
				(indZNow - thmZNow) * (indZNow - thmZNow)
			);

			vel_gripAp = abs(oldGrip_aperture - grip_aperture);

			// find distance to Home 
			x_dist_home = abs(grip_Origin_X - startPosX);
			y_dist_home = abs(grip_Origin_Y - startPosY);
			z_dist_home = abs(grip_Origin_Z - startPosZ);
			oldDistanceGripCenterToHome = distanceGripCenterToHome;
			distanceGripCenterToHome = sqrt((x_dist_home * x_dist_home) + (y_dist_home * y_dist_home) + (z_dist_home * z_dist_home));
			vel_gripCntrToHome = abs(oldDistanceGripCenterToHome - distanceGripCenterToHome);
			handAtHome = (distanceGripCenterToHome < 40);
			handFarHome = distanceGripCenterToHome > 300;
			handNearHome = distanceGripCenterToHome < 100;



			// find distance from grip center to object center
			x_dist = abs(grip_Origin_X - visualOriginX);
			y_dist = abs(grip_Origin_Y - visualOriginY);
			z_dist = abs(grip_Origin_Z - visualOriginZ);
			oldDistanceGripCenterToObject = distanceGripCenterToObject;
			distanceGripCenterToObject = sqrt((x_dist * x_dist) + (y_dist * y_dist) + (z_dist * z_dist));
			//if (PositionSet) {
			vel_gripCntrToObj = abs(oldDistanceGripCenterToObject - distanceGripCenterToObject);
			//}

				// find distance from thumb to front curve

			y_dist_thm = abs(thmYNow - visualOriginY);
			z_dist_thm = abs(thmZNow - visualOriginZ);
			z_dist_ind = abs(indZNow - visualOriginZ - depth);

			//target_ind_Z
			handOnObject = ((y_dist_thm < 17) && (z_dist_thm < 17) && (z_dist_ind < 17));
			handNearObject = ((y_dist_thm < 20) && (z_dist_thm < 20));
			handFarObject = distanceGripCenterToObject >= 250;
			handTouchingObject = ((y_dist_thm < 15) && (z_dist_thm < 15));;
		}
	}
	else {
		allVisibleFingers = isVisible(markers.at(centercalMarker).p);
	}
}




/* 1. trialInitialized.
2. If hand stay home -> trialInProgress
3. If detect movement started-> movmentStarted.
4. If detect response made -> responseCompleted; Decide failCause.
5. If gave feedback -> trialInProgressed = false;
6. If failed, audio + feedbackInfo + wait -> IntervalOver. Advancetrial
*/

void online_trial() {
	//enum Stages { calibrationStage, preparingTrial, waitForHand, respond, feedback, breaktime, expFinished };

	switch (currentState) {

	case waitForHand:
		frameNum++;
		currentState = drawVisual;

		break;

	case drawVisual:
		frameNum++;
		if (handAtHome && grip_aperture < 40 && holdStillCountAtHome < 60){
			holdStillCountAtHome++;
		}
		if (holdStillCountAtHome == 60) {
			holdStillCountAtHome = 61;
			currentState = respond;
			beepOk(10); // say grasp
		}
		break;


	}

	////////////////////////////////////////////////

}

void lighting() {
	// Set up the lighting
	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, LightSpecular);
	glLightf(GL_LIGHT1, GL_CONSTANT_ATTENUATION, 0.5f);

	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_LIGHT1);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

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

void initMotors()
{
	// put the monitor and the arm back at the start
	homeEverything(4051, 3500);
}

void initOptotrak()
{
	optotrak.setTranslation(calibration);

	if (optotrak.init(LastAlignedFile, OPTO_NUM_MARKERS, OPTO_FRAMERATE, OPTO_MARKER_FREQ, OPTO_DUTY_CYCLE, OPTO_VOLTAGE) != 0)
	{
		cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
		cin.ignore(1E6, '\n');
		exit(0);
	}

	// Read 10 frames of coordinates and fill the markers vector
	for (int i = 0; i < 10; i++)
	{
		updateTheMarkers();
	}
}

void cleanup()
{
	// Stop the optotrak when finished
	optotrak.stopCollection();
}

// stepper with linear actuator head - 0 = 24 mm, 4194304 = 89 mmn

int main(int argc, char* argv[])
{
	mathcommon::randomizeStart();

	// initializes optotrak and velmex motors
	initOptotrak();
	//initMotors();

	rotTable = stepper_connect();
	cerr << 'bby1' << endl;
	//homeEverything(6000, 4000);

	// initializing glut
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
	glutGameModeString("1024x768:32@85");
	glutEnterGameMode();
	//glutFullScreen();

	// initializing experiment's parameters
	initRendering();
	initStreams();
	cerr << 'bby2' << endl;
	LoadGLTextures();

	// glut callback
	glutDisplayFunc(drawGLScene);
	glutKeyboardFunc(handleKeypress);
	glutReshapeFunc(handleResize);
	glutIdleFunc(idle);
	glutTimerFunc(TIMER_MS, update, 0);
	glutSetCursor(GLUT_CURSOR_NONE);

	boost::thread initVariablesThread(&initVariables);

	// Application main loop
	glutMainLoop();
	stepper_rotate(rotTable, 0, 238.67);
	stepper_close(rotTable);
	//homeEverything(6000, 4000);
	cleanup();

	return 0;

}
