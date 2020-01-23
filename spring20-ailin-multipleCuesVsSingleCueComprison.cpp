// This file is for Ailin to learn drawing four rods
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
//#include "Optotrak.h"
//#include "Optotrak2.h"
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
#include "ParametersLoader.h"
#include "Util.h"
#define BROWN 
#ifdef BROWN
#include "BrownMotorFunctions.h"
#else
#include "RoveretoMotorFunctions.h"
#endif
/********* NAMESPACE DIRECTIVES ************************/
using namespace std;
using namespace mathcommon;
using namespace Eigen;
using namespace util;
using namespace BrownMotorFunctions;

/********* #DEFINE DIRECTIVES **************************/
#define TIMER_MS 11 // 85 hz
#define SCREEN_WIDTH  1024      // pixels
#define SCREEN_HEIGHT 768       // pixels
static const double SCREEN_WIDE_SIZE = 306;    // millimeters

/********* 18 October 2011   CALIBRATION ON CHIN REST *****/
static const Vector3d calibration(160,179,-75);
//static const Vector3d objCalibration(199.1, -149.2, -319.6);
// Alignment between optotrak z axis and screen z axis
double alignmentX =  33.5;
double alignmentY =  33;
double focalDistance= -270.0, homeFocalDistance=-270.0;
static const Vector3d center(0,0,focalDistance);
static const Vector3d centercal(-21.8,-327.2,-330.0);
Screen screen;
/********* VISUALIZATION VARIABLES *****************/
static const bool gameMode=true;
static const bool stereo=true;

/********* VARIABLES OBJECTS  **********************/
VRCamera cam;
//Optotrak2 *optotrak;
CoordinatesExtractor headEyeCoords;
/********** VISUALIZATION AND STIMULI ***************/
StimulusDrawer stimDrawer[9];
CylinderPointsStimulus cylinder[9];
StimulusDrawer ellipsDrawer;
EllipsoidPointsStimulus ellipsoid;
bool isStimulusDrawn=true;
bool isSquareDrawn=false;
int leftButtonDown = 0, rightButtonDown = 0;
// square measures
double edge = 0.0, dedge = 0, jitterX = 0.0, xedge = 0.0, zedge = 0.0, jitter = 0.0, theta=0, phi=M_PI*3/4, dz = 0.0, dx = 0.0, r = 0.0, fdx = 1.0, instant = 0.0, dy = 10.0, EBheight = 30.0;
Timer timer;
Timer globalTimer;

/********** EYES AND MARKERS **********************/
Vector3d eyeLeft, eyeRight;
vector <Marker> markers;
static double interoculardistance=60;

/********* CALIBRATION VARIABLES *********/
int headCalibrationDone=0;
int platformCalibrationDone=0;
bool allVisiblePatch=false;
bool visibleInfo=true;

/********* TRIAL VARIABLES *********/
int trialNumber = 0;
ParametersLoader parameters;
BalanceFactor<double> trial;
//map<string,double> factors;
static const Vector3d objLocation;
double depth =30.0;
double sepz = 40.0;
/********** STREAMS **************/
ofstream responseFile, markersFile, timeFile;


/********** FUNCTION PROTOTYPES *****/
void beepOk(int tone);
void drawCircle(double radius, double x, double y, double z);

void drawCalibration();
void drawSquare();
void drawFourRods();
void drawTrial();
void drawGLScene();
void handleKeypress(unsigned char key, int x, int y);
void handleResize(int w, int h);
void initProjectionScreen(double _focalDist, const Affine3d &_transformation=Affine3d::Identity(),bool synchronous=true);
void update(int value);
void idle();
void initMotors();
void initGLVariables();
void initVariables();
void initStreams();
//void initOptotrak();
void initRendering();
void advanceTrial();
void initTrial();
void positionObj(Vector3d pos);
void drawFourRods(double deltaz);

/***** SOUND THINGS *****/
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
	}
	return;
}

/*************************** FUNCTIONS ***********************************/


void initGLVariables()
{
	for (int i=0; i<4; i++)
	{
		cylinder[i].setNpoints(200);
		cylinder[i].setRadiusAndHeight(3,39); // raggio (mm) altezza (mm)
		// Dispone i punti random sulla superficie cilindrica 
		cylinder[i].compute();
		stimDrawer[i].setStimulus(&cylinder[i]);
		// seguire questo ordine altrimenti setspheres non ha effetto se chiamata dopo StimulusDrawer::initList
		stimDrawer[i].setSpheres(true);
		stimDrawer[i].initList(&cylinder[i], glRed);
	}
}

void drawFourRods()
{
	if ( isStimulusDrawn )
	{
	glLoadIdentity();

	// ###### draw rotated stimulus (adjusted) ######
	glTranslated(0.0,0,trial.getCurrent()["AbsDepth"]);
	glRotated(trial.getCurrent()["Orientation"]*(str2num<double>(parameters.find("ThetaSign")))+theta,0,1,0);

	// Front Right rod on focal plane
	glPushMatrix();
	glTranslated((trial.getCurrent()["RelDepthObj"]/2)+dx, -dy, dz/2);
	stimDrawer[0].draw();
	glPopMatrix();

	// Front Left rod on focal plane
	glPushMatrix();
	glTranslated(-(trial.getCurrent()["RelDepthObj"]/2)+dx, -dy, dz/2);
	stimDrawer[1].draw();
	glPopMatrix();

	// Rear right rod shifted 2 cm behind the focal plane
	glPushMatrix();

	glTranslated( (trial.getCurrent()["RelDepthObj"]/2) , dy, - dz/2 );
	stimDrawer[2].draw();
	glPopMatrix();

	// Rear left rod shifted 2 cm behind the focal plane
	glPushMatrix();
	glTranslated( -(trial.getCurrent()["RelDepthObj"]/2) , dy, - dz/2 );
	stimDrawer[3].draw();
	glPopMatrix();

	}
}


void drawGLScene()
{
    if (stereo)
    {   glDrawBuffer(GL_BACK);
        // Draw left eye view
        glDrawBuffer(GL_BACK_LEFT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
        cam.setEye(eyeLeft);
        drawFourRods();
		

        // Draw right eye view
        glDrawBuffer(GL_BACK_RIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
        cam.setEye(eyeRight);
        drawFourRods();
		
        glutSwapBuffers();
    }
    else
    {   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(0.0,0.0,0.0,1.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        cam.setEye(eyeRight);
        drawFourRods();
		
        glutSwapBuffers();
    }
}

void initTrial()
{
	//factors = trial.getNext();
	trial.next();
	isStimulusDrawn=false;
	drawGLScene();
	initProjectionScreen(trial.getCurrent()["AbsDepth"]);
	
	xedge = 0;
	zedge = 0;
	theta = 0;
	jitter = unifRand(str2num<double>(parameters.find("VirtualObjRelDepthLowBound")),
		str2num<double>(parameters.find("VirtualObjRelDepthUpBound")));
	jitterX = unifRand(str2num<double>(parameters.find("VirtualObjSizeLowBound")),
		str2num<double>(parameters.find("VirtualObjSizeUpBound")));
	timer.start();
	isStimulusDrawn=true;
}

// Funzione di callback per gestire pressioni dei tasti
void handleKeypress(unsigned char key, int x, int y)
{   switch (key)
    {   //Quit program
	case 'x':
		
		trial.next();
		isStimulusDrawn=false;
		drawGLScene();
		initProjectionScreen(trial.getCurrent()["AbsDepth"]);
		isStimulusDrawn=true;
		break;
	
    case 27:	//corrisponde al tasto ESC
    {   
        exit(0);
    }
    break;
    case ' ':
    {
        // Here we record the head shape - coordinates of eyes and markers, but centered in (0,0,0)
        if ( headCalibrationDone==0 && allVisiblePatch )
        {
            headEyeCoords.init(markers[1].p-Vector3d(230,0,0),markers[1].p, markers[5].p,markers[6].p,markers[7].p,interoculardistance );
            headCalibrationDone=1;
			beepOk(0);
            break;
        }
        // Second calibration, you must look a fixed fixation point
        if ( headCalibrationDone==1 && allVisiblePatch )
        {
            headEyeCoords.init( headEyeCoords.getP1(),headEyeCoords.getP2(), markers[5].p, markers[6].p,markers[7].p,interoculardistance );
            headCalibrationDone=2;
            break;
        }
    }
    break;
    // Enter key: press to make the final calibration
    case 13:
    {
        if ( headCalibrationDone == 2 && allVisiblePatch )
        {
            headEyeCoords.init( headEyeCoords.getP1(),headEyeCoords.getP2(), markers[1].p, markers[2].p,markers[3].p,interoculardistance );
            headCalibrationDone=3;
			visibleInfo=false;
        }
    }
    break;
      case '5':
      {  
		  if(dz/2>10)
			zedge -= .25;
		  else
			  zedge = zedge;
	  }
      break;
      case '8':
      {  
		 zedge += .25;
      }
      break;
	  case '4':
      {  
		xedge -= .25;
	  }
      break;
      case '6':
      {  
		  xedge += .25;
      }
      break;
      case 'a':
      {  
		  theta -= M_PI/2.0;
      }
      break;
      case 's':
      {  
		  theta += M_PI/2.0;
	  }
      break;
	  
	  case '1':
	  {
		  depth += 0.5;
	  }
	  break;
	  case '2':
	  {
		  depth -= 0.5;
	  }
	  break;
	  case '3':
		  {	  
			  dy -= 0.5;
		  }
		  break;
	  case '7':
		  {
			  dy += 0.5;
		  }
		  break;

	}
}

// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0,0,SCREEN_WIDTH, SCREEN_HEIGHT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
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
		moveScreenAbsolute(_focalDist,homeFocalDistance,3500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist,homeFocalDistance,3500);
}

void positionObj(Vector3d pos)
{
	//Vector3d objLocation(0,-44,-360);
	moveObjectAbsolute(pos, centercal, 5000);
}
// Questa funzione si occupa di fare il refresh della schermata ed e' chiamata ogni TIMER_MS millisecond, tienila cosi'
void update(int value)
{
    glutPostRedisplay();
    glutTimerFunc(TIMER_MS, update, 0);
}

void idle()
{
timeFile << globalTimer.getElapsedTimeInMilliSec() << endl;
    eyeLeft = headEyeCoords.getLeftEye();
    eyeRight = headEyeCoords.getRightEye();
	
	eyeRight = Vector3d(interoculardistance/2,0,0);
	eyeLeft = -eyeRight;

	dz = zedge + (jitter/2);
	dx = xedge+jitterX;

	r = sqrt(dx*dz/4 + dz*dz/4);

}


void initRendering()
{   glClearColor(0.0,0.0,0.0,1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    /* Set depth buffer clear value */
    glClearDepth(1.0);
    /* Enable depth test */
    glEnable(GL_DEPTH_TEST);
    /* Set depth function */
    glDepthFunc(GL_LEQUAL);
	
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
	glLineWidth(1.5);
}

void initVariables()
{
	trial.init(parameters);
	initTrial();
}


// Inizializza gli stream, apre il file per poi scriverci
void initStreams()
{
	// Initializza il file parametri partendo dal file parameters.txt, se il file non esiste te lo dice
	ifstream parametersFile;
	parametersFile.open("C:/workspace/cncsvisioncmake/experimentsbrown/parameters/parametersstereopsisTestClean.txt");
	parameters.loadParameterFile(parametersFile);
	
	// Subject name
    string subjectName = parameters.find("SubjectName");

	// Principal streams file
    string responseFileName =  "OrientRespFile_"   + subjectName + ".txt";

	// Check for output file existence
	/// Response file
    if ( !fileExists((responseFileName)) )
        responseFile.open((responseFileName).c_str());

	responseFile << fixed << "NSubjName\tIOD\ttrialN\tAbsDepth\tRelDepthObj\txedge\tEstSize\tzedge\tEstDepth\tjitter\tjitterX\ttheta\torientation\ttrialDuration" << endl;

	timeFile.open("timeFile.txt");	// XXX aggiunto da Nicolini
	globalTimer.start();
}
// Porta tutti i motori nella posizione di home e azzera i contatori degli steps
void initMotors()
{
	homeEverything(5000,3500);
}

int main(int argc, char*argv[])
{
	mathcommon::randomizeStart();
    glutInit(&argc, argv);
    if (stereo)
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

    if (gameMode==false)
    {   glutInitWindowSize( 640,480 );
        glutCreateWindow("EXP WEXLER");
//glutFullScreen();
    }
    else
	{   glutGameModeString("1024x768:32@85");
        glutEnterGameMode();
        glutFullScreen();
    }

    initRendering();
	initGLVariables();
	initStreams();
	initVariables();
    glutDisplayFunc(drawGLScene);
    glutKeyboardFunc(handleKeypress);
    glutReshapeFunc(handleResize);
    glutIdleFunc(idle);
    glutTimerFunc(TIMER_MS, update, 0);
    glutSetCursor(GLUT_CURSOR_NONE);
	boost::thread initVariablesThread(&initVariables);
    glutMainLoop();

    return 0;
}
