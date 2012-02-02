/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public8 License for more details.                      *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/




//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <stdio.h>
#include <time.h>

#include "NxPhysics.h"
#include "ErrorStream.h"
#include "Utilities.h"
#include "cooking.h"
#include "Stream.h"

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.h"

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintState = TRUE;
XnPoint3D jpoint[2][15]; // 現在の各関節点の座標(Scenedrawerを参照

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

#define GL_WIN_SIZE_X 900
#define GL_WIN_SIZE_Y 675
#define M_PI       3.14159265358979323846

XnBool g_bPause = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;

// PhysX
static NxPhysicsSDK*	gPhysicsSDK = NULL; 
static NxScene*			gScene = NULL;

NxActor* Head = NULL;
NxActor* Neck = NULL;
NxActor* rightshoulder = NULL;
NxActor* rightelbow = NULL;
NxActor* righthand = NULL;
NxActor* leftshoulder = NULL;
NxActor* leftelbow = NULL;
NxActor* lefthand = NULL;
NxActor* Torso = NULL;
NxActor* righthip = NULL;
NxActor* rightknee = NULL;
NxActor* rightfoot = NULL;// 右足の挙動
NxActor* lefthip = NULL;
NxActor* leftknee = NULL;
NxActor* leftfoot = NULL;
NxActor* leg = NULL;
NxActor* ballactor = NULL; // ボール
NxActor* target1 = NULL; // 各パネルの情報
NxActor* target2 = NULL;
NxActor* target3 = NULL;
NxActor* target4 = NULL;
NxActor* target5 = NULL;
NxActor* target6 = NULL;
NxActor* target7 = NULL;
NxActor* target8 = NULL;
NxActor* dodai1 = NULL;
NxActor* dodai2 = NULL;
NxActor* dodai3 = NULL;
NxActor* mato1 = NULL;
NxActor* mato2 = NULL;
NxActor* mato3 = NULL;
NxActor* mato4 = NULL;
NxActor* mato5 = NULL;
NxActor* mato6 = NULL;
NxActor* mato7 = NULL;
NxActor* mato8 = NULL;
NxActor* mato9 = NULL;

	XnPoint3D oldjpoint[2][15];
	NxVec3 head;
	NxVec3 neck;
	NxVec3 rshoulder;
	NxVec3 relbow;
	NxVec3 rhand;
	NxVec3 lshoulder;
	NxVec3 lelbow;
	NxVec3 lhand;
	NxVec3 torso;
	NxVec3 rhip;
	NxVec3 rknee;
	NxVec3 rfoot;//右足
	NxVec3 lhip;
	NxVec3 lknee;
	NxVec3 lfoot; //左足
	NxVec3 rightreduction; // １フレーム前との右足の位置の差
	NxVec3 leftreduction;

int touchcount[9];
int ballsize = 100;
int positionx = 35;
int positiony = 900;
int positionz = 2650;

// 描画
static NxVec3	gEye(0.0f, 400.0f, -2000.0f); // 初期視点位置
static NxVec3	gDir(0.0f,0.0f,1.0f);
static NxVec3	gViewY;
static int		gMouseX = 0;
static int		gMouseY = 0;

int gameflag = 0;
//int kickflag = 0;
int endcountflag = 0;
char gametime[25]={0};
clock_t t1, t2;
clock_t balltimer1, balltimer2;
double btime, gtime;
double balldire=100.0, ballspeed=250.0;

// 関数のプロトタイプ宣言
static bool InitNx(void);
static void ArrowKeyCallback(int key, int x, int y);
static void MouseCallback(int button, int state, int x, int y);
static void MotionCallback(int x, int y);
static void ReshapeCallback(int width, int height);
static void cylinder(float radius,float height,int sides);
static void Mycylinder(float radius,float height,int sides);
static void cuboid(float width,float height,float depth);
static void link(float pole1[], float pole2[]);
static void ylink( float x1, float y1, float z1, float x2, float y2, float z2);
static void tlink( float x1, float y1, float z1, float x2, float y2, float z2);
static void position(int value);
static void panelcreate();
static NxActor* CreateCube(const NxVec3& pos, int size, const NxVec3* initialVelocity = NULL);
static NxActor* Createtrg(const NxVec3& pos, int width, int length, int height, int houkou, int col, const NxVec3* initialVelocity = NULL);
static NxActor* CreateSphere(const NxVec3& pos, float radius, int ball, const NxVec3* initialvelocity = NULL);
static NxActor* CreateCapsule(const NxVec3& pos, float radius, NxF32 height, float flags, const NxVec3* initialvelocity = NULL);
//static NxTriangleMesh* createTriangleMesh(NxPhysicsSDK* pNxPhysicsSDK, LPD3DXMESH pMesh);
void drawBitmapString(void *font, char *string);

// ユーザーデータの定義
struct myData
{
	int color;
	int	size;
	int shape;
	int height;
	int flags;
	int width;
	int length;
	int houkou;
	int ball;
};

static float pole[][3]=
{{3000.0,0.0,4000.0},      // [0]右下前
 {3000.0,0.0,6000.0},      // [1]右下奥
 {-3000.0,0.0,4000.0},     // [2]左下前
 {-3000.0,0.0,6000.0},     // [3]左下奥
 {3000.0,2000.0,4000.0},   // [4]右上前
 {3000.0,2000.0,6000.0},   // [5]右上奥
 {-3000.0,2000.0,4000.0},  // [6]左上前
 {-3000.0,2000.0,6000.0}}; // [7]左上奥

// 接触のコールバック
class ContactCallback : public NxUserContactReport
{
	void onContactNotify(NxContactPair& pair, NxU32 events)
	{
		myData* mydata;

		mydata=(myData*)(pair.actors[0]->userData);

		if( mydata->shape==1){
			gScene->releaseActor(*(pair.actors[0]));
		}else
		{
			mydata=(myData*)(pair.actors[1]->userData);
			if(mydata->shape == 1)gScene->releaseActor(*(pair.actors[1]));
		}	
		if( mydata->shape==2 ){
				if( mydata->ball == 0 ){
					pair.actors[0]->setLinearVelocity(rightreduction*100.0); // reductionの計算される間隔を意図的に設定した。(position関数を使用)
				}
				else {
					pair.actors[1]->setLinearVelocity(rightreduction*100.0);
			}


		}//else{*/
			if(-400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x<= 400
				&& 0<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 800)
			{
				touchcount[0]++;
				if(touchcount[0]==3){
				touchcount[0]=0;
				balldire=2.0;
				ballspeed=250.0;
				}
			}
			if(-400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< 400
				&& 800<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 1600)
			{
				touchcount[1]++;
				if(touchcount[1]==3){
				touchcount[1]=0;
				}
			}
			if(-400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< 400
				&& 1600<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 2400)
			{
				touchcount[2]++;
				if(touchcount[2]==3){
				touchcount[2]=0;
				}
			}
			if(400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< 1200
				&& 0<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 800)
			{
				touchcount[3]++;
				if(touchcount[3]==3){
				touchcount[3]=0;
				}
			}
			if(400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< 1200
				&& 800<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 1600)
			{
				touchcount[4]++;
				if(touchcount[4]==3){
				touchcount[4]=0;
				}
			}
			if(400<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< 1200
				&& 1600<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 2400)
			{
				touchcount[5]++;
				if(touchcount[5]==3){
				touchcount[5]=0;
				}
			}
			if(-1200<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< -400
				&& 0<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 800)
			{
				touchcount[6]++;
				if(touchcount[6]==3){
				touchcount[6]=0;
				}
			}
			if(-1200<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< -400
				&& 800<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 1600)
			{
				touchcount[7]++;
				if(touchcount[7]==3){
				touchcount[7]=0;
				}
			}
			if(-1200<= leg->getGlobalPosition().x && leg->getGlobalPosition().x< -400
				&& 1600<= leg->getGlobalPosition().y && leg->getGlobalPosition().y < 2400)
			{
				touchcount[8]++;
				if(touchcount[8]==3){
				touchcount[8]=0;
				}
			}
		
	}
} contactCallback;

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

void CleanupExit()
{
	g_Context.Shutdown();
	if(gPhysicsSDK != NULL)
	{
		if(gScene != NULL) gPhysicsSDK->releaseScene(*gScene);
		gScene = NULL;
		NxReleasePhysicsSDK(gPhysicsSDK);
		gPhysicsSDK = NULL;
	}
	exit (1);
}

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("New User %d\n", nId);
	// New user found
	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}
// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	printf("Lost user %d\n", nId);
}
// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
	printf("Pose %s detected for user %d\n", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}
// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	printf("Calibration started for user %d\n", nId);
}
// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		// Calibration succeeded
		printf("Calibration complete, start tracking user %d\n", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		// Calibration failed
		printf("Calibration failed for user %d\n", nId);
		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

// this function is called each frame
void glutDisplay (void)
{

	if(gScene == NULL) return;
	
	gScene->simulate(1.0f/30.0f);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	// 射影マトリックス
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(100.0f, (float)glutGet(GLUT_WINDOW_WIDTH)/(float)glutGet(GLUT_WINDOW_HEIGHT), 1.0f, 10000.0f); // 視点の位置

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(gEye.x, gEye.y, gEye.z, gEye.x + gDir.x, gEye.y + gDir.y, gEye.z + gDir.z, 0.0f, 1.0f, 0.0f);
	
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);

	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitAndUpdateAll();
	}

	for(int i=0;i<2;i++){
		for(int j=0;j<15;j++){
			oldjpoint[i][j]=jpoint[i][j];
		}
	}
	// Process the data
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	CalculateJoint();

	head.x=(jpoint[0][0].X);
	head.y=(jpoint[0][0].Y);
	head.z=(jpoint[0][0].Z);

	neck.x=(jpoint[0][1].X);
	neck.y=(jpoint[0][1].Y);
	neck.z=(jpoint[0][1].Z);
	
	rshoulder.x=(jpoint[0][2].X);
	rshoulder.y=(jpoint[0][2].Y);
	rshoulder.z=(jpoint[0][2].Z);

	relbow.x=(jpoint[0][3].X*2+oldjpoint[0][3].X)/3;
	relbow.y=(jpoint[0][3].Y*2+oldjpoint[0][3].Y)/3;
	relbow.z=(jpoint[0][3].Z*2+oldjpoint[0][3].Z)/3;

	rhand.x=(jpoint[0][4].X*2+oldjpoint[0][4].X)/3;
	rhand.y=(jpoint[0][4].Y*2+oldjpoint[0][4].Y)/3;
	rhand.z=(jpoint[0][4].Z*2+oldjpoint[0][4].Z)/3;

	lshoulder.x=(jpoint[0][5].X*2+oldjpoint[0][5].X)/3;
	lshoulder.y=(jpoint[0][5].Y*2+oldjpoint[0][5].Y)/3;
	lshoulder.z=(jpoint[0][5].Z*2+oldjpoint[0][5].Z)/3;

	lelbow.x=(jpoint[0][6].X*2+oldjpoint[0][6].X)/3;
	lelbow.y=(jpoint[0][6].Y*2+oldjpoint[0][6].Y)/3;
	lelbow.z=(jpoint[0][6].Z*2+oldjpoint[0][6].Z)/3;

	lhand.x=(jpoint[0][7].X*2+oldjpoint[0][7].X)/3;
	lhand.y=(jpoint[0][7].Y*2+oldjpoint[0][7].Y)/3;
	lhand.z=(jpoint[0][7].Z*2+oldjpoint[0][7].Z)/3;

	torso.x=(jpoint[0][8].X*2+oldjpoint[0][8].X)/3;
	torso.y=(jpoint[0][8].Y*2+oldjpoint[0][8].Y)/3;
	torso.z=(jpoint[0][8].Z*2+oldjpoint[0][8].Z)/3;

	rhip.x=(jpoint[0][9].X*2+oldjpoint[0][9].X)/3;
	rhip.y=(jpoint[0][9].Y*2+oldjpoint[0][9].Y)/3;
	rhip.z=(jpoint[0][9].Z*2+oldjpoint[0][9].Z)/3;

	rknee.x=(jpoint[0][10].X*2+oldjpoint[0][10].X)/3;
	rknee.y=(jpoint[0][10].Y*2+oldjpoint[0][10].Y)/3;
	rknee.z=(jpoint[0][10].Z*2+oldjpoint[0][10].Z)/3;

	rfoot.x=(jpoint[0][11].X*2+oldjpoint[0][11].X)/3;
	rfoot.y=(jpoint[0][11].Y*2+oldjpoint[0][11].Y)/3;
	rfoot.z=(jpoint[0][11].Z*2+oldjpoint[0][11].Z)/3;

	lhip.x=(jpoint[0][12].X*2+oldjpoint[0][12].X)/3;
	lhip.y=(jpoint[0][12].Y*2+oldjpoint[0][12].Y)/3;
	lhip.z=(jpoint[0][12].Z*2+oldjpoint[0][12].Z)/3;

	lknee.x=(jpoint[0][13].X*2+oldjpoint[0][13].X)/3;
	lknee.y=(jpoint[0][13].Y*2+oldjpoint[0][13].Y)/3;
	lknee.z=(jpoint[0][13].Z*2+oldjpoint[0][13].Z)/3;

	lfoot.x=(jpoint[0][14].X*2+oldjpoint[0][14].X)/3;
	lfoot.y=(jpoint[0][14].Y*2+oldjpoint[0][14].Y)/3;
	lfoot.z=(jpoint[0][14].Z*2+oldjpoint[0][14].Z)/3;

	printf("%f, %f, %f\n",rightreduction.x, rightreduction.y, rightreduction.z);
	printf("%f, %f, %f\n",leftreduction.x, leftreduction.y, leftreduction.z);

	if(jpoint[0][8].X!=0.0&&jpoint[0][8].Y!=0.0&&jpoint[0][8].Z!=0.0){

		Head->setGlobalPosition(NxVec3(-head.x+positionx, -head.y+positiony, -head.z+positionz));
		Head->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		Neck->setGlobalPosition(NxVec3(-neck.x+positionx, -neck.y+positiony, -neck.z+positionz));
		Neck->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		rightshoulder->setGlobalPosition(NxVec3(-rshoulder.x+positionx, -rshoulder.y+positiony, -rshoulder.z+positionz));
		rightshoulder->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		rightelbow->setGlobalPosition(NxVec3(-relbow.x+positionx, -relbow.y+positiony, -relbow.z+positionz));
		rightelbow->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		righthand->setGlobalPosition(NxVec3(-rhand.x+positionx, -rhand.y+positiony, -rhand.z+positionz));
		righthand->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		leftshoulder->setGlobalPosition(NxVec3(-lshoulder.x+positionx, -lshoulder.y+positiony, -lshoulder.z+positionz));
		leftshoulder->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		leftelbow->setGlobalPosition(NxVec3(-lelbow.x+positionx, -lelbow.y+positiony, -lelbow.z+positionz));
		leftelbow->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		lefthand->setGlobalPosition(NxVec3(-lhand.x+positionx, -lhand.y+positiony, -lhand.z+positionz));
		lefthand->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		Torso->setGlobalPosition(NxVec3(-torso.x+positionx, -torso.y+positiony, -torso.z+positionz));
		Torso->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		righthip->setGlobalPosition(NxVec3(-rhip.x+positionx, -rhip.y+positiony, -rhip.z+positionz));
		righthip->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		rightknee->setGlobalPosition(NxVec3(-rknee.x+positionx, -rknee.y+positiony, -rknee.z+positionz));
		rightknee->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		rightfoot->setGlobalPosition(NxVec3(-rfoot.x+positionx, -rfoot.y+positiony, -rfoot.z+positionz));
		rightfoot->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

		lefthip->setGlobalPosition(NxVec3(-lhip.x+positionx, -lhip.y+positiony, -lhip.z+positionz));
		lefthip->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		leftknee->setGlobalPosition(NxVec3(-lknee.x+positionx, -lknee.y+positiony, -lknee.z+positionz));
		leftknee->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));
		leftfoot->setGlobalPosition(NxVec3(-lfoot.x+positionx, -lfoot.y+positiony, -lfoot.z+positionz));
		leftfoot->setLinearVelocity(NxVec3(0.0f, 0.0f, 0.0f));

	}

	glPushMatrix();
	glBegin(GL_POLYGON);
	glColor4f(0.2f,1.2f,0.1f,1.0f); // 地面の色
	glVertex3f( 10000.0f,-1.0f, 10000.0f);
	glVertex3f( 10000.0f,-1.0f,-10000.0f);
	glVertex3f(-10000.0f,-1.0f,-10000.0f);
	glVertex3f(-10000.0f,-1.0f, 10000.0f);
	glEnd();
	glPopMatrix();

	//地平線の描画
	glPushMatrix();
	glColor4f(1.0f, 1.0f, 1.0f,1.0f);
	glLineWidth(1);
	glBegin(GL_LINES);
	for(int i=-10000; i< 10000; i+=500) {
		glVertex3f( i    , -0.2f,-10000 );
		glVertex3f( i    , -0.2f, 10000 );
		glVertex3f(-10000, -0.2f, i     ); 
		glVertex3f( 10000, -0.2f, i     ); 
	}
	glEnd();
	glPopMatrix();
	
	switch(gameflag){
		case 0: // スタート画面
			glViewport( 0, 0, (float)glutGet(GLUT_WINDOW_WIDTH),(float)glutGet(GLUT_WINDOW_HEIGHT) );
			glMatrixMode( GL_PROJECTION );
			glLoadIdentity();
			glOrtho( 0.0f, (float)glutGet(GLUT_WINDOW_WIDTH),(float)glutGet(GLUT_WINDOW_HEIGHT), 0.0f, 0.0f, 1.0f );
			glMatrixMode( GL_MODELVIEW );
			glLoadIdentity();

			glPushMatrix();
			glColor3d(1.0, 1.0, 0.0);
			glRasterPos3d(GL_WIN_SIZE_X/2-80, GL_WIN_SIZE_Y/2, 0.0);
			drawBitmapString(GLUT_BITMAP_HELVETICA_18,"Push 's' -> START");
			glPopMatrix();
			printf("Push 's' -> START\n");
			break;

		case 1: // ゲーム終了時
			glViewport( 0, 0, (float)glutGet(GLUT_WINDOW_WIDTH),(float)glutGet(GLUT_WINDOW_HEIGHT) );
			glMatrixMode( GL_PROJECTION );
			glLoadIdentity();
			glOrtho( 0.0f, (float)glutGet(GLUT_WINDOW_WIDTH),(float)glutGet(GLUT_WINDOW_HEIGHT), 0.0f, 0.0f, 1.0f );
			glMatrixMode( GL_MODELVIEW );
			glLoadIdentity();

			glPushMatrix();
			glColor3d(0.0, 0.0, 1.0);
			glRasterPos3d(GL_WIN_SIZE_X/2-60, GL_WIN_SIZE_Y/2+20, 0.0);
			drawBitmapString(GLUT_BITMAP_HELVETICA_18,"GAME OVER");
			glPopMatrix();
			glPushMatrix();
			glColor3d(1.0, 1.0, 0.0);
			glRasterPos3d(GL_WIN_SIZE_X/2-65, GL_WIN_SIZE_Y/2, 0.0);
			drawBitmapString(GLUT_BITMAP_HELVETICA_18,gametime);
			glPopMatrix();
			glPushMatrix();
			glColor3d(1.0, 0.0, 0.0);
			glRasterPos3d(GL_WIN_SIZE_X/2-90, GL_WIN_SIZE_Y/2-20, 0.0);
			drawBitmapString(GLUT_BITMAP_HELVETICA_18,"Push 'r' -> RESTART");
			glPopMatrix();
			printf("GAME OVER\ntime -> %f\n",(double)(t2 - t1) / CLOCKS_PER_SEC);
			break;

		case 2: //　ゲーム中の画面

			glEnable(GL_DEPTH_TEST);
			// アクターの描画
			int nbActors = gScene->getNbActors();
			NxActor** actors = gScene->getActors();
			while(nbActors--)
			{
				NxActor* actor = *actors++;
				if(!actor->userData) continue;

				glPushMatrix();
				glEnable(GL_LIGHTING);
				float glMat[16];
				actor->getGlobalPose().getColumnMajor44(glMat);
				glMultMatrixf(glMat);
				myData* mydata = (myData*)actor->userData;
				int shape = mydata->shape;
				int color = mydata->color;

				switch(shape){
				case 1:
						glColor4f(0.0f,0.0f,1.0f,1.0f);
						if(mydata->houkou==0){
						link(pole[0], pole[4]);
						}
						else if(mydata->houkou==1){
						tlink(3000.0,2000.0,6000.0,-3000.0,2000.0,6000.0);
						tlink(-3000.0,2000.0,4000.0,-3000.0,2000.0,6000.0);
						}
						else if(mydata->houkou==2){
						ylink(3000.0,2000.0,4000.0,-3000.0,2000.0,4000.0);
						ylink(3000.0,2000.0,6000.0,-3000.0,2000.0,6000.0);
						}
						else if(mydata->houkou==3){ // パネル
						glColor4f(0.0f,1.0f,1.0f,1.0f);
						cuboid(mydata->width*2,mydata->length*2, mydata->height*2);
						}
						else if(mydata->houkou==4){
						}
						break;
					case 2:
						if(mydata->ball == 1){
						glColor4f(1.0f,0.0f,0.0f,1.0f); // ボールの色
						glutSolidSphere(mydata->size,15,15);
						}
						else if(mydata->ball == 0){
							glColor4f(1.0f,1.0f,0.0f,1.0f);
							glutSolidSphere(mydata->size,15,15);
						}
						break;
					case 4:
						glColor4f(1.0f,0.0f,0.0f,1.0f);
						cylinder(50, 100, 50);
						break;
				}
				glDisable(GL_LIGHTING);
				glPopMatrix();


			}

			glDisable(GL_DEPTH_TEST);
			t2 = clock();
			balltimer2 = clock();
			btime=(double)(balltimer2 - balltimer1) / CLOCKS_PER_SEC;
			gtime=(double)(t2 - t1) / CLOCKS_PER_SEC;
			printf("%f\n",gtime);

			if(gtime<=10.0){
				balldire=2.0;
				ballspeed=250.0;
			}
			else if(gtime<=20.0){
				balldire=1.5;
				ballspeed=300.0;
			}
			else if(gtime<=30.0){
				balldire=1.0;
				ballspeed=350.0;
			}
			else if(gtime<=40.0){
				balldire=0.5;
				ballspeed=400.0;
			}
			break;
	}

	// 描画の終了
	gScene->flushStream();
	gScene->fetchResults(NX_RIGID_BODY_FINISHED, true);
	glutSwapBuffers();
}

void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		CleanupExit();
	case 's':
		if(gameflag==0){
			gameflag=2;
			t1 = clock();
		}
		break;
	case 'r':
		gameflag=0;
		endcountflag=0;

		for(int i =0;i<9;i++)
			touchcount[i]=0;	
		{
			int nbActors = gScene->getNbActors();
			NxActor** actors = gScene->getActors();
			for(int i=28;i<nbActors;i++){
				gScene->releaseActor(*actors[i]);// オブジェクトリセット
			}
panelcreate();
		}
	
		break;
	case 't':
		t1 = clock();
		break;
	case 'l':
		// Print ID & state as label, or only ID?
		g_bPrintState = !g_bPrintState;
		break;
	case'p':
		leg->setGlobalPosition(NxVec3(0.0,100.0,1000.0)); // 再配置位置
		leg->setLinearVelocity(NxVec3(0, 0,0));
		leg->setAngularVelocity(NxVec3(0,0,0));
		endcountflag++;
		if(endcountflag>9)
		{
			gameflag = 1;
		}
		break;
	case'q':
		leg->setLinearVelocity(NxVec3(0, 0,10000));
		break;

	case 'a': // ボール発射
		if(gameflag==2){
			if(btime>=balldire){
				ballactor=CreateSphere(NxVec3(gEye.x,gEye.y,gEye.z+50), 50, 1);
				ballactor->setLinearVelocity(gDir * ballspeed*10.0);
				balltimer1 = clock();
			}
		}
		break;
	case GLUT_KEY_UP:
		gEye += gDir*30.0f;
		break;
	case GLUT_KEY_DOWN:
		gEye -= gDir*30.0f;
		break;
	case GLUT_KEY_LEFT:
		gEye -= gViewY*30.0f;
		break;
	case GLUT_KEY_RIGHT:
		gEye += gViewY*30.0f;
		break;
	}
}

void glInit (int * pargc, char ** argv)
{
	glutInit(pargc, argv);
	//window1
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	int mainHandle = glutCreateWindow("PKgame");
	glutSetWindow(mainHandle);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutReshapeFunc(ReshapeCallback);
	glutIdleFunc(glutIdle);
	glutSpecialFunc(ArrowKeyCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback);
	MotionCallback(0,0);

	// デフォルトの描画パラメータ
	glClearColor(0.3f, 0.4f, 0.5f, 1.0);
	glEnable(GL_COLOR_MATERIAL);

	// 光源の設定
	float ambientColor[]	= { 0.0f, 0.1f, 0.2f, 0.0f };
	float diffuseColor[]	= { 1.0f, 1.0f, 1.0f, 0.0f };		
	float specularColor[]	= { 0.0f, 0.0f, 0.0f, 0.0f };		
	float position[]		= { 100.0f, 100.0f, -400.0f, 1.0f };		
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, specularColor);
	glLightfv(GL_LIGHT0, GL_POSITION, position);
	glEnable(GL_LIGHT0);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}

#define SAMPLE_XML_PATH "Data/SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)										
	if (nRetVal != XN_STATUS_OK)									
	{																
		printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));
		return nRetVal;												
	}

int main(int argc, char **argv)
{
	XnStatus nRetVal = XN_STATUS_OK;

	if (argc > 1)
	{
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(argv[1]);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			return 1;
		}
	}
	else
	{
		nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH);
		CHECK_RC(nRetVal, "InitFromXml");
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(nRetVal, "Find depth generator");
	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		return 1;
	}
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			printf("Pose required, but not supported\n");
			return 1;
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	glInit(&argc, argv);
	// PhysXの初期化
	if (InitNx()){

		Head          = CreateSphere(NxVec3(1.0,1.0,1.0),ballsize, 0);
		Neck          = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		rightshoulder = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		rightelbow    = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		righthand     = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		leftshoulder  = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		leftelbow     = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		lefthand      = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		Torso         = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		righthip      = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		rightknee     = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		rightfoot     = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		lefthip       = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		leftknee      = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);
		leftfoot      = CreateSphere(NxVec3(1.0,0.0,0.0),ballsize, 0);

		leg=CreateSphere(NxVec3(0.0,100.0,1000.0), 150, 1); // ボール位置

		target1=Createtrg(NxVec3(0.0,4150.0,6000.0), 5800, 50, 50, 2, 0);
		target2=Createtrg(NxVec3(0.0,4150.0,4000.0), 5800, 50, 50, 2, 1);
		target3=Createtrg(NxVec3(3000.0,4150.0,5000.0), 50, 50, 950, 1, 2);
		target4=Createtrg(NxVec3(-3000.0,4150.0,5000.0), 50, 50, 950, 1, 3);
		target5=Createtrg(NxVec3 (pole[4]),200, 2000, 100, 0, 4);
		target6=Createtrg(NxVec3(pole[5]),200, 2000, 100, 0, 5);
		target7=Createtrg(NxVec3(pole[6]),200, 2000, 100, 0, 6);
		target8=Createtrg(NxVec3(pole[7]),100, 2000, 100, 0, 7);

		dodai1=Createtrg(NxVec3(0.0,650.0,5000.0), 2800, 650, 600, 4, 8);
		dodai2=Createtrg(NxVec3(0.0,1950.0,5200.0), 2800, 650, 400, 4, 9);
		dodai3=Createtrg(NxVec3(0.0,3250.0,5400.0), 2800, 650, 200, 4, 10);

panelcreate();
			gScene->setActorPairFlags(*rightfoot, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*leftfoot, *leg, NX_NOTIFY_ON_START_TOUCH);
		printf("create ball\n");
		glutTimerFunc(100.0, position, 0);
		glutMainLoop();
	}

}

// PhysXの初期化
static bool InitNx()
{
	// PhysX SDKの生成
	NxPhysicsSDKDesc desc;
	NxSDKCreateError errorCode = NXCE_NO_ERROR;
	gPhysicsSDK = NxCreatePhysicsSDK(NX_PHYSICS_SDK_VERSION, NULL, new ErrorStream(), desc, &errorCode);
	if(gPhysicsSDK == NULL) 
	{
		printf("\nSDKの生成に失敗\n");
		return false;
	}

	gPhysicsSDK->setParameter(NX_SKIN_WIDTH, 0.05f);

	// シーンの生成
	NxSceneDesc sceneDesc;
	sceneDesc.gravity				= NxVec3(0.0f, -19.6f*8, 0.0f);
	gScene = gPhysicsSDK->createScene(sceneDesc);
	if(gScene == NULL) 
	{
		printf("\nシーンの生成に失敗\n");
		return false;
	}

	gScene->setUserContactReport(&contactCallback);

	// デフォルトのマテリアル
	NxMaterial* defaultMaterial = gScene->getMaterialFromIndex(0);
	defaultMaterial->setRestitution(0.5f);
	defaultMaterial->setStaticFriction(0.5f);
	defaultMaterial->setDynamicFriction(0.5f);

	// 地面の生成
	NxPlaneShapeDesc planeDesc;
	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&planeDesc);
	gScene->createActor(actorDesc);

	return true;
}

// 矢印キーの処理
static void ArrowKeyCallback(int key, int x, int y)
{
	glutKeyboard(key,x,y);
}

// マウスの処理
static void MouseCallback(int button, int state, int x, int y)
{
	gMouseX = x;
	gMouseY = y;
}

// マウスのドラッグの処理
static void MotionCallback(int x, int y)
{
	int dx = gMouseX - x;
	int dy = gMouseY - y;
	
	gDir.normalize();
	gViewY.cross(gDir, NxVec3(0,1,0));

	NxQuat qx(NxPiF32 * dx * 20/ 180.0f, NxVec3(0,1,0));
	qx.rotate(gDir);
	NxQuat qy(NxPiF32 * dy * 20/ 180.0f, gViewY);
	qy.rotate(gDir);

	gMouseX = x;
	gMouseY = y;
}

// ウインドウのサイズの変更
static void ReshapeCallback(int width, int height)
{
	glViewport(0, 0, width, height);
}

//boxの生成
static NxActor* CreateCube(const NxVec3& pos, int size, const NxVec3* initialVelocity)
{
	if(gScene == NULL) return NULL;	

	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping	= 0.5f;
	if(initialVelocity) bodyDesc.linearVelocity = *initialVelocity;

	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions = NxVec3((float)size, (float)size, (float)size);

	myData* mydata = new myData;
	mydata->shape = 1;
	mydata->size = size;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 10.0f;
	actorDesc.globalPose.t  = pos;
	actorDesc.userData = mydata;

	return gScene->createActor(actorDesc);
}

// 柱の生成
static NxActor* Createtrg(const NxVec3& pos, int width, int length, int height, int houkou, int col, const NxVec3* initialVelocity)
{
	if(gScene == NULL) return NULL;	

	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping	= 0.5f;
	if(initialVelocity) bodyDesc.linearVelocity = *initialVelocity;

	NxBoxShapeDesc boxDesc;
	boxDesc.dimensions = NxVec3((float)width, (float)length, (float)height);

	myData* mydata = new myData;
	mydata->color = col;
	mydata->shape = 1;
	mydata->width = width;
	mydata->length = length;
	mydata->height = height;
	mydata->houkou = houkou;
	//mydata->size = size;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&boxDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 100000.0f;
	actorDesc.globalPose.t  = pos;
	actorDesc.userData = mydata;

	return gScene->createActor(actorDesc);
}
// 人間の描画
static NxActor* CreateCapsule(const NxVec3& pos, NxF32 radius, NxF32 height, NxF32 flags, const NxVec3* initialVelocity)
{
	if(gScene == NULL) return NULL;	

	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping	= 0.5f;
	if(initialVelocity) bodyDesc.linearVelocity = *initialVelocity;

	NxCapsuleShapeDesc capsuleDesc;
	capsuleDesc.radius = radius;
	capsuleDesc.height = height;
	capsuleDesc.flags = flags;

	myData* mydata = new myData;
	mydata->shape = 4;
	mydata->size = radius;
	mydata->height = height;
	mydata->flags = flags;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&capsuleDesc);
	actorDesc.body			= &bodyDesc;
	actorDesc.density		= 10.0f;
	actorDesc.globalPose.t  = pos;
	actorDesc.userData = mydata;

	return gScene->createActor(actorDesc);
}

//球の生成
static NxActor* CreateSphere(const NxVec3& pos, NxF32 radius, int ball, const NxVec3* initialvelocity)
{
	if(gScene == NULL) return NULL;
	//Create body
	NxBodyDesc bodyDesc;
	bodyDesc.angularDamping = 0.5f;
	if(initialvelocity) bodyDesc.linearVelocity = *initialvelocity;
	NxSphereShapeDesc sphereDesc;
	sphereDesc.radius = radius;

	myData* mydata = new myData;
	mydata->shape = 2;
	mydata->size = radius;
	mydata->ball = ball;

	NxActorDesc actorDesc;
	actorDesc.shapes.pushBack(&sphereDesc);
	actorDesc.body = &bodyDesc;
	actorDesc.density = 10.0f;
	actorDesc.globalPose.t = pos;
	actorDesc.userData = mydata;

	return gScene->createActor(actorDesc);
}

void drawBitmapString(void *font, char *string)
{
  glPushAttrib(GL_CURRENT_BIT);

  /* ビットマップ文字列の描画 */
  while (*string) 
    glutBitmapCharacter(font, *string++);

  glPopAttrib();
}

void Mycylinder(float radius,float height,int sides)
{
 double pi = 3.1415;
 //上面
 glNormal3d(0.0, 1.0, 0.0);
 glBegin(GL_POLYGON);
glutSolidSphere(radius, 15, 15);
 glEnd();
 //側面
 glBegin(GL_QUAD_STRIP);
 for(double i=0;i<=sides;i=i+1){
  double t = i*2*pi/sides;
  glNormal3f((GLfloat)cos(t),0.0,(GLfloat)sin(t));
  glVertex3f((GLfloat)(radius*cos(t)),-height,(GLfloat)(radius*sin(t)));
  glVertex3f((GLfloat)(radius*cos(t)),height,(GLfloat)(radius*sin(t)));
 }
 glEnd();
 //下面
 glNormal3d(0.0, -1.0, 0.0);
 glBegin(GL_POLYGON);
 glutSolidSphere(radius, 15, 15);
 glEnd();
}


// 円柱の描画用関数
void cylinder(float radius,float height,int sides)
{
 double pi = 3.1415;
 //上面
 glNormal3d(0.0, 1.0, 0.0);
 glBegin(GL_POLYGON);
 for(double i = 0; i < sides; i++) {
  double t = pi*2/sides * (double)i;
  glVertex3d(radius * cos(t), height, radius * sin(t));
 }
 glEnd();
 //側面
 glBegin(GL_QUAD_STRIP);
 for(double i=0;i<=sides;i=i+1){
  double t = i*2*pi/sides;
  glNormal3f((GLfloat)cos(t),0.0,(GLfloat)sin(t));
  glVertex3f((GLfloat)(radius*cos(t)),-height,(GLfloat)(radius*sin(t)));
  glVertex3f((GLfloat)(radius*cos(t)),height,(GLfloat)(radius*sin(t)));
 }
 glEnd();
 //下面
 glNormal3d(0.0, -1.0, 0.0);
 glBegin(GL_POLYGON);
 for(double i = sides; i >= 0; --i) {
  double t = pi*2/sides * (double)i;
  glVertex3d(radius * cos(t), -height, radius * sin(t));
 }
 glEnd();
}

// 回転
void link( float pole1[], float pole2[])
{
    float x = pole2[0] - pole1[0];
    float y = pole2[1] - pole1[1];
    float z = pole2[2] - pole1[2];
    double dist = sqrt(x*x + y*y + z*z);
    float rad = acos(y / dist);    
    glPushMatrix();
    {
#define EQ(a,b) (fabs(b - a) < 1e-3)
        if (EQ(x, 0.) && EQ(z, 0.))
            glRotated(rad*180/M_PI, 1, 0, 0);
        else{
            glRotated(rad*180/M_PI, z, 0, 1);
			glTranslatef(0.0f, 2000.0f, 0.0f);
		}
        cylinder(100, dist, 100);
    }
    glPopMatrix();
}

void ylink( float x1, float y1, float z1, float x2, float y2, float z2)
{
    float x = x2 - x1;
    float y = y2 - y1;
    float z = z2 - z1;
    double dist = sqrt(x*x + y*y + z*z);
    float rad = acos(y / dist);    
    glPushMatrix();
    {
            glRotated(rad*180/M_PI, 0, 0, 1);
        cylinder(100, 3000, 100);
    }
    glPopMatrix();
}

void tlink( float x1, float y1, float z1, float x2, float y2, float z2)
{
    float x = x2 - x1;
    float y = y2 - y1;
    float z = z2 - z1;
    double dist = sqrt(x*x + y*y + z*z);
    float rad = acos(y / dist);    
    glPushMatrix();
    {
        glRotated(rad*180/M_PI, 1, 0, 0);
        cylinder(100, 1000, 100);
    }
    glPopMatrix();
}
void cuboid(float width,float height,float depth)
{
	 glBegin(GL_QUADS);
	 //前
	 glNormal3f(0.0,0.0,-1.0);
	 glVertex3f(width/2,height/2,depth/2);
	 glVertex3f(-width/2,height/2,depth/2);
	 glVertex3f(-width/2,-height/2,depth/2);
	 glVertex3f(width/2,-height/2,depth/2);

	 //左
	 glNormal3f(1.0,0.0,0.0);
	 glVertex3f(width/2,height/2,depth/2);
	 glVertex3f(width/2,height/2,-depth/2);
	 glVertex3f(width/2,-height/2,-depth/2);
	 glVertex3f(width/2,-height/2,depth/2);

	 //右
	 glNormal3f(-1.0,0.0,0.0);
	 glVertex3f(-width/2,height/2,-depth/2);
	 glVertex3f(-width/2,height/2,depth/2);
	 glVertex3f(-width/2,-height/2,depth/2);
	 glVertex3f(-width/2,-height/2,-depth/2);

	 //後
	 glNormal3f(0.0,0.0,1.0);
	 glVertex3f(width/2,height/2,-depth/2);
	 glVertex3f(-width/2,height/2,-depth/2);
	 glVertex3f(-width/2,-height/2,-depth/2);
	 glVertex3f(width/2,-height/2,-depth/2);

	 //上
	 glNormal3f(0.0,1.0,0.0);
	 glVertex3f(width/2,height/2,depth/2);
	 glVertex3f(-width/2,height/2,depth/2);
	 glVertex3f(-width/2,height/2,-depth/2);
	 glVertex3f(width/2,height/2,-depth/2);

	 //下
	 glNormal3f(0.0,-1.0,0.0);
	 glVertex3f(width/2,-height/2,depth/2);
	 glVertex3f(-width/2,-height/2,depth/2);
	 glVertex3f(-width/2,-height/2,-depth/2);
	 glVertex3f(width/2,-height/2,-depth/2);
	 glEnd();
}


void position(int value)
{
	
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);

	if (!g_bPause)
	{
		// Read next available data
		g_Context.WaitAndUpdateAll();
	}

	for(int i=0;i<2;i++){
		for(int j=0;j<15;j++){
			oldjpoint[i][j]=jpoint[i][j];
		}
	}
	// Process the data
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	CalculateJoint();

	rightreduction.x = ( oldjpoint[0][11].X - jpoint[0][11].X );
	rightreduction.y = ( oldjpoint[0][11].Y - jpoint[0][11].Y );
	rightreduction.z = ( oldjpoint[0][11].Z - jpoint[0][11].Z );

	leftreduction.x = ( oldjpoint[0][14].X - jpoint[0][14].X );
	leftreduction.y = ( oldjpoint[0][14].Y - jpoint[0][14].Y );
	leftreduction.z = ( oldjpoint[0][14].Z - jpoint[0][14].Z );

	glutTimerFunc(100.0, position, 0);
}

void panelcreate() // パネル描画とコールバックの関数
{
			mato1=Createtrg(NxVec3(-1500.0,3100.0,4900.0),500, 500, 50, 3, 10);
			mato2=Createtrg(NxVec3(    0.0,3100.0,4900.0), 500, 500, 50, 3, 10);
			mato3=Createtrg(NxVec3(1500.0,3100.0,4900.0), 500, 500, 50, 3, 10);
			mato4=Createtrg(NxVec3(-1500.0,1800.0,4600.0),500, 500, 50, 3, 10);
			mato5=Createtrg(NxVec3(   0.0,1800.0,4600.0), 500, 500, 50, 3, 10);
			mato6=Createtrg(NxVec3(1500.0,1800.0,4600.0), 500, 500, 50, 3, 10);
			mato7=Createtrg(NxVec3(-1500.0,500.0,4300.0), 500, 500, 50, 3, 10);
			mato8=Createtrg(NxVec3(    0.0,500.0,4300.0), 500, 500, 50, 3, 10);
			mato9=Createtrg(NxVec3(1500.0,500.0,4300.0),  500, 500, 50, 3, 10);
	
			gScene->setActorPairFlags(*mato1, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato2, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato3, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato4, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato5, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato6, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato7, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato8, *leg, NX_NOTIFY_ON_START_TOUCH);
			gScene->setActorPairFlags(*mato9, *leg, NX_NOTIFY_ON_START_TOUCH);
}
