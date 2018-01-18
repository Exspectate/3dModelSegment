#include "lty_STL.h"
#include "dijkstra.h"
#include <glut.h>
#include <math.h>
#include <fstream>
#include "bsp.h"



#define  GLUT_WHEEL_UP 3           //定义滚轮操作  
#define  GLUT_WHEEL_DOWN 4  

struct TY {
	double d;
	int ind;
	TY() {}
	TY(double d_, int i_) : d(d_), ind(i_) {}
	bool operator < (const TY &t) {
		return d < t.d;
	}
};

static const int COLOR1[19] = { 255,255,255,  0,141, 23, 66,255,139,255,139,139,255,139,139,255,255 };
static const int COLOR2[19] = { 255,  0,117,  0, 75,248, 80,129,0  ,101,58 ,69 ,137,0  ,228,250,0 };
static const int COLOR3[19] = { 255,151,  0,255,187,203,102,  0,76 ,0  ,8  ,58 ,0  ,137,0  ,225,205,255 };
static const double PI = 3.1415926535;
static const int MAX = 20;
static int du = 90, OriX = -500, OriY = -50;   //du是视点和x轴的夹角
static float r = 1000, h = 0;   //r是视点绕y轴的半径，h是视点高度即在y轴上的坐标
static float c = PI / 180.0;    //弧度和角度转换参数
static float times = 1;			//放大缩小倍数

static float oldx = 0;					//矩阵变换的坐标
static float oldy = 0;

static int cx = 0;					//交叉点的坐标
static int cy = 0;


static Mesh mesh;


static vector<vector<int>> regionE(MAX);
static int regionCount;
static Dijkstra dija;
static vector<vector<Point3d> >regionPt;



static int bfs(int node, int color, Mesh &mesh)
{
	int ret = 0;
	queue<int> que;
	que.push(node);
	while (!que.empty())
	{
		int tmp = que.front();
		que.pop();
		if (mesh.Verts[tmp].color != 0)
			continue;
		mesh.Verts[tmp].color = color;
		++ret;
		for (int i = 0; i < mesh.adjVert[tmp].size(); i++)
			if (mesh.Verts[mesh.adjVert[tmp][i]].color == 0)
				que.push(mesh.adjVert[tmp][i]);

	}
	return ret;
}

static int reBfs(int node, Mesh &mesh) {  //将本区域点颜色标记为0
	int ret = 0;
	queue<int> que;
	que.push(node);
	while (!que.empty())
	{
		int tmp = que.front();
		que.pop();
		if (mesh.Verts[tmp].color <= 0)
			continue;
		mesh.Verts[tmp].color = 0;
		++ret;
		for (int i = 0; i < mesh.adjVert[tmp].size(); i++)
			if (mesh.Verts[mesh.adjVert[tmp][i]].color > 0)
				que.push(mesh.adjVert[tmp][i]);

	}
	return ret;
}

static void setColor(Mesh &mesh)
{
	int nowColor = 0;
	for (int i = 0; i < mesh.Verts.size(); i++)
	{
		if (mesh.Verts[i].color == 0)
		{
			nowColor++;
			int ret = bfs(i, nowColor, mesh);
			cout << "点数:" << ret << "\n";
		}
	}

	cout << "块数:" << nowColor << endl;
}

static GLfloat winx, winy, winz;
static GLdouble posx, posy, posz;
static void pickPoint() {
	GLdouble modelMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
	GLdouble projMatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projMatrix);
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);

	winy = GLdouble(viewport[3] - (GLint)winy);
	glReadPixels(winx, winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);
	gluUnProject(winx, winy, winz, modelMatrix, projMatrix, viewport, &posx, &posy, &posz);
}

bool mouseLeftDown;
bool mouseRightDown;

static void SpecialKey(GLint key, GLint x, GLint y) { //通过上下左右键盘平移模型
	GLfloat unit = 20.f;  //平移单位
	if (key == GLUT_KEY_UP) {
		oldy += unit;
	}
	if (key == GLUT_KEY_LEFT) {
		oldx -= unit;
	}
	if (key == GLUT_KEY_DOWN) {
		oldy -= unit;
	}
	if (key == GLUT_KEY_RIGHT) {
		oldx += unit;
	}
}

static void Mouse(int button, int state, int x, int y)
{
	GLfloat scaleUnit = 0.06f;   //缩放尺度单位
	//左键控制旋转
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			//记录鼠标按下位置
			OriX = x, OriY = y;
			mouseLeftDown = true;
		}
		else
			mouseLeftDown = false;
	}

	//右键获取点击位置的三维坐标
	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
			//记住鼠标右击后光标坐标
			winx = x;
			winy = y;
			pickPoint();
		}
		else if (state == GLUT_UP)
			mouseRightDown = false;
	}
	/*
	* 鼠标滚轮控制图形缩放
	*/
	else if (state == GLUT_UP && button == GLUT_WHEEL_UP)
	{
		times = scaleUnit + times;
		glutPostRedisplay();
	}

	else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN)
	{
		times = -scaleUnit + times;
		glutPostRedisplay();
	}
}

static void onMouseMove(int x, int y)   //处理鼠标拖动
{
	GLfloat rotateXUnit = 1.0f;
	GLfloat rotateYUnit = 20.0f;
	if (mouseLeftDown) {
		du += (x - OriX)*rotateXUnit; //鼠标在窗口x轴方向上的增量加到视点与x轴的夹角上，就可以左右转
		h += (y - OriY)*rotateYUnit;  //鼠标在窗口y轴方向上的改变加到视点y的坐标上，就可以上下转
		//if (h>1.0)   h = 1.0;  //对视点y坐标作一些限制，不会使视点太奇怪
	//	else if (h<-1.0) h = -1.0;
		OriX = x, OriY = y;  //将此时的坐标作为旧值，为下一次计算增量做准备
	}
	else if (mouseRightDown) {
		//计算拖动后的偏移量，然后进行xy叠加减
		//oldx += ((x - cx) * 1);
		//oldy -= ((y - cy) * 1);
		//glutPostRedisplay();
		//cx = x;
		//cy = y;
	}

}

static void drawSTL() {
	//cout << "drawSTL";
	glClearColor(161. / 255, 175. / 255, 201. / 255, 1.0);   //背景颜色
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除缓冲，GL_COLOR_BUFFER_BIT ：颜色缓冲标志位
	glLoadIdentity();                                       //重置当前矩阵为4*4的单位矩阵
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //从视点看远点

	int targetId = -1;  //记录在模型上与选取点最近点的id
	int targetColor;	//记录当前部件的颜色
	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {    //但单击鼠标右键时,获取到对应三维空间的坐标
		double dis = 1e30;
		for (int i = 0; i < mesh.Verts.size(); ++i) {
			if (mesh.Verts[i].color < 0) continue;  //如果是坏边上的点，则跳过
			Point3d tp = Point3d(posx - mesh.Verts[i].x, posy - mesh.Verts[i].y, posz - mesh.Verts[i].z);
			if (tp.len() < dis) {
				dis = tp.len();
				targetId = i;
			}
		}
		if (dis < 1e2) {      //将选中部件染成白色表示选中
			targetColor = mesh.Verts[targetId].color;
			reBfs(targetId, mesh);
		}
		else { //距离过大说明右键点击在模型外，不予处理
			posx = posy = posz = 0;
		}
	}

	glScalef(times, times, times);//缩放
	glTranslatef(oldx, oldy, 0);    //平移
									//渲染颜色
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	for (int i = 0; i < mesh.Facets.size(); i = i + 1) {
		
		int nowColor = max(max(mesh.Verts[mesh.Facets[i].vertIdx[0]].color, mesh.Verts[mesh.Facets[i].vertIdx[1]].color), mesh.Verts[mesh.Facets[i].vertIdx[2]].color);
		if (nowColor == -1)
		{
			for (int j = 0; j < mesh.adjVert[mesh.Facets[i].vertIdx[0]].size(); j++)
				nowColor = max(nowColor, mesh.Verts[mesh.adjVert[mesh.Facets[i].vertIdx[0]][j]].color);
		}

		if (nowColor == 4) continue;
		if (nowColor == 2) continue;
		if (nowColor == 1) continue;

		glColor3f(1.0*COLOR1[nowColor] / 255, 1.0*COLOR2[nowColor] / 255, 1.0*COLOR3[nowColor] / 255);		//颜色
		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; ++j) {
			glNormal3f(mesh.vertnorm[mesh.Facets[i].vertIdx[j]].x, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].y, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].z);
			glVertex3f(mesh.Verts[mesh.Facets[i].vertIdx[j]].x, mesh.Verts[mesh.Facets[i].vertIdx[j]].y, mesh.Verts[mesh.Facets[i].vertIdx[j]].z);
		}
		glEnd();
	}

	////F1横线
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 0.0);			//绿色
	//glVertex3d(getf1().L.x, getf1().L.y, getf1().L.z);
	//glVertex3d(getf1().R.x, getf1().R.y, getf1().R.z);
	//glEnd();


    ////F1圈上曲线
    //glLineWidth(5.0f);
    //glBegin(GL_LINES); 
    //glColor3f(1.0, 0.0, 0.0);			//红色
    //for (int i = 0; i < getf1().UpCurve.size(); ++i) {

    //    Point3d &a = getf1().UpCurve[i].first;
    //	Point3d &b = getf1().UpCurve[i].second;
    //	glVertex3d(a.x, a.y, a.z);
    //	glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

 //   //F1圈下曲线
 //   glLineWidth(5.0f);
 //   glBegin(GL_LINES);
 //   glColor3f(0.0, 1.0, 0.0);			//绿色
 //   for (int i = 0; i < getf1().DownCurve.size(); ++i) {

 //       Point3d &a = getf1().DownCurve[i].first;
 //       Point3d &b = getf1().DownCurve[i].second;
 //       glVertex3d(a.x, a.y, a.z);
 //       glVertex3d(b.x, b.y, b.z);
 //   }
 //   glEnd();

	////F1圈(F2圈已经包括)
	//glLineWidth(2.0f);
	//glBegin(GL_LINES); 
	//glColor3f(1.0, 0.0, 0.0);			//红色
	//for (int i = 0; i < getf1().inbadEdge.size(); ++i) {

	//	Point3d &a = mesh.Verts[getf1().inbadEdge[i].first];
	//	Point3d &b = mesh.Verts[getf1().inbadEdge[i].second];
	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();

	////F1前曲线
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 0.0);			//红色
	//for (int i = 0; i < getf1().Curve.size()-1; ++i) {

	//	Point3d &a = getf1().Curve[i];
	//	Point3d &b = getf1().Curve[i+1];

	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);

	//}
	//glEnd();

	////F2横线
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 1.0);			//青色
	//glVertex3d(getf2().L.x, getf2().L.y, getf2().L.z);
	//glVertex3d(getf2().R.x, getf2().R.y, getf2().R.z);
	//glEnd();

	////F2边界
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 1.0);			//青色
	//glVertex3d(getf1().L.x, getf1().L.y, getf1().L.z);
	//glVertex3d(getf2().L.x, getf2().L.y, getf2().L.z);

	//glVertex3d(getf1().R.x, getf1().R.y, getf1().R.z);
	//glVertex3d(getf2().R.x, getf2().R.y, getf2().R.z);

	//glEnd();

	////F2圈(前后两个均是，其一与F1圈接近重合)
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 0.0, 1.0);			//蓝色
	//for (int i = 0; i < getf2().inbadEdge.size(); ++i) {

	//	Point3d &a = mesh.Verts[getf2().inbadEdge[i].first];
	//	Point3d &b = mesh.Verts[getf2().inbadEdge[i].second];

	//	glVertex3f(a.x, a.y, a.z);
	//	glVertex3f(b.x, b.y, b.z);

	//}
	//glEnd();


    ////F2圈上曲线
    //glLineWidth(5.0f);
    //glBegin(GL_LINES);
    //glColor3f(1.0, 0.0, 0.0);			//红色
    //for (int i = 0; i < getf2().UpCurve.size(); ++i) {

    //    Point3d &a = getf2().UpCurve[i].first;
    //    Point3d &b = getf2().UpCurve[i].second;
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

    ////F2圈下曲线
    //glLineWidth(5.0f);
    //glBegin(GL_LINES);
    //glColor3f(0.0, 1.0, 0.0);			//绿色
    //for (int i = 0; i < getf2().DownCurve.size(); ++i) {

    //    Point3d &a = getf2().DownCurve[i].first;
    //    Point3d &b = getf2().DownCurve[i].second;
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

	////F3横线
	//glLineWidth(8.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 1.0);			//紫色
	//glVertex3d(getf3().L.x, getf3().L.y, getf3().L.z);
	//glVertex3d(getf3().R.x, getf3().R.y, getf3().R.z);
	//glEnd();

	////F3突变线
	//glLineWidth(2.0f);
	//glBegin(GL_LINE_LOOP);
	//glColor3f(0.0, 0.0, 1.0);			//蓝色
	//for (int i = 0; i < getf3().testpiiv.size(); ++i) {
	//	Point3d &a = mesh.Verts[getf3().testpiiv[i].first];
	//	Point3d &b = mesh.Verts[getf3().testpiiv[i].second];
	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();


    ////F3突变线上的点
    //glPointSize(1);
    //glBegin(GL_POINTS);
    //glColor3f(0.0, 1.0, 0.0);			//绿色
    //for (int i = 0; i < getf3().RidgeLine.size(); ++i) {
    //    Point3d &a = mesh.Verts[getf3().RidgeLine[i].first];
    //    Point3d &b = mesh.Verts[getf3().RidgeLine[i].second];
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

	////F3突变线DOWN上的点
	//glPointSize(1);
	//glBegin(GL_POINTS);
	//glColor3f(0.0, 1.0, 0.0);			//绿色
	//for (int i = 0; i < getf3().DownCurve.size(); ++i) {
	//    Point3d &a = mesh.Verts[getf3().RidgeLine[i].first];
	//    Point3d &b = mesh.Verts[getf3().RidgeLine[i].second];
	//    glVertex3d(a.x, a.y, a.z);
	//    glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();


	//F3度数多的点
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);			//蓝色
	
	//glVertex3d(getf3().bigpoint.x, getf3().bigpoint.y, getf3().bigpoint.z);
 //   glVertex3d(getf3().L.x, getf3().L.y, getf3().L.z);
 //   glVertex3d(getf3().R.x, getf3().R.y, getf3().R.z);

	//glVertex3d(getf3().Pnear2.x, getf3().Pnear2.y, getf3().Pnear2.z);
	//glVertex3d(getf3().Pnear3.x, getf3().Pnear3.y, getf3().Pnear3.z);
	//glVertex3d(getf3().Pnear4.x, getf3().Pnear4.y, getf3().Pnear4.z);
	//glVertex3d(getf3().Pnear5.x, getf3().Pnear5.y, getf3().Pnear5.z);
	glEnd();

	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);			//蓝色
	glVertex3d(getf3().Pnear2.x, getf3().Pnear2.y, getf3().Pnear2.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 1.0, 0.0);			//green色
	glVertex3d(getf3().Pnear3.x, getf3().Pnear3.y, getf3().Pnear3.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);			//red色
	glVertex3d(getf3().Pnear4.x, getf3().Pnear4.y, getf3().Pnear4.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);			//white色
	glVertex3d(getf3().Pnear5.x, getf3().Pnear5.y, getf3().Pnear5.z);
	glEnd();


	////F3边界线
	//glLineWidth(8.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 0.0);			//红色
	//
	//Point3d &a1 = getf3().BorderL.first;
	//Point3d &a2 = getf3().BorderL.second;
	//Point3d &b1 = getf3().BorderR.first;
	//Point3d &b2 = getf3().BorderR.second;

	//glVertex3f(a1.x, a1.y, a1.z);
	//glVertex3f(a2.x, a2.y, a2.z);

	//glVertex3f(b1.x, b1.y, b1.z);
	//glVertex3f(b2.x, b2.y, b2.z);

	//glEnd();



	//ALL LINE
    glLineWidth(3.0f);
 
	for (int k = 0; k < getf3().featureLines.size(); ++k)
	{
		glBegin(GL_LINES);
		int nowColor = 17 - k;
		glColor3f(1.0*COLOR1[nowColor] / 255, 1.0*COLOR2[nowColor] / 255, 1.0*COLOR3[nowColor] / 255);
		vector<Point3d> aline = getf3().featureLines[k];
		for (int i = 0; i < aline.size() - 1; ++i) {
			Point3d &a = aline[i];
			Point3d &b = aline[i + 1];

			glVertex3f(a.x, a.y, a.z);
			glVertex3f(b.x, b.y, b.z);

		}
		glEnd();
	}




	glDisable(GL_COLOR_MATERIAL);



	glutSwapBuffers();                                      //交换两个缓冲区指针

	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {   //单击右键后处理
		system("pause");


		//ofstream ofile;  //输出选中的点
		//ofile.open("temp.txt");
		//for (int i = 0; i < mesh.Verts.size(); ++i) {
		//	if (mesh.Verts[i].color != 0) continue;
		//	ofile << mesh.Verts[i].x << " " << mesh.Verts[i].y << " " << mesh.Verts[i].z << "\n";
		//}
		//ofile.close();

		//bfs(targetId, targetColor, mesh); //在标记完成后，将选中部件染回原来的颜色
		//posx = posy = posz = 0;
	}
}

static void init(void)
{
	GLfloat light_position[] = { 0, 0, 0, 0 };	//光源位置 最后一位 0-方向光源、1-点光源
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	glEnable(GL_LIGHT0);			   //打开0#灯
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//设置LIGHT0的位置
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //散射光属性
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //镜面反射光
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBA模式的环境光，为0 

	GLfloat light_position1[] = { 0, 0, 200.0, 0 };	//光源位置 最后一位 0-方向光源、1-点光源
	GLfloat white_light1[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	GLfloat Light_Model_Ambient1[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	glEnable(GL_LIGHT1);			   //打开1#灯
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);	//设置LIGHT1的位置
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white_light1);      //散射光属性
	glLightfv(GL_LIGHT1, GL_SPECULAR, white_light1);     //镜面反射光
	glLightfv(GL_LIGHT1, GL_AMBIENT, Light_Model_Ambient1);//RGBA模式的环境光，为0 

	GLfloat light_position2[] = { 2000, 0, 0, 0 };	//光源位置 最后一位 0-方向光源、1-点光源
	GLfloat white_light2[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	GLfloat Light_Model_Ambient2[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	glEnable(GL_LIGHT2);			   //打开2#灯
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2);	//设置LIGHT2的位置
	glLightfv(GL_LIGHT2, GL_DIFFUSE, white_light2);      //散射光属性
	glLightfv(GL_LIGHT2, GL_SPECULAR, white_light2);     //镜面反射光
	glLightfv(GL_LIGHT2, GL_AMBIENT, Light_Model_Ambient2);//RGBA模式的环境光，为0 

	//GLfloat light_position3[] = { 2000, 0, 200, 0 };	//光源位置 最后一位 0-方向光源、1-点光源
	//GLfloat white_light3[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	//GLfloat Light_Model_Ambient3[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	//glEnable(GL_LIGHT3);			   //打开0#灯
	//glLightfv(GL_LIGHT3, GL_POSITION, light_position3);	//设置LIGHT0的位置
	//glLightfv(GL_LIGHT3, GL_DIFFUSE, white_light3);      //散射光属性
	//glLightfv(GL_LIGHT3, GL_SPECULAR, white_light3);     //镜面反射光
	//glLightfv(GL_LIGHT3, GL_AMBIENT, Light_Model_Ambient3);//RGBA模式的环境光，为0 

	glEnable(GL_LIGHTING);			   //开关:使用光
	glShadeModel(GL_SMOOTH);           //颜色过度模式
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);		   //深度测试
									   //glEnable(GL_DEPTH_TEST);    //启用深度，根据坐标的远近自动隐藏被遮住的图形（材料）
}

static void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //截图;1、2为视口的左下角;3、4为视口的宽度和高度
	glMatrixMode(GL_PROJECTION);    //将当前矩阵指定为投影矩阵
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 4000.0); //1、视野在Y-Z平面的角度[0,180];2、投影平面宽度与高度的比率;3、近截剪面到视点的距离;4、远截剪面到视点的距离
	glMatrixMode(GL_MODELVIEW);     //对模型视景矩阵堆栈应用随后的矩阵操作.
}

//static void Mouse(int button, int state, int x, int y)
//{
//	if (state == GLUT_DOWN)  //记录鼠标按下位置
//		OriX = x, OriY = y;
//}

//static void onMouseMove(int x, int y)   //处理鼠标拖动
//{
//	du += x - OriX; //鼠标在窗口x轴方向上的增量加到视点与x轴的夹角上，就可以左右转
//	h += 0.03*(y - OriY);  //鼠标在窗口y轴方向上的改变加到视点y的坐标上，就可以上下转
//	if (h>1.0)   h = 1.0;  //对视点y坐标作一些限制，不会使视点太奇怪
//	else if (h<-1.0) h = -1.0;
//	OriX = x, OriY = y;  //将此时的坐标作为旧值，为下一次计算增量做准备
//}

static void InitMesh(Mesh& mdata) {
	cout << "Calculating adjacent facets" << endl;
	mdata.CaladjFacet();//计算各顶点的邻接面片
	cout << "Calculating adjacent vertexes" << endl;
	mdata.CaladjVert();//计算各顶点的邻接点
	cout << "Calculating normal vector for vertexes" << endl;
	mdata.Calvertnorm();//计算各顶点的法向量
	CalCf(mdata);//计算各顶点的凹凸信号
				 //sort(mdata.Cf.begin(), mdata.Cf.end(), cmp); //升序
}


int main(int argc, char **argv) {
	char* orifile = "C:/Users/exspectate/Desktop/LAST.stl";
	Mesh orimData; //原始网格数据
	readASCIstl(orifile, orimData);//读取数据
	InitMesh(orimData); //计算各顶点的邻接点
	for (vector<int>::iterator it = orimData.badPointsId.begin(); it != orimData.badPointsId.end(); ++it) {
		orimData.Verts[*it].color = -1;
	}
	setColor(orimData);

	mesh = orimData;

    mesh.kind_color.insert(make_pair(1, 1));
    mesh.kind_color.insert(make_pair(2, 2));
    mesh.kind_color.insert(make_pair(3, 4));
    mesh.kind_color.insert(make_pair(4, 3));
    mesh.kind_color.insert(make_pair(5, 5));
    mesh.kind_color.insert(make_pair(6, 6));


	callbsp(mesh);


	int tempInt = 1;
	char *tempStr = "OpenGL";
	glutInit(&tempInt, &tempStr);                                          //初始化glut库
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);      //设置初始显示模式
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(1200, 600);
	glutCreateWindow("***************");
	init();
	glutReshapeFunc(reshape);       //
	glutDisplayFunc(drawSTL);       //
	glutIdleFunc(drawSTL);          //设置不断调用显示函数
	glutMouseFunc(Mouse);
	glutMotionFunc(onMouseMove);
	glutSpecialFunc(SpecialKey);
	glutMainLoop();//enters the GLUT event processing loop.
	return 0;
}