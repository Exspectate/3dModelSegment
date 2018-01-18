#include "lty_STL.h"
#include "dijkstra.h"
#include <glut.h>
#include <math.h>
#include <fstream>
#include "bsp.h"



#define  GLUT_WHEEL_UP 3           //������ֲ���  
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
static int du = 90, OriX = -500, OriY = -50;   //du���ӵ��x��ļн�
static float r = 1000, h = 0;   //r���ӵ���y��İ뾶��h���ӵ�߶ȼ���y���ϵ�����
static float c = PI / 180.0;    //���ȺͽǶ�ת������
static float times = 1;			//�Ŵ���С����

static float oldx = 0;					//����任������
static float oldy = 0;

static int cx = 0;					//����������
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

static int reBfs(int node, Mesh &mesh) {  //�����������ɫ���Ϊ0
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
			cout << "����:" << ret << "\n";
		}
	}

	cout << "����:" << nowColor << endl;
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

static void SpecialKey(GLint key, GLint x, GLint y) { //ͨ���������Ҽ���ƽ��ģ��
	GLfloat unit = 20.f;  //ƽ�Ƶ�λ
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
	GLfloat scaleUnit = 0.06f;   //���ų߶ȵ�λ
	//���������ת
	if (button == GLUT_LEFT_BUTTON) {
		if (state == GLUT_DOWN) {
			//��¼��갴��λ��
			OriX = x, OriY = y;
			mouseLeftDown = true;
		}
		else
			mouseLeftDown = false;
	}

	//�Ҽ���ȡ���λ�õ���ά����
	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
			//��ס����һ���������
			winx = x;
			winy = y;
			pickPoint();
		}
		else if (state == GLUT_UP)
			mouseRightDown = false;
	}
	/*
	* �����ֿ���ͼ������
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

static void onMouseMove(int x, int y)   //��������϶�
{
	GLfloat rotateXUnit = 1.0f;
	GLfloat rotateYUnit = 20.0f;
	if (mouseLeftDown) {
		du += (x - OriX)*rotateXUnit; //����ڴ���x�᷽���ϵ������ӵ��ӵ���x��ļн��ϣ��Ϳ�������ת
		h += (y - OriY)*rotateYUnit;  //����ڴ���y�᷽���ϵĸı�ӵ��ӵ�y�������ϣ��Ϳ�������ת
		//if (h>1.0)   h = 1.0;  //���ӵ�y������һЩ���ƣ�����ʹ�ӵ�̫���
	//	else if (h<-1.0) h = -1.0;
		OriX = x, OriY = y;  //����ʱ��������Ϊ��ֵ��Ϊ��һ�μ���������׼��
	}
	else if (mouseRightDown) {
		//�����϶����ƫ������Ȼ�����xy���Ӽ�
		//oldx += ((x - cx) * 1);
		//oldy -= ((y - cy) * 1);
		//glutPostRedisplay();
		//cx = x;
		//cy = y;
	}

}

static void drawSTL() {
	//cout << "drawSTL";
	glClearColor(161. / 255, 175. / 255, 201. / 255, 1.0);   //������ɫ
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //������壬GL_COLOR_BUFFER_BIT ����ɫ�����־λ
	glLoadIdentity();                                       //���õ�ǰ����Ϊ4*4�ĵ�λ����
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //���ӵ㿴Զ��

	int targetId = -1;  //��¼��ģ������ѡȡ��������id
	int targetColor;	//��¼��ǰ��������ɫ
	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {    //����������Ҽ�ʱ,��ȡ����Ӧ��ά�ռ������
		double dis = 1e30;
		for (int i = 0; i < mesh.Verts.size(); ++i) {
			if (mesh.Verts[i].color < 0) continue;  //����ǻ����ϵĵ㣬������
			Point3d tp = Point3d(posx - mesh.Verts[i].x, posy - mesh.Verts[i].y, posz - mesh.Verts[i].z);
			if (tp.len() < dis) {
				dis = tp.len();
				targetId = i;
			}
		}
		if (dis < 1e2) {      //��ѡ�в���Ⱦ�ɰ�ɫ��ʾѡ��
			targetColor = mesh.Verts[targetId].color;
			reBfs(targetId, mesh);
		}
		else { //�������˵���Ҽ������ģ���⣬���账��
			posx = posy = posz = 0;
		}
	}

	glScalef(times, times, times);//����
	glTranslatef(oldx, oldy, 0);    //ƽ��
									//��Ⱦ��ɫ
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

		glColor3f(1.0*COLOR1[nowColor] / 255, 1.0*COLOR2[nowColor] / 255, 1.0*COLOR3[nowColor] / 255);		//��ɫ
		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; ++j) {
			glNormal3f(mesh.vertnorm[mesh.Facets[i].vertIdx[j]].x, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].y, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].z);
			glVertex3f(mesh.Verts[mesh.Facets[i].vertIdx[j]].x, mesh.Verts[mesh.Facets[i].vertIdx[j]].y, mesh.Verts[mesh.Facets[i].vertIdx[j]].z);
		}
		glEnd();
	}

	////F1����
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 0.0);			//��ɫ
	//glVertex3d(getf1().L.x, getf1().L.y, getf1().L.z);
	//glVertex3d(getf1().R.x, getf1().R.y, getf1().R.z);
	//glEnd();


    ////F1Ȧ������
    //glLineWidth(5.0f);
    //glBegin(GL_LINES); 
    //glColor3f(1.0, 0.0, 0.0);			//��ɫ
    //for (int i = 0; i < getf1().UpCurve.size(); ++i) {

    //    Point3d &a = getf1().UpCurve[i].first;
    //	Point3d &b = getf1().UpCurve[i].second;
    //	glVertex3d(a.x, a.y, a.z);
    //	glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

 //   //F1Ȧ������
 //   glLineWidth(5.0f);
 //   glBegin(GL_LINES);
 //   glColor3f(0.0, 1.0, 0.0);			//��ɫ
 //   for (int i = 0; i < getf1().DownCurve.size(); ++i) {

 //       Point3d &a = getf1().DownCurve[i].first;
 //       Point3d &b = getf1().DownCurve[i].second;
 //       glVertex3d(a.x, a.y, a.z);
 //       glVertex3d(b.x, b.y, b.z);
 //   }
 //   glEnd();

	////F1Ȧ(F2Ȧ�Ѿ�����)
	//glLineWidth(2.0f);
	//glBegin(GL_LINES); 
	//glColor3f(1.0, 0.0, 0.0);			//��ɫ
	//for (int i = 0; i < getf1().inbadEdge.size(); ++i) {

	//	Point3d &a = mesh.Verts[getf1().inbadEdge[i].first];
	//	Point3d &b = mesh.Verts[getf1().inbadEdge[i].second];
	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();

	////F1ǰ����
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 0.0);			//��ɫ
	//for (int i = 0; i < getf1().Curve.size()-1; ++i) {

	//	Point3d &a = getf1().Curve[i];
	//	Point3d &b = getf1().Curve[i+1];

	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);

	//}
	//glEnd();

	////F2����
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 1.0);			//��ɫ
	//glVertex3d(getf2().L.x, getf2().L.y, getf2().L.z);
	//glVertex3d(getf2().R.x, getf2().R.y, getf2().R.z);
	//glEnd();

	////F2�߽�
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 1.0, 1.0);			//��ɫ
	//glVertex3d(getf1().L.x, getf1().L.y, getf1().L.z);
	//glVertex3d(getf2().L.x, getf2().L.y, getf2().L.z);

	//glVertex3d(getf1().R.x, getf1().R.y, getf1().R.z);
	//glVertex3d(getf2().R.x, getf2().R.y, getf2().R.z);

	//glEnd();

	////F2Ȧ(ǰ���������ǣ���һ��F1Ȧ�ӽ��غ�)
	//glLineWidth(1.0f);
	//glBegin(GL_LINES);
	//glColor3f(0.0, 0.0, 1.0);			//��ɫ
	//for (int i = 0; i < getf2().inbadEdge.size(); ++i) {

	//	Point3d &a = mesh.Verts[getf2().inbadEdge[i].first];
	//	Point3d &b = mesh.Verts[getf2().inbadEdge[i].second];

	//	glVertex3f(a.x, a.y, a.z);
	//	glVertex3f(b.x, b.y, b.z);

	//}
	//glEnd();


    ////F2Ȧ������
    //glLineWidth(5.0f);
    //glBegin(GL_LINES);
    //glColor3f(1.0, 0.0, 0.0);			//��ɫ
    //for (int i = 0; i < getf2().UpCurve.size(); ++i) {

    //    Point3d &a = getf2().UpCurve[i].first;
    //    Point3d &b = getf2().UpCurve[i].second;
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

    ////F2Ȧ������
    //glLineWidth(5.0f);
    //glBegin(GL_LINES);
    //glColor3f(0.0, 1.0, 0.0);			//��ɫ
    //for (int i = 0; i < getf2().DownCurve.size(); ++i) {

    //    Point3d &a = getf2().DownCurve[i].first;
    //    Point3d &b = getf2().DownCurve[i].second;
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

	////F3����
	//glLineWidth(8.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 1.0);			//��ɫ
	//glVertex3d(getf3().L.x, getf3().L.y, getf3().L.z);
	//glVertex3d(getf3().R.x, getf3().R.y, getf3().R.z);
	//glEnd();

	////F3ͻ����
	//glLineWidth(2.0f);
	//glBegin(GL_LINE_LOOP);
	//glColor3f(0.0, 0.0, 1.0);			//��ɫ
	//for (int i = 0; i < getf3().testpiiv.size(); ++i) {
	//	Point3d &a = mesh.Verts[getf3().testpiiv[i].first];
	//	Point3d &b = mesh.Verts[getf3().testpiiv[i].second];
	//	glVertex3d(a.x, a.y, a.z);
	//	glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();


    ////F3ͻ�����ϵĵ�
    //glPointSize(1);
    //glBegin(GL_POINTS);
    //glColor3f(0.0, 1.0, 0.0);			//��ɫ
    //for (int i = 0; i < getf3().RidgeLine.size(); ++i) {
    //    Point3d &a = mesh.Verts[getf3().RidgeLine[i].first];
    //    Point3d &b = mesh.Verts[getf3().RidgeLine[i].second];
    //    glVertex3d(a.x, a.y, a.z);
    //    glVertex3d(b.x, b.y, b.z);
    //}
    //glEnd();

	////F3ͻ����DOWN�ϵĵ�
	//glPointSize(1);
	//glBegin(GL_POINTS);
	//glColor3f(0.0, 1.0, 0.0);			//��ɫ
	//for (int i = 0; i < getf3().DownCurve.size(); ++i) {
	//    Point3d &a = mesh.Verts[getf3().RidgeLine[i].first];
	//    Point3d &b = mesh.Verts[getf3().RidgeLine[i].second];
	//    glVertex3d(a.x, a.y, a.z);
	//    glVertex3d(b.x, b.y, b.z);
	//}
	//glEnd();


	//F3������ĵ�
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 1.0);			//��ɫ
	
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
	glColor3f(0.0, 0.0, 1.0);			//��ɫ
	glVertex3d(getf3().Pnear2.x, getf3().Pnear2.y, getf3().Pnear2.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 1.0, 0.0);			//greenɫ
	glVertex3d(getf3().Pnear3.x, getf3().Pnear3.y, getf3().Pnear3.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(1.0, 0.0, 0.0);			//redɫ
	glVertex3d(getf3().Pnear4.x, getf3().Pnear4.y, getf3().Pnear4.z);
	glEnd();
	/*************************************************************/
	glPointSize(7);
	glBegin(GL_POINTS);
	glColor3f(0.0, 0.0, 0.0);			//whiteɫ
	glVertex3d(getf3().Pnear5.x, getf3().Pnear5.y, getf3().Pnear5.z);
	glEnd();


	////F3�߽���
	//glLineWidth(8.0f);
	//glBegin(GL_LINES);
	//glColor3f(1.0, 0.0, 0.0);			//��ɫ
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



	glutSwapBuffers();                                      //��������������ָ��

	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {   //�����Ҽ�����
		system("pause");


		//ofstream ofile;  //���ѡ�еĵ�
		//ofile.open("temp.txt");
		//for (int i = 0; i < mesh.Verts.size(); ++i) {
		//	if (mesh.Verts[i].color != 0) continue;
		//	ofile << mesh.Verts[i].x << " " << mesh.Verts[i].y << " " << mesh.Verts[i].z << "\n";
		//}
		//ofile.close();

		//bfs(targetId, targetColor, mesh); //�ڱ����ɺ󣬽�ѡ�в���Ⱦ��ԭ������ɫ
		//posx = posy = posz = 0;
	}
}

static void init(void)
{
	GLfloat light_position[] = { 0, 0, 0, 0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	glEnable(GL_LIGHT0);			   //��0#��
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//����LIGHT0��λ��
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //ɢ�������
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //���淴���
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBAģʽ�Ļ����⣬Ϊ0 

	GLfloat light_position1[] = { 0, 0, 200.0, 0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	GLfloat white_light1[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	GLfloat Light_Model_Ambient1[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	glEnable(GL_LIGHT1);			   //��1#��
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);	//����LIGHT1��λ��
	glLightfv(GL_LIGHT1, GL_DIFFUSE, white_light1);      //ɢ�������
	glLightfv(GL_LIGHT1, GL_SPECULAR, white_light1);     //���淴���
	glLightfv(GL_LIGHT1, GL_AMBIENT, Light_Model_Ambient1);//RGBAģʽ�Ļ����⣬Ϊ0 

	GLfloat light_position2[] = { 2000, 0, 0, 0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	GLfloat white_light2[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	GLfloat Light_Model_Ambient2[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	glEnable(GL_LIGHT2);			   //��2#��
	glLightfv(GL_LIGHT2, GL_POSITION, light_position2);	//����LIGHT2��λ��
	glLightfv(GL_LIGHT2, GL_DIFFUSE, white_light2);      //ɢ�������
	glLightfv(GL_LIGHT2, GL_SPECULAR, white_light2);     //���淴���
	glLightfv(GL_LIGHT2, GL_AMBIENT, Light_Model_Ambient2);//RGBAģʽ�Ļ����⣬Ϊ0 

	//GLfloat light_position3[] = { 2000, 0, 200, 0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	//GLfloat white_light3[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	//GLfloat Light_Model_Ambient3[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	//glEnable(GL_LIGHT3);			   //��0#��
	//glLightfv(GL_LIGHT3, GL_POSITION, light_position3);	//����LIGHT0��λ��
	//glLightfv(GL_LIGHT3, GL_DIFFUSE, white_light3);      //ɢ�������
	//glLightfv(GL_LIGHT3, GL_SPECULAR, white_light3);     //���淴���
	//glLightfv(GL_LIGHT3, GL_AMBIENT, Light_Model_Ambient3);//RGBAģʽ�Ļ����⣬Ϊ0 

	glEnable(GL_LIGHTING);			   //����:ʹ�ù�
	glShadeModel(GL_SMOOTH);           //��ɫ����ģʽ
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);		   //��Ȳ���
									   //glEnable(GL_DEPTH_TEST);    //������ȣ����������Զ���Զ����ر���ס��ͼ�Σ����ϣ�
}

static void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //��ͼ;1��2Ϊ�ӿڵ����½�;3��4Ϊ�ӿڵĿ�Ⱥ͸߶�
	glMatrixMode(GL_PROJECTION);    //����ǰ����ָ��ΪͶӰ����
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 4000.0); //1����Ұ��Y-Zƽ��ĽǶ�[0,180];2��ͶӰƽ������߶ȵı���;3�����ؼ��浽�ӵ�ľ���;4��Զ�ؼ��浽�ӵ�ľ���
	glMatrixMode(GL_MODELVIEW);     //��ģ���Ӿ������ջӦ�����ľ������.
}

//static void Mouse(int button, int state, int x, int y)
//{
//	if (state == GLUT_DOWN)  //��¼��갴��λ��
//		OriX = x, OriY = y;
//}

//static void onMouseMove(int x, int y)   //��������϶�
//{
//	du += x - OriX; //����ڴ���x�᷽���ϵ������ӵ��ӵ���x��ļн��ϣ��Ϳ�������ת
//	h += 0.03*(y - OriY);  //����ڴ���y�᷽���ϵĸı�ӵ��ӵ�y�������ϣ��Ϳ�������ת
//	if (h>1.0)   h = 1.0;  //���ӵ�y������һЩ���ƣ�����ʹ�ӵ�̫���
//	else if (h<-1.0) h = -1.0;
//	OriX = x, OriY = y;  //����ʱ��������Ϊ��ֵ��Ϊ��һ�μ���������׼��
//}

static void InitMesh(Mesh& mdata) {
	cout << "Calculating adjacent facets" << endl;
	mdata.CaladjFacet();//�����������ڽ���Ƭ
	cout << "Calculating adjacent vertexes" << endl;
	mdata.CaladjVert();//�����������ڽӵ�
	cout << "Calculating normal vector for vertexes" << endl;
	mdata.Calvertnorm();//���������ķ�����
	CalCf(mdata);//���������İ�͹�ź�
				 //sort(mdata.Cf.begin(), mdata.Cf.end(), cmp); //����
}


int main(int argc, char **argv) {
	char* orifile = "C:/Users/exspectate/Desktop/LAST.stl";
	Mesh orimData; //ԭʼ��������
	readASCIstl(orifile, orimData);//��ȡ����
	InitMesh(orimData); //�����������ڽӵ�
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
	glutInit(&tempInt, &tempStr);                                          //��ʼ��glut��
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);      //���ó�ʼ��ʾģʽ
	glutInitWindowPosition(50, 50);
	glutInitWindowSize(1200, 600);
	glutCreateWindow("***************");
	init();
	glutReshapeFunc(reshape);       //
	glutDisplayFunc(drawSTL);       //
	glutIdleFunc(drawSTL);          //���ò��ϵ�����ʾ����
	glutMouseFunc(Mouse);
	glutMotionFunc(onMouseMove);
	glutSpecialFunc(SpecialKey);
	glutMainLoop();//enters the GLUT event processing loop.
	return 0;
}