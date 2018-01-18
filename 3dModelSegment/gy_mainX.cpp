#include "lty_STL.h"
#include <glut.h>
#include <math.h>

#define  GLUT_WHEEL_UP 3           //������ֲ���  
#define  GLUT_WHEEL_DOWN 4  

static struct TY {
	double d;
	int ind;
	TY() {}
	TY(double d_, int i_): d(d_), ind(i_) {}
	bool operator < (const TY &t) {
		return d < t.d;
	}
};

static const int COLOR1[19] = { 0,0,255,25 ,0  ,0  ,85 ,139,255,0  ,255,139,139,255,139,139,255,255,255 };
static const int COLOR2[19] = { 0,0,222,25 ,255,255,107,129,0  ,0  ,255,101,58 ,69 ,137,0  ,228,250,0 };
static const int COLOR3[19] = { 0,205,173,112,127,0  ,47 ,76 ,0  ,255,0  ,8  ,58 ,0  ,137,0  ,225,205,255 };
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
		for (int i = 0; i<mesh.adjVert[tmp].size(); i++)
			if (mesh.Verts[mesh.adjVert[tmp][i]].color == 0)
				que.push(mesh.adjVert[tmp][i]);

	}
	return ret;
}

static void setColor(Mesh &mesh)
{
	int nowColor = 0;
	for (int i = 0; i<mesh.Verts.size(); i++)
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

static void SpecialKey(GLint key, GLint x, GLint y) {
	if (key == GLUT_KEY_LEFT) {
	
	}
	
	if (key == GLUT_KEY_RIGHT) {
	
	}
}

static void Mouse(int button, int state, int x, int y)
{
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

	//�Ҽ�����ƽ��
	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
			//��ס�������������
			cx = x;
			cy = y;
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
		times = 0.008f + times;
		glutPostRedisplay();
	}

	else if (state == GLUT_UP && button == GLUT_WHEEL_DOWN)
	{
		times = -0.008f + times;
		glutPostRedisplay();
	}
}

static void onMouseMove(int x, int y)   //��������϶�
{
	if (mouseLeftDown) {
		du += x - OriX; //����ڴ���x�᷽���ϵ������ӵ��ӵ���x��ļн��ϣ��Ϳ�������ת
		h += 0.03*(y - OriY);  //����ڴ���y�᷽���ϵĸı�ӵ��ӵ�y�������ϣ��Ϳ�������ת
		if (h>1.0)   h = 1.0;  //���ӵ�y������һЩ���ƣ�����ʹ�ӵ�̫���
		else if (h<-1.0) h = -1.0;
		OriX = x, OriY = y;  //����ʱ��������Ϊ��ֵ��Ϊ��һ�μ���������׼��
	}
	else if (mouseRightDown) {
		//�����϶����ƫ������Ȼ�����xy���Ӽ�
		oldx += ((x - cx) * 0.1);
		oldy -= ((y - cy) * 0.1);
		glutPostRedisplay();
		cx = x;
		cy = y;
	}

}

static void drawSTL() {
	//cout << "drawSTL";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //������壬GL_COLOR_BUFFER_BIT ����ɫ�����־λ
	glLoadIdentity();                                       //���õ�ǰ����Ϊ4*4�ĵ�λ����
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //���ӵ㿴Զ��

	//�����ҵ��Ƿ�׼ȷ, �ҵ���(posx,posy,posz)�����NN����Ⱦ����ɫ18�����׼ȷ����λ�õ���ɫ��Ϊ��ɫ18
	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {
		cout << posx << "  " << posy << "  " << posz << endl;
		//posy = -posy;
		int NN = 100;  //������
		vector<pair<double, int> > tvec(mesh.Verts.size());
		for (int i = 0; i < mesh.Verts.size(); ++i) {   
			Point3d tp = Point3d(posx- mesh.Verts[i].x, posy - mesh.Verts[i].y, posz - mesh.Verts[i].z);
			tvec[i] = make_pair(tp.len(), i);
		}
		sort(tvec.begin(), tvec.end());
		for (int i = 0; i < NN; ++i) mesh.Verts[tvec[i].second].color = 18;
		posx = posy = posz = 0;
	}

	glScalef(times, times, times);//����
	glTranslatef(oldx, oldy, 0);    //ƽ��
									//��Ⱦ��ɫ
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	for (int i = 0; i <mesh.Facets.size(); i = i + 1) {
		int nowColor = max(max(mesh.Verts[mesh.Facets[i].vertIdx[0]].color, mesh.Verts[mesh.Facets[i].vertIdx[1]].color), mesh.Verts[mesh.Facets[i].vertIdx[2]].color);
		if (nowColor == -1)
		{
			for (int j = 0; j<mesh.adjVert[mesh.Facets[i].vertIdx[0]].size(); j++)
				nowColor = max(nowColor, mesh.Verts[mesh.adjVert[mesh.Facets[i].vertIdx[0]][j]].color);
		}
		glColor3f(1.0*COLOR1[nowColor] / 255, 1.0*COLOR2[nowColor] / 255, 1.0*COLOR3[nowColor] / 255);		//��ɫ
		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; ++j) {
			glNormal3f(mesh.vertnorm[mesh.Facets[i].vertIdx[j]].x, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].y, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].z);
			glVertex3f(mesh.Verts[mesh.Facets[i].vertIdx[j]].x, mesh.Verts[mesh.Facets[i].vertIdx[j]].y, mesh.Verts[mesh.Facets[i].vertIdx[j]].z);
		}
		glEnd();
	}

	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);			//��ɫ
	for (int i = 0; i < regionCount; ++i) {
		/*for (int j = 0; j < regionE[i].size(); ++j) {
		//glColor3f(1.0, 1.0, 1.0);
		//glNormal3f(mesh.vertnorm[mesh.Edges[regionE[i][j]].a].x, mesh.vertnorm[mesh.Edges[regionE[i][j]].a].y, mesh.vertnorm[mesh.Edges[regionE[i][j]].a].z);
		glVertex3f(mesh.Verts[mesh.Edges[regionE[i][j]].a].x, mesh.Verts[mesh.Edges[regionE[i][j]].a].y, mesh.Verts[mesh.Edges[regionE[i][j]].a].z);

		//glNormal3f(mesh.vertnorm[mesh.Edges[regionE[i][j]].b].x, mesh.vertnorm[mesh.Edges[regionE[i][j]].b].y, mesh.vertnorm[mesh.Edges[regionE[i][j]].b].z);
		glVertex3f(mesh.Verts[mesh.Edges[regionE[i][j]].b].x, mesh.Verts[mesh.Edges[regionE[i][j]].b].y, mesh.Verts[mesh.Edges[regionE[i][j]].b].z);

		}*/
		//int i=5;
		for (int j = 1; j<regionPt[i].size(); j++)
		{
			glVertex3f(regionPt[i][j - 1].x, regionPt[i][j - 1].y, regionPt[i][j - 1].z);
			glVertex3f(regionPt[i][j].x, regionPt[i][j].y, regionPt[i][j].z);
		}
	}
	glEnd();
	glDisable(GL_COLOR_MATERIAL);



	glutSwapBuffers();                                      //��������������ָ��


}

static void init(void)
{
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//����LIGHT0��λ��
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //ɢ�������
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //���淴���
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBAģʽ�Ļ����⣬Ϊ0 
	glShadeModel(GL_SMOOTH);           //��ɫ����ģʽ
	glEnable(GL_LIGHTING);			   //����:ʹ�ù�
	glEnable(GL_LIGHT0);			   //��0#��
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);		   //��Ȳ���
									   //glEnable(GL_DEPTH_TEST);    //������ȣ����������Զ���Զ����ر���ס��ͼ�Σ����ϣ�
}


static void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //��ͼ;1��2Ϊ�ӿڵ����½�;3��4Ϊ�ӿڵĿ�Ⱥ͸߶�
	glMatrixMode(GL_PROJECTION);    //����ǰ����ָ��ΪͶӰ����
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 1000.0); //1����Ұ��Y-Zƽ��ĽǶ�[0,180];2��ͶӰƽ������߶ȵı���;3�����ؼ��浽�ӵ�ľ���;4��Զ�ؼ��浽�ӵ�ľ���
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
	char* orifile = "STLFile/ECR.stl";
	//char* smoothedfile = "OutFile/r1.stl";
	Mesh orimData; //ԭʼ��������
	readASCIstl(orifile, orimData);//��ȡ����
	InitMesh(orimData); //�����������ڽӵ�
	for (vector<int>::iterator it = orimData.badPointsId.begin(); it != orimData.badPointsId.end(); ++it) {
		orimData.Verts[*it].color = -1;
	}
	setColor(orimData);

	mesh = orimData;
	glutInit(&argc, argv);                                          //��ʼ��glut��
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
	glutMainLoop();//enters the GLUT event processing loop.

	return 0;
}