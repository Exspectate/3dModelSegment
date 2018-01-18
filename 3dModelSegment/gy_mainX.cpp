#include "lty_STL.h"
#include <glut.h>
#include <math.h>

#define  GLUT_WHEEL_UP 3           //定义滚轮操作  
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

static void SpecialKey(GLint key, GLint x, GLint y) {
	if (key == GLUT_KEY_LEFT) {
	
	}
	
	if (key == GLUT_KEY_RIGHT) {
	
	}
}

static void Mouse(int button, int state, int x, int y)
{
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

	//右键控制平移
	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouseRightDown = true;
			//记住鼠标点击后光标坐标
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
	* 鼠标滚轮控制图形缩放
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

static void onMouseMove(int x, int y)   //处理鼠标拖动
{
	if (mouseLeftDown) {
		du += x - OriX; //鼠标在窗口x轴方向上的增量加到视点与x轴的夹角上，就可以左右转
		h += 0.03*(y - OriY);  //鼠标在窗口y轴方向上的改变加到视点y的坐标上，就可以上下转
		if (h>1.0)   h = 1.0;  //对视点y坐标作一些限制，不会使视点太奇怪
		else if (h<-1.0) h = -1.0;
		OriX = x, OriY = y;  //将此时的坐标作为旧值，为下一次计算增量做准备
	}
	else if (mouseRightDown) {
		//计算拖动后的偏移量，然后进行xy叠加减
		oldx += ((x - cx) * 0.1);
		oldy -= ((y - cy) * 0.1);
		glutPostRedisplay();
		cx = x;
		cy = y;
	}

}

static void drawSTL() {
	//cout << "drawSTL";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除缓冲，GL_COLOR_BUFFER_BIT ：颜色缓冲标志位
	glLoadIdentity();                                       //重置当前矩阵为4*4的单位矩阵
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //从视点看远点

	//测试找点是否准确, 找到与(posx,posy,posz)最近的NN个点染成颜色18，如果准确则点击位置的颜色置为颜色18
	if (sign(posx) != 0 || sign(posy) != 0 || sign(posz) != 0) {
		cout << posx << "  " << posy << "  " << posz << endl;
		//posy = -posy;
		int NN = 100;  //近邻数
		vector<pair<double, int> > tvec(mesh.Verts.size());
		for (int i = 0; i < mesh.Verts.size(); ++i) {   
			Point3d tp = Point3d(posx- mesh.Verts[i].x, posy - mesh.Verts[i].y, posz - mesh.Verts[i].z);
			tvec[i] = make_pair(tp.len(), i);
		}
		sort(tvec.begin(), tvec.end());
		for (int i = 0; i < NN; ++i) mesh.Verts[tvec[i].second].color = 18;
		posx = posy = posz = 0;
	}

	glScalef(times, times, times);//缩放
	glTranslatef(oldx, oldy, 0);    //平移
									//渲染颜色
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	for (int i = 0; i <mesh.Facets.size(); i = i + 1) {
		int nowColor = max(max(mesh.Verts[mesh.Facets[i].vertIdx[0]].color, mesh.Verts[mesh.Facets[i].vertIdx[1]].color), mesh.Verts[mesh.Facets[i].vertIdx[2]].color);
		if (nowColor == -1)
		{
			for (int j = 0; j<mesh.adjVert[mesh.Facets[i].vertIdx[0]].size(); j++)
				nowColor = max(nowColor, mesh.Verts[mesh.adjVert[mesh.Facets[i].vertIdx[0]][j]].color);
		}
		glColor3f(1.0*COLOR1[nowColor] / 255, 1.0*COLOR2[nowColor] / 255, 1.0*COLOR3[nowColor] / 255);		//颜色
		glBegin(GL_TRIANGLES);
		for (int j = 0; j < 3; ++j) {
			glNormal3f(mesh.vertnorm[mesh.Facets[i].vertIdx[j]].x, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].y, mesh.vertnorm[mesh.Facets[i].vertIdx[j]].z);
			glVertex3f(mesh.Verts[mesh.Facets[i].vertIdx[j]].x, mesh.Verts[mesh.Facets[i].vertIdx[j]].y, mesh.Verts[mesh.Facets[i].vertIdx[j]].z);
		}
		glEnd();
	}

	glBegin(GL_LINES);
	glColor3f(1.0, 0.0, 0.0);			//红色
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



	glutSwapBuffers();                                      //交换两个缓冲区指针


}

static void init(void)
{
	GLfloat light_position[] = { 1.0, 1.0, 1.0, 0.0 };	//光源位置 最后一位 0-方向光源、1-点光源
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//设置LIGHT0的位置
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //散射光属性
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //镜面反射光
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBA模式的环境光，为0 
	glShadeModel(GL_SMOOTH);           //颜色过度模式
	glEnable(GL_LIGHTING);			   //开关:使用光
	glEnable(GL_LIGHT0);			   //打开0#灯
	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);		   //深度测试
									   //glEnable(GL_DEPTH_TEST);    //启用深度，根据坐标的远近自动隐藏被遮住的图形（材料）
}


static void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //截图;1、2为视口的左下角;3、4为视口的宽度和高度
	glMatrixMode(GL_PROJECTION);    //将当前矩阵指定为投影矩阵
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 1000.0); //1、视野在Y-Z平面的角度[0,180];2、投影平面宽度与高度的比率;3、近截剪面到视点的距离;4、远截剪面到视点的距离
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
	char* orifile = "STLFile/ECR.stl";
	//char* smoothedfile = "OutFile/r1.stl";
	Mesh orimData; //原始网格数据
	readASCIstl(orifile, orimData);//读取数据
	InitMesh(orimData); //计算各顶点的邻接点
	for (vector<int>::iterator it = orimData.badPointsId.begin(); it != orimData.badPointsId.end(); ++it) {
		orimData.Verts[*it].color = -1;
	}
	setColor(orimData);

	mesh = orimData;
	glutInit(&argc, argv);                                          //初始化glut库
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
	glutMainLoop();//enters the GLUT event processing loop.

	return 0;
}