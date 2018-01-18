#include <stack>  
#include "lty_STL.h"
#include <glut.h>
#define MAX 20
#include <math.h>
#define PI 3.1415926535
#define E 2.7182818284
#pragma comment (lib, "glew32.lib")  

#pragma warning(disable:4996)
using namespace std;


const int COLOR1[19]={0,122,255,25 ,0  ,0  ,85 ,139,255,0  ,255,139,139,255,139,139,255,255,255};
const int COLOR2[19]={0,197,222,25 ,255,255,107,129,0  ,0  ,255,101,58 ,69 ,137,0  ,228,250,0};
const int COLOR3[19]={0,205,173,112,127,0  ,47 ,76 ,0  ,255,0  ,8  ,58 ,0  ,137,0  ,225,205,255};

static int du = 90, OriX = -1, OriY = -1;   //du是视点和x轴的夹角
static float r = 1.5, h = 0.0;   //r是视点绕y轴的半径，h是视点高度即在y轴上的坐标
static float c = PI / 180.0;    //弧度和角度转换参数
const int rotateTime = 12;
#pragma warning(disable:4996)
using namespace std;

vector<vector<int>> regionE(MAX);
int regionCount;
Dijkstra dija;
vector<vector<Point3d> >regionPt;
void InitMesh(Mesh& mdata) {
	cout << "Calculating adjacent facets" << endl;
	mdata.CaladjFacet();//计算各顶点的邻接面片
	cout << "Calculating adjacent vertexes" << endl;
	mdata.CaladjVert();//计算各顶点的邻接点
	cout << "Calculating normal vector for vertexes" << endl;
	mdata.Calvertnorm();//计算各顶点的法向量
	CalCf(mdata);//计算各顶点的凹凸信号
	//sort(mdata.Cf.begin(), mdata.Cf.end(), cmp); //升序
}

void LaplaceSmooth(char* wfile, Mesh &mdata, Mesh &ndata) {
	cout << "1.Laplace smooth" << endl;

	//2. Laplace平滑-------------------
	cout << "Calculating Laplace matrix" << endl;
	SparseMatrix<double> L = CalLapMatrix(mdata);//计算网格模型的拉普拉斯矩阵
	cout << "Calculating Cf" << endl;
	vector<VtCf> Cf1;
	Cf1.assign(mdata.Cf.begin(), mdata.Cf.end());
	//顶点凹凸信号排序
	cout << "Sorting Cf" << endl;

	sort(Cf1.begin(), Cf1.end(), cmp); //升序

									   //选取30%低频控制顶点
	cout << "Selecting control vertexes" << endl;
	vector<int> ControlVerts;//控制顶点索引集
	double ratio_control = 0.30;//控制顶点比例
	int CPnum = (int)(ratio_control*(Cf1.size()));
	//cout << CPnum << endl;
	int VTnum = mdata.Verts.size();
	//cout << VTnum << endl;
	for (int i = 0; i < CPnum; i++) {
		ControlVerts.push_back(Cf1[i].idx);
	}

	//解方程组
	double c = 0.01; //控制权重

					 //初始化稀疏矩阵A 
	cout << "Initializing matrices" << endl;
	vector<T> triplets;
	for (int k = 0; k < L.outerSize(); ++k) {
		for (SparseMatrix<double>::InnerIterator it(L, k); it; ++it)
		{
			if (it.value() != 0) {
				triplets.push_back(T(it.row(), it.col(), it.value()));
			}
		}
	}

	for (int i = VTnum; i < CPnum + VTnum; i++) {
		int j = i - VTnum;
		if (j >= 0 && j<VTnum)
			triplets.push_back(T(i, j, c));
		j = ControlVerts[i - VTnum];
		if (j >= 0 && j<VTnum)
			triplets.push_back(T(i, j, 1));
	}
	SparseMatrix<double> A(VTnum + CPnum, VTnum);
	A.setFromTriplets(triplets.begin(), triplets.end());
	VectorXd bx(CPnum + VTnum);
	VectorXd by(CPnum + VTnum);
	VectorXd bz(CPnum + VTnum);
	bx.head(VTnum).setZero();
	by.head(VTnum).setZero();
	bz.head(VTnum).setZero();
	for (int i = VTnum; i < CPnum + VTnum; i++) {
		bx[i] = mdata.Verts[ControlVerts[i - VTnum]].x;
		by[i] = mdata.Verts[ControlVerts[i - VTnum]].y;
		bz[i] = mdata.Verts[ControlVerts[i - VTnum]].z;
	}
	SparseMatrix<double> B = A.adjoint()*A;
	//B.makeCompressed();
	Eigen::SimplicialCholesky<SparseMatrix<double> > * chol = new Eigen::SimplicialCholesky<SparseMatrix<double> >(B);
	VectorXd VX(CPnum + VTnum);
	VectorXd VY(CPnum + VTnum);
	VectorXd VZ(CPnum + VTnum);
	cout << "Solving matrices equation" << endl;
	VX = chol->solve(A.adjoint()*bx);
	VY = chol->solve(A.adjoint()*by);
	VZ = chol->solve(A.adjoint()*bz);

	//--------------------------------------------------------------------------------------------------------------------
	//cout << VX << endl; cout << VY<<endl; cout << VZ<<endl;

	ndata.Verts.assign(mdata.Verts.begin(), mdata.Verts.end());
	ndata.Facets.assign(mdata.Facets.begin(), mdata.Facets.end());
	ndata.adjFacet.assign(mdata.adjFacet.begin(), mdata.adjFacet.end());
	ndata.adjVert.assign(mdata.adjVert.begin(), mdata.adjVert.end());

	//写入光顺后的顶点坐标
	cout << "Saving vertexes coordinate after smoothing" << endl;
	for (int i = 0; i < VTnum; i++) {
		ndata.Verts[i].x = VX[i];
		ndata.Verts[i].y = VY[i];
		ndata.Verts[i].z = VZ[i];
	}
	//计算光顺后各三角面片的法向量
	cout << "Re-calculating normal vectors of facets" << endl;
	ndata.Calfacetnorm();
	ndata.Calvertnorm();
	CalCf(ndata);
	//将光顺后的顶点坐标和面法向量写入新的stl文件
	cout << "Writing to new STL" << endl;

	writeASCIstl(wfile, ndata.Verts, ndata.Facets);
	//writeASCIstltest(wfile, ndata.Verts);

	cout << "1.Finish" << endl;
	return;
}

void concaveRegion(Mesh &oridata, Mesh &smdata) {
	cout << "2.concaveRegion" << endl;

	//3.1 特征区域粗检测       (在smdata上进行)
	cout << "特征区域粗检测" << endl;
	double ave = 0, var = 0;
	double threshold1 = 2.5;  //粗检测阈值  1.6-2.6
	vector<int> concaveVerts; //凹特征点索引集

	vector<int> concavesign(oridata.Verts.size(), -1);//凹特征区域点标记
													  //计算凹凸信号均值
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		ave += smdata.Cf[i].Cf;
	}
	ave /= smdata.Verts.size();
	//计算凹凸信号方差
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		var += (smdata.Cf[i].Cf - ave) * (smdata.Cf[i].Cf - ave);
	}
	var /= smdata.Verts.size();
	var = sqrt(var);
	cout << "均值=" << ave << endl;
	cout << "标准差=" << var << endl;

	//计算标准化凹凸信号并筛选凹特征点
	//for (size_t i = 0; i < smdata.Verts.size(); i++) {
	//	if ((smdata.Cf[i].Cf - ave) / var > threshold1) {
	//		concaveVerts.push_back(smdata.Cf[i].idx);
	//		concavesign[smdata.Cf[i].idx] = -2;  //标记
	//	}
	//}
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		if ((smdata.Verts[i].Cf - ave) / var > threshold1) {
			concaveVerts.push_back(i);
			concavesign[i] = -2;  //标记
		}
	}
	//cout << concaveVerts.size() << endl;

	//vector<Point3d> testVerts;

	//	for (size_t j = 0; j < concaveVerts.size(); j++) {
	//		testVerts.push_back(oridata.Verts[concaveVerts[j]]);
	//	}

	//writeASCIstltest("OutFile/testconcave.stl", testVerts);
	//exit(0);


	//3.2 凹特征区域精化   (在oridata上进行)  
	//用区域生长法形成若干凹特征点子区域
	cout << "特征区域精化" << endl;
	vector<int> unprocessed;  //待分配点集
	int regionNum = 0;
	vector<vector<int>> regionVerts;//各区域内点
	vector<vector<VtCf>> regionCf;
	unprocessed.assign(concaveVerts.begin(), concaveVerts.end());
	while (!unprocessed.empty()) {  //每循环一次处理一个区域
		vector<int> thisregion;
		vector<VtCf> thisregionCf;
		stack<int> s;
		int idx = *(unprocessed.begin());//取当前的第一个待分配点
		thisregion.push_back(idx);//将其加入该区域点集
		concavesign[idx] = regionNum;//记录该点所属区域
	//	thisregionCf.push_back(VtCf(oridata.Cf[idx].Cf,idx));
	  thisregionCf.push_back(VtCf(oridata.Verts[idx].Cf, idx));
		delvec(unprocessed, idx);//将其从待分配点集中删除
		do {
			if (oridata.adjVert[idx].size() != 0) {//将该点邻接点入栈，并加入该区域点集，并从待分配点集中删除
				for (size_t i = 0; i < oridata.adjVert[idx].size(); i++) {
					if (concavesign[oridata.adjVert[idx][i]] == -1) { continue; } //排除非凹区域点
					if (concavesign[oridata.adjVert[idx][i]] != -2) { continue; } //排除加入的点
					thisregion.push_back(oridata.adjVert[idx][i]);
					concavesign[oridata.adjVert[idx][i]] = regionNum;//记录该点所
		// thisregionCf.push_back(VtCf(oridata.Cf[oridata.adjVert[idx][i]].Cf, oridata.adjVert[idx][i]));
			   thisregionCf.push_back(VtCf(oridata.Verts[oridata.adjVert[idx][i]].Cf, oridata.adjVert[idx][i]));
					delvec(unprocessed, oridata.adjVert[idx][i]);
					s.push(oridata.adjVert[idx][i]);
				}
			}
			//弹出栈顶的点,对该点重复步骤
			if (!s.empty()) {
				idx = s.top();
				s.pop();
			}
		} while (!s.empty());
		if (thisregion.size() > 1) {   //如果该区域只有一个点，则删除该区域
			regionVerts.push_back(thisregion);//保存该区域的点
			regionCf.push_back(thisregionCf);//保存该区域的点
			regionNum++;
		}
		else concavesign[idx] = -1;
		thisregion.clear();
		thisregionCf.clear();
	}
	cout << "regionNum:" << regionVerts.size() << endl;

	/*for (size_t i = 0; i < concavesign.size(); i++) {
	if(concavesign[i]>0)
	cout << concavesign[i] << endl;
	}
	exit(0);*/

	//对每个凹特征点子区域进行处理
	double threshold2 = 0.5;               //精检测阈值
	int regionNumMinus = 0;
	for (size_t i = 0; i < regionCf.size(); i++) {
		sort(regionCf[i].begin(), regionCf[i].end(), cmp);
		double Cmin = regionCf[i].front().Cf;
		double Cmax = regionCf[i].back().Cf;
		//归一化得到N(Cf)
		for (size_t j = 0; j < regionCf[i].size(); j++) {
			double NCf = (regionCf[i][j].Cf - Cmin) / (Cmax - Cmin);
			//去除大于精检测阈值的点
			//if (NCf >= threshold2) {
			//	//delvec(concaveVerts, regionCf[i][j].idx);
			//	delvec(regionVerts[i], regionCf[i][j].idx);
			//	concavesign[regionCf[i][j].idx] = -1;
			//}
		}
		//如果精检测后某凹区域只有1个点或0个点，则删除该区域
		if (regionVerts[i].size() <= 1) {
			cout << "删除了区域" << i << endl;
			//if(regionVerts[i].size()==1) concavesign[regionVerts[i][0]] = -1;
			for (size_t j = 0; j < regionVerts[i].size(); j++) {
				concavesign[regionVerts[i][j]] = -1;
			}
			regionVerts.erase(regionVerts.begin() + i);
			regionCf.erase(regionCf.begin() + i);
			i--;
			regionNum--;
			regionNumMinus = 1;
		}
		//如果有区域被删除，则之后区域的点区域标记减1
		if (regionNumMinus == 1) {
			for (size_t j = i + 1; j < regionVerts.size(); j++) {
				for (size_t k = 0; k < regionVerts[j].size(); k++) {
					concavesign[regionVerts[j][k]]--;
				}
			}
			regionNumMinus = 0;
		}
	}
	cout << "regionNum:" << regionVerts.size() << endl;
	/*for (int i = 0; i < oridata.Verts.size(); i++) {
	if (concavesign[i] != -1)
	cout << concavesign[i] << endl;
	}*/
	//修改
	//test 3---------------------

	vector<Point3d> testVerts;

	for (size_t i = 0; i < regionVerts.size(); i++) {
		for (size_t j = 0; j < regionVerts[i].size(); j++) {
			testVerts.push_back(oridata.Verts[regionVerts[i][j]]);
		}
	}
	writeASCIstltest("OutFile/testconcave.stl", testVerts);
	cout << "3.Finish" << endl;

	//----------------------------

	//精检测后得到的每个凹特征区域至少包含2个点，且这2个点之间一定有一条边

	//4. 特征点区域细化

	cout << "4.特征点区域细化" << endl;
	cout << "计算网格边集" << endl;
	//计算网格边集
	int eidx = 0;
	for (int i = 0; i < oridata.Verts.size(); i++) {
		for (int j = 0; j < oridata.adjVert[i].size(); j++) {
			if (oridata.adjVert[i][j] <= i) continue;
			oridata.Edges.push_back(Edge(i, oridata.adjVert[i][j], eidx));
			eidx++;
		}
	}
	cout << "edgenum:" << eidx + 1 << endl;

	vector<int> concavesigne(oridata.Edges.size(), -1);//凹特征区域边标记


	cout << "计算所有边的邻接面片" << endl;
	// 计算所有边的邻接面片，并向邻接面片中添加该边的信息
	for (int i = 0; i < oridata.Edges.size(); i++) {
		adjFacetForEdge(oridata.Edges[i], oridata);
	}

	vector<vector<int>> regionEdge(regionNum);
	//对每个凹区域
	cout << "细化每个凹区域..." << endl;
	cout << "区域数" << regionNum << endl;

	for (int i = 0; i < regionNum; i++) {
		regionVerts[i].clear();
		for (size_t j = 0; j < oridata.Edges.size(); j++) {
			if (concavesign[oridata.Edges[j].a] == i && concavesign[oridata.Edges[j].b] == i)
			{
				regionEdge[i].push_back(oridata.Edges[j].idx);//向该凹区域添加一条边
				concavesigne[oridata.Edges[j].idx] = i;
			}
		}
		//cout << regionEdge[i].size() << endl;
		list<int> edgeList;
		for (int j = 0; j < regionEdge[i].size(); j++) {
			int eidx = regionEdge[i][j];
			if (isBoundaryEdge(oridata.Edges[eidx], oridata, concavesigne, i)) {
				edgeList.push_back(eidx);
			}
		}
		//cout << edgeList.size() << endl;
		while (!edgeList.empty()) {
			//cout << "删除！" << endl;
			int e = edgeList.front();
			edgeList.pop_front();
			if (!belongsToMeshFeature(oridata.Edges[e], oridata, regionEdge[i])) {
				addnewboundEdges(edgeList, oridata.Edges[e], oridata, concavesigne, i);
				delvec(regionEdge[i], e);
				concavesigne[e] = -1;
			}
		}
		edgeList.clear();
		//cout << regionEdge[i].size() << endl;
		//已得到每个区域内的细化后的边。现在修改regionVerts[i]为细化线上的点


		for (size_t k = 0; k < regionEdge[i].size(); k++) {
			if (find(regionVerts[i].begin(), regionVerts[i].end(), oridata.Edges[regionEdge[i][k]].a) == regionVerts[i].end())
				regionVerts[i].push_back(oridata.Edges[regionEdge[i][k]].a);
			if (find(regionVerts[i].begin(), regionVerts[i].end(), oridata.Edges[regionEdge[i][k]].b) == regionVerts[i].end())
				regionVerts[i].push_back(oridata.Edges[regionEdge[i][k]].b);
		}
		//--------将特征区域线储存
		for (size_t k = 0; k < regionEdge[i].size(); k++) {
			regionE[i].push_back(regionEdge[i][k]);
		}
	}
	regionCount = regionNum;
	//test 4---------------------
	cout << "test 4 写入" << endl;
	testVerts.clear();

	for (size_t i = 0; i < regionVerts.size(); i++) {
		for (size_t j = 0; j < regionVerts[i].size(); j++) {
			testVerts.push_back(oridata.Verts[regionVerts[i][j]]);
		}
	}
	writeASCIstltest("OutFile/testrefine.stl", testVerts);
	//----------------------------

	cout << "4.Finish" << endl;
}


//-----------------------step5-----------------------

Point3d getBarycenterPoint(vector<Point3d> &points)
{
	double sumx=0;
	double sumy=0;
	double sumz=0;
	for(int i=0;i<points.size();i++)
	{
		sumx+=points[i].x;
		sumy+=points[i].y;
		sumz+=points[i].z;
	}
	int pointNum = points.size();
	if(pointNum==0)
		pointNum++;
	sumx/=pointNum;
	sumy/=pointNum;
	sumz/=pointNum;
	return Point3d(sumx,sumy,sumz);
}

void addIntersection(Point3d pointa,Point3d barycenterPoint,Mesh &mesh,vector<int> &intersections,set<int> ps)
{
	for(int i=0;i<mesh.Facets.size();i++)
	{
		int t0 = mesh.Facets[i].vertIdx[0];
		int t1 = mesh.Facets[i].vertIdx[1];
		int t2 = mesh.Facets[i].vertIdx[2];
		if(rayByTri(mesh.Verts[t0],mesh.Verts[t1],mesh.Verts[t2],pointa,barycenterPoint)&&!ps.count(t0))
			intersections.push_back(t0);
	}
}

int findBestIntersection(vector<int> &intersections,Point3d barycenterPoint,Mesh &mesh)
{
	int nowPos = 0;
	double nowValue = (mesh.Verts[intersections[0]]-barycenterPoint).len();// * pow(E,1-mesh.Verts[intersections[0]].Cf);
	for(int i=1;i<intersections.size();i++)
	{
		double tmpValue = (mesh.Verts[intersections[i]]-barycenterPoint).len();// * pow(E,1-mesh.Verts[intersections[i]].Cf);
		if(nowValue>tmpValue)
		{
			nowValue = tmpValue;
			nowPos = i;
		}
	}
	return intersections[nowPos];
}

bool iterationCurve(vector<int> &path,Mesh &mesh)
{
	vector<Point3d> pathPoint;
	vector<int> intersections;
	set<int> ps;
	for(int i=0;i<path.size();i++)
		ps.insert(path[i]);
	for(int i=0;i<path.size();i++)
	{
		double tmpx = mesh.Verts[path[i]].x;
		double tmpy = mesh.Verts[path[i]].y;
		double tmpz = mesh.Verts[path[i]].z;
		pathPoint.push_back(Point3d(tmpx,tmpy,tmpz));
	}
	Point3d barycenterPoint = getBarycenterPoint(pathPoint);
	int pointNum = pathPoint.size();
	double offset = 1.0*(pointNum-1)/(rotateTime-1);
	double now = 0;
	/*for(int i=0;i<rotateTime;i++)
	{
		int tmpNode = now;
		if(tmpNode<0)
			tmpNode=0;
		if(tmpNode>=pointNum)
			tmpNode = pointNum-1;
		addIntersection(pathPoint[tmpNode],barycenterPoint,mesh,intersections,ps);
		now+=offset;
	}*///要改！！！！！！！！！！！！！！！
	addIntersection(pathPoint[pointNum/2],barycenterPoint,mesh,intersections,ps);
	if(intersections.size()==0)
		return false;
	//找到最合适的交点
	int bestIntersection = findBestIntersection(intersections,barycenterPoint,mesh);//要改！！！！！！！！！！！！！！！
	//dijkstra连线
	dija.work(bestIntersection);
	vector<int> tmpNodes;
	int step = 0;
	bool flag = false;
	if(dija.getDist(path[0])<dija.getDist(path[path.size()-1]))
	{
		flag = true;
		int now = path[0];
		int oriNum = path.size();
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			tmpNodes.push_back(now);
			path.push_back(-1);
		}
		int newNum = tmpNodes.size();
		for(int i=oriNum-1;i>=0;i--)
			path[i+newNum] = path[i];
		for(int i=0;i<newNum;i++)
			path[newNum-i-1] = tmpNodes[i];

		now = path[path.size()-1];
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			step++;
		}
	}
	else
	{
		int now = path[path.size()-1];
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			path.push_back(now);
		}

		now = path[0];
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			step++;
		}
	}
	//判断true or false
	/*if(step>5)
		return true;*///要改！！！！！！！！！！！！！！！

	if(flag)
	{
		int now = path[path.size()-1];
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			path.push_back(now);
		}
	}
	else
	{
		int now = path[0];
		int oriNum = path.size();
		while(now!=bestIntersection)
		{
			now = dija.getFather(now);
			tmpNodes.push_back(now);
			path.push_back(-1);
		}
		int newNum = tmpNodes.size();
		for(int i=oriNum-1;i>=0;i--)
			path[i+newNum] = path[i];
		for(int i=0;i<newNum;i++)
			path[newNum-i-1] = tmpNodes[i];
	}
	return false;
}

void iterationCurve2(vector<int> &path,Mesh &mesh)
{
	int pointNum = path.size();
	Point3d p1 = Point3d(mesh.Verts[path[0]].x,mesh.Verts[path[0]].y,mesh.Verts[path[0]].z);
	Point3d p2 = Point3d(mesh.Verts[path[pointNum-1]].x,mesh.Verts[path[pointNum-1]].y,mesh.Verts[path[pointNum-1]].z);
	Point3d p3 = Point3d(mesh.Verts[path[pointNum/2]].x,mesh.Verts[path[pointNum/2]].y,mesh.Verts[path[pointNum/2]].z);
	Point3d pv = pVec(p1,p2,p3);
	Point3d p0 = p1+pv;
	Dijkstra dijb;
	dijb.init(mesh.Verts.size());
	for(int i=0;i<mesh.Edges.size();i++)
	{
		Point3d t1 = mesh.Verts[mesh.Edges[i].a];
		Point3d t2 = mesh.Verts[mesh.Edges[i].b];
		if(segByPlane(p0,p1,p2,p3,t1)&&segByPlane(p0,p1,p2,p3,t2))
		{
			double fk = (mesh.Verts[mesh.Edges[i].a] - mesh.Verts[mesh.Edges[i].b]).len();
			dijb.add(mesh.Edges[i].a,mesh.Edges[i].b,fk,false);
		}
	}
	dijb.work(path[0],path[pointNum-1]);
	int now = path[pointNum-1];
	while(now!=path[0])
	{
		now = dijb.getFather(now);
		if(now==-1)
			break;
		path.push_back(now);
	}
}

void completeCurve(vector<int> &curve,Mesh &mesh,int curvePos)
{
	map<int,vector<int> > ma;
	vector<int> path;
	map<int,int> vis;
	for(int i=0;i<curve.size();i++)
	{
		int t1 = mesh.Edges[curve[i]].a;
		int t2 = mesh.Edges[curve[i]].b;
		ma[t1].push_back(t2);
		ma[t2].push_back(t1);
	}
	int startNode = -1;
	for(map<int,vector<int> >::iterator it = ma.begin();it!=ma.end();it++)
		if(it->second.size()==1)
		{
			startNode = it->first;
			break;
		}
	//如果已经成环 则不操作
	if(startNode==-1)
	{
		vector<Point3d> tmpvvv;
		regionPt.push_back(tmpvvv);
		for(int i=0;i<path.size();i++)
		{
			double tx = mesh.Verts[path[i]].x;
			double ty = mesh.Verts[path[i]].y;
			double tz = mesh.Verts[path[i]].z;
			regionPt[curvePos].push_back(Point3d(tx,ty,tz));
		}
		return;
	}
		
	path.push_back(startNode);
	int lastNode = -1;
	vis[startNode] = 1;
	while(1)
	{
		int endNode = path[path.size()-1];
		int nextNode = -1;
		for(int i=0;i<ma[endNode].size();i++)
			if(ma[endNode][i]!=lastNode)
				nextNode = ma[endNode][i];
		//到另一端则break
		if(nextNode==-1)
			break;
		//如果已经成环 则不操作
		if(vis[nextNode]==1)
		{
			vector<Point3d> tmpvvv;
			regionPt.push_back(tmpvvv);
			for(int i=0;i<path.size();i++)
			{
				double tx = mesh.Verts[path[i]].x;
				double ty = mesh.Verts[path[i]].y;
				double tz = mesh.Verts[path[i]].z;
				regionPt[curvePos].push_back(Point3d(tx,ty,tz));
			}
			return;
		}
		//迭代更新
		vis[nextNode] = 1;
		lastNode = endNode;
		path.push_back(nextNode);
	}
	//while(iterationCurve(path,mesh));
	iterationCurve2(path,mesh);
	vector<Point3d> tmpvvv;
	regionPt.push_back(tmpvvv);
	for(int i=0;i<path.size();i++)
	{
		double tx = mesh.Verts[path[i]].x;
		double ty = mesh.Verts[path[i]].y;
		double tz = mesh.Verts[path[i]].z;
		regionPt[curvePos].push_back(Point3d(tx,ty,tz));
		mesh.Verts[path[i]].color = -1;
	}
}

void completeCurves(Mesh &mesh)
{
	cout << "5.Start" <<endl;
	for(int i=0;i<regionCount;i++)
	{
		completeCurve(regionE[i],mesh,i);
		//cout<<i<<endl;
	}
	cout << "5.Finish" << endl;
}

//---------step6.999 染色--------------------------------------------------


void bfs(int node,int color,Mesh &mesh)
{
	queue<int> que;
	que.push(node);
	while(!que.empty())
	{
		int tmp = que.front();
		que.pop();
		if(mesh.Verts[tmp].color!=0)
			continue;
		mesh.Verts[tmp].color = color;
		for(int i=0;i<mesh.adjVert[tmp].size();i++)
			if(mesh.Verts[mesh.adjVert[tmp][i]].color==0)
				que.push(mesh.adjVert[tmp][i]);
		
	}
}

void setColor(Mesh &mesh)
{
	int nowColor = 0;
	for(int i=0;i<mesh.Verts.size();i++)
	{
		if(mesh.Verts[i].color==0)
		{
			nowColor++;
			bfs(i,nowColor,mesh);
		}
	}
	
	//cout<<"块数:"<<nowColor<<endl;
}

//---------step7 可视化 ----------------------------------------------------
Mesh mesh;
void drawSTL() {
	//cout << "drawSTL";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //清除缓冲，GL_COLOR_BUFFER_BIT ：颜色缓冲标志位
	glLoadIdentity();                                       //重置当前矩阵为4*4的单位矩阵
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //从视点看远点
	
	//渲染颜色
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	for (int i = 0; i <mesh.Facets.size(); i = i + 1) {
		int nowColor = max(max(mesh.Verts[mesh.Facets[i].vertIdx[0]].color,mesh.Verts[mesh.Facets[i].vertIdx[1]].color),mesh.Verts[mesh.Facets[i].vertIdx[2]].color);
		if(nowColor==-1)
		{
			for(int j=0;j<mesh.adjVert[mesh.Facets[i].vertIdx[0]].size();j++)
				nowColor = max(nowColor,mesh.Verts[mesh.adjVert[mesh.Facets[i].vertIdx[0]][j]].color);
		}
		glColor3f(1.0*COLOR1[nowColor]/255, 1.0*COLOR2[nowColor]/255, 1.0*COLOR3[nowColor]/255);		//颜色
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
		for(int j=1;j<regionPt[i].size();j++)
		{
			glVertex3f(regionPt[i][j-1].x, regionPt[i][j-1].y, regionPt[i][j-1].z);
			glVertex3f(regionPt[i][j].x, regionPt[i][j].y, regionPt[i][j].z);
		}
	}
	glEnd();
	glDisable(GL_COLOR_MATERIAL);
	
	glutSwapBuffers();                                      //交换两个缓冲区指针


}

void init(void)
{
	GLfloat light_position[] = { 10.0, 0.0, 0.0, 0.0 };	//光源位置 最后一位 0-方向光源、1-点光源
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //散射光和镜面反射光参数
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //环境光参数,白光
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//设置LIGHT0的位置
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //散射光属性
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //镜面反射光
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBA模式的环境光，为0 
	glShadeModel(GL_SMOOTH);           //颜色过度模式
	glEnable(GL_LIGHTING);			   //开关:使用光
	glEnable(GL_LIGHT0);			   //打开0#灯
	glEnable(GL_DEPTH_TEST);		   //深度测试
	//glEnable(GL_DEPTH_TEST);    //启用深度，根据坐标的远近自动隐藏被遮住的图形（材料）
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //截图;1、2为视口的左下角;3、4为视口的宽度和高度
	glMatrixMode(GL_PROJECTION);    //将当前矩阵指定为投影矩阵
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 1000.0); //1、视野在Y-Z平面的角度[0,180];2、投影平面宽度与高度的比率;3、近截剪面到视点的距离;4、远截剪面到视点的距离
	glMatrixMode(GL_MODELVIEW);     //对模型视景矩阵堆栈应用随后的矩阵操作.
}

void Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)  //记录鼠标按下位置
		OriX = x, OriY = y;
}

void onMouseMove(int x, int y)   //处理鼠标拖动
{
	du += x - OriX; //鼠标在窗口x轴方向上的增量加到视点与x轴的夹角上，就可以左右转
	h += 0.03*(y - OriY);  //鼠标在窗口y轴方向上的改变加到视点y的坐标上，就可以上下转
	if (h>1.0)   h = 1.0;  //对视点y坐标作一些限制，不会使视点太奇怪
	else if (h<-1.0) h = -1.0;
	OriX = x, OriY = y;  //将此时的坐标作为旧值，为下一次计算增量做准备
}

//dijkstra init
void dijkstraInit(Dijkstra &dija,Mesh &mesh)
{
	dija.init(mesh.Verts.size());
	for(int i=0;i<mesh.Edges.size();i++)
	{
		double fk = (mesh.Verts[mesh.Edges[i].a] - mesh.Verts[mesh.Edges[i].b]).len();
		dija.add(mesh.Edges[i].a,mesh.Edges[i].b,fk,false);
	}
}

//int main(int argc, char *argv[])
//{
//	char* orifile = "STLFile/69.stl";
//	char* smoothedfile = "OutFile/r1.stl";
//	Mesh orimData; //原始网格数据
//	Mesh smoothedData;//光顺后的网格数据
//	readASCIstl(orifile, orimData);//读取数据
//	InitMesh(orimData);//计算网格数据
//     //LaplaceSmooth(smoothedfile,orimData,smoothedData);//全局光顺并写入文件
//	concaveRegion(orimData, orimData);  //暂时不光顺！！！！！！
//										
//	dijkstraInit(dija,orimData);
//	completeCurves(orimData);
//	setColor(orimData);
//										//----------可视化----------------
//	mesh = orimData;
//	glutInit(&argc, argv);                                          //初始化glut库
//	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);      //设置初始显示模式
//	glutInitWindowPosition(100, 100);
//	glutInitWindowSize(600, 600);
//	glutCreateWindow("***************");
//	init();
//	glutReshapeFunc(reshape);       //
//	glutDisplayFunc(drawSTL);           //
//	glutIdleFunc(drawSTL);          //设置不断调用显示函数
//	glutMouseFunc(Mouse);
//	glutMotionFunc(onMouseMove);
//	glutMainLoop();//enters the GLUT event processing loop.
//
//
//	return 0;
//}
