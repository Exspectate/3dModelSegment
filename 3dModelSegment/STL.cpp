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

static int du = 90, OriX = -1, OriY = -1;   //du���ӵ��x��ļн�
static float r = 1.5, h = 0.0;   //r���ӵ���y��İ뾶��h���ӵ�߶ȼ���y���ϵ�����
static float c = PI / 180.0;    //���ȺͽǶ�ת������
const int rotateTime = 12;
#pragma warning(disable:4996)
using namespace std;

vector<vector<int>> regionE(MAX);
int regionCount;
Dijkstra dija;
vector<vector<Point3d> >regionPt;
void InitMesh(Mesh& mdata) {
	cout << "Calculating adjacent facets" << endl;
	mdata.CaladjFacet();//�����������ڽ���Ƭ
	cout << "Calculating adjacent vertexes" << endl;
	mdata.CaladjVert();//�����������ڽӵ�
	cout << "Calculating normal vector for vertexes" << endl;
	mdata.Calvertnorm();//���������ķ�����
	CalCf(mdata);//���������İ�͹�ź�
	//sort(mdata.Cf.begin(), mdata.Cf.end(), cmp); //����
}

void LaplaceSmooth(char* wfile, Mesh &mdata, Mesh &ndata) {
	cout << "1.Laplace smooth" << endl;

	//2. Laplaceƽ��-------------------
	cout << "Calculating Laplace matrix" << endl;
	SparseMatrix<double> L = CalLapMatrix(mdata);//��������ģ�͵�������˹����
	cout << "Calculating Cf" << endl;
	vector<VtCf> Cf1;
	Cf1.assign(mdata.Cf.begin(), mdata.Cf.end());
	//���㰼͹�ź�����
	cout << "Sorting Cf" << endl;

	sort(Cf1.begin(), Cf1.end(), cmp); //����

									   //ѡȡ30%��Ƶ���ƶ���
	cout << "Selecting control vertexes" << endl;
	vector<int> ControlVerts;//���ƶ���������
	double ratio_control = 0.30;//���ƶ������
	int CPnum = (int)(ratio_control*(Cf1.size()));
	//cout << CPnum << endl;
	int VTnum = mdata.Verts.size();
	//cout << VTnum << endl;
	for (int i = 0; i < CPnum; i++) {
		ControlVerts.push_back(Cf1[i].idx);
	}

	//�ⷽ����
	double c = 0.01; //����Ȩ��

					 //��ʼ��ϡ�����A 
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

	//д���˳��Ķ�������
	cout << "Saving vertexes coordinate after smoothing" << endl;
	for (int i = 0; i < VTnum; i++) {
		ndata.Verts[i].x = VX[i];
		ndata.Verts[i].y = VY[i];
		ndata.Verts[i].z = VZ[i];
	}
	//�����˳���������Ƭ�ķ�����
	cout << "Re-calculating normal vectors of facets" << endl;
	ndata.Calfacetnorm();
	ndata.Calvertnorm();
	CalCf(ndata);
	//����˳��Ķ���������淨����д���µ�stl�ļ�
	cout << "Writing to new STL" << endl;

	writeASCIstl(wfile, ndata.Verts, ndata.Facets);
	//writeASCIstltest(wfile, ndata.Verts);

	cout << "1.Finish" << endl;
	return;
}

void concaveRegion(Mesh &oridata, Mesh &smdata) {
	cout << "2.concaveRegion" << endl;

	//3.1 ��������ּ��       (��smdata�Ͻ���)
	cout << "��������ּ��" << endl;
	double ave = 0, var = 0;
	double threshold1 = 2.5;  //�ּ����ֵ  1.6-2.6
	vector<int> concaveVerts; //��������������

	vector<int> concavesign(oridata.Verts.size(), -1);//�������������
													  //���㰼͹�źž�ֵ
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		ave += smdata.Cf[i].Cf;
	}
	ave /= smdata.Verts.size();
	//���㰼͹�źŷ���
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		var += (smdata.Cf[i].Cf - ave) * (smdata.Cf[i].Cf - ave);
	}
	var /= smdata.Verts.size();
	var = sqrt(var);
	cout << "��ֵ=" << ave << endl;
	cout << "��׼��=" << var << endl;

	//�����׼����͹�źŲ�ɸѡ��������
	//for (size_t i = 0; i < smdata.Verts.size(); i++) {
	//	if ((smdata.Cf[i].Cf - ave) / var > threshold1) {
	//		concaveVerts.push_back(smdata.Cf[i].idx);
	//		concavesign[smdata.Cf[i].idx] = -2;  //���
	//	}
	//}
	for (size_t i = 0; i < smdata.Verts.size(); i++) {
		if ((smdata.Verts[i].Cf - ave) / var > threshold1) {
			concaveVerts.push_back(i);
			concavesign[i] = -2;  //���
		}
	}
	//cout << concaveVerts.size() << endl;

	//vector<Point3d> testVerts;

	//	for (size_t j = 0; j < concaveVerts.size(); j++) {
	//		testVerts.push_back(oridata.Verts[concaveVerts[j]]);
	//	}

	//writeASCIstltest("OutFile/testconcave.stl", testVerts);
	//exit(0);


	//3.2 ���������򾫻�   (��oridata�Ͻ���)  
	//�������������γ����ɰ�������������
	cout << "�������򾫻�" << endl;
	vector<int> unprocessed;  //������㼯
	int regionNum = 0;
	vector<vector<int>> regionVerts;//�������ڵ�
	vector<vector<VtCf>> regionCf;
	unprocessed.assign(concaveVerts.begin(), concaveVerts.end());
	while (!unprocessed.empty()) {  //ÿѭ��һ�δ���һ������
		vector<int> thisregion;
		vector<VtCf> thisregionCf;
		stack<int> s;
		int idx = *(unprocessed.begin());//ȡ��ǰ�ĵ�һ���������
		thisregion.push_back(idx);//������������㼯
		concavesign[idx] = regionNum;//��¼�õ���������
	//	thisregionCf.push_back(VtCf(oridata.Cf[idx].Cf,idx));
	  thisregionCf.push_back(VtCf(oridata.Verts[idx].Cf, idx));
		delvec(unprocessed, idx);//����Ӵ�����㼯��ɾ��
		do {
			if (oridata.adjVert[idx].size() != 0) {//���õ��ڽӵ���ջ�������������㼯�����Ӵ�����㼯��ɾ��
				for (size_t i = 0; i < oridata.adjVert[idx].size(); i++) {
					if (concavesign[oridata.adjVert[idx][i]] == -1) { continue; } //�ų��ǰ������
					if (concavesign[oridata.adjVert[idx][i]] != -2) { continue; } //�ų�����ĵ�
					thisregion.push_back(oridata.adjVert[idx][i]);
					concavesign[oridata.adjVert[idx][i]] = regionNum;//��¼�õ���
		// thisregionCf.push_back(VtCf(oridata.Cf[oridata.adjVert[idx][i]].Cf, oridata.adjVert[idx][i]));
			   thisregionCf.push_back(VtCf(oridata.Verts[oridata.adjVert[idx][i]].Cf, oridata.adjVert[idx][i]));
					delvec(unprocessed, oridata.adjVert[idx][i]);
					s.push(oridata.adjVert[idx][i]);
				}
			}
			//����ջ���ĵ�,�Ըõ��ظ�����
			if (!s.empty()) {
				idx = s.top();
				s.pop();
			}
		} while (!s.empty());
		if (thisregion.size() > 1) {   //���������ֻ��һ���㣬��ɾ��������
			regionVerts.push_back(thisregion);//���������ĵ�
			regionCf.push_back(thisregionCf);//���������ĵ�
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

	//��ÿ������������������д���
	double threshold2 = 0.5;               //�������ֵ
	int regionNumMinus = 0;
	for (size_t i = 0; i < regionCf.size(); i++) {
		sort(regionCf[i].begin(), regionCf[i].end(), cmp);
		double Cmin = regionCf[i].front().Cf;
		double Cmax = regionCf[i].back().Cf;
		//��һ���õ�N(Cf)
		for (size_t j = 0; j < regionCf[i].size(); j++) {
			double NCf = (regionCf[i][j].Cf - Cmin) / (Cmax - Cmin);
			//ȥ�����ھ������ֵ�ĵ�
			//if (NCf >= threshold2) {
			//	//delvec(concaveVerts, regionCf[i][j].idx);
			//	delvec(regionVerts[i], regionCf[i][j].idx);
			//	concavesign[regionCf[i][j].idx] = -1;
			//}
		}
		//���������ĳ������ֻ��1�����0���㣬��ɾ��������
		if (regionVerts[i].size() <= 1) {
			cout << "ɾ��������" << i << endl;
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
		//���������ɾ������֮������ĵ������Ǽ�1
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
	//�޸�
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

	//������õ���ÿ���������������ٰ���2���㣬����2����֮��һ����һ����

	//4. ����������ϸ��

	cout << "4.����������ϸ��" << endl;
	cout << "��������߼�" << endl;
	//��������߼�
	int eidx = 0;
	for (int i = 0; i < oridata.Verts.size(); i++) {
		for (int j = 0; j < oridata.adjVert[i].size(); j++) {
			if (oridata.adjVert[i][j] <= i) continue;
			oridata.Edges.push_back(Edge(i, oridata.adjVert[i][j], eidx));
			eidx++;
		}
	}
	cout << "edgenum:" << eidx + 1 << endl;

	vector<int> concavesigne(oridata.Edges.size(), -1);//����������߱��


	cout << "�������бߵ��ڽ���Ƭ" << endl;
	// �������бߵ��ڽ���Ƭ�������ڽ���Ƭ����Ӹñߵ���Ϣ
	for (int i = 0; i < oridata.Edges.size(); i++) {
		adjFacetForEdge(oridata.Edges[i], oridata);
	}

	vector<vector<int>> regionEdge(regionNum);
	//��ÿ��������
	cout << "ϸ��ÿ��������..." << endl;
	cout << "������" << regionNum << endl;

	for (int i = 0; i < regionNum; i++) {
		regionVerts[i].clear();
		for (size_t j = 0; j < oridata.Edges.size(); j++) {
			if (concavesign[oridata.Edges[j].a] == i && concavesign[oridata.Edges[j].b] == i)
			{
				regionEdge[i].push_back(oridata.Edges[j].idx);//��ð��������һ����
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
			//cout << "ɾ����" << endl;
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
		//�ѵõ�ÿ�������ڵ�ϸ����ıߡ������޸�regionVerts[i]Ϊϸ�����ϵĵ�


		for (size_t k = 0; k < regionEdge[i].size(); k++) {
			if (find(regionVerts[i].begin(), regionVerts[i].end(), oridata.Edges[regionEdge[i][k]].a) == regionVerts[i].end())
				regionVerts[i].push_back(oridata.Edges[regionEdge[i][k]].a);
			if (find(regionVerts[i].begin(), regionVerts[i].end(), oridata.Edges[regionEdge[i][k]].b) == regionVerts[i].end())
				regionVerts[i].push_back(oridata.Edges[regionEdge[i][k]].b);
		}
		//--------�����������ߴ���
		for (size_t k = 0; k < regionEdge[i].size(); k++) {
			regionE[i].push_back(regionEdge[i][k]);
		}
	}
	regionCount = regionNum;
	//test 4---------------------
	cout << "test 4 д��" << endl;
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
	}*///Ҫ�ģ�����������������������������
	addIntersection(pathPoint[pointNum/2],barycenterPoint,mesh,intersections,ps);
	if(intersections.size()==0)
		return false;
	//�ҵ�����ʵĽ���
	int bestIntersection = findBestIntersection(intersections,barycenterPoint,mesh);//Ҫ�ģ�����������������������������
	//dijkstra����
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
	//�ж�true or false
	/*if(step>5)
		return true;*///Ҫ�ģ�����������������������������

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
	//����Ѿ��ɻ� �򲻲���
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
		//����һ����break
		if(nextNode==-1)
			break;
		//����Ѿ��ɻ� �򲻲���
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
		//��������
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

//---------step6.999 Ⱦɫ--------------------------------------------------


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
	
	//cout<<"����:"<<nowColor<<endl;
}

//---------step7 ���ӻ� ----------------------------------------------------
Mesh mesh;
void drawSTL() {
	//cout << "drawSTL";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //������壬GL_COLOR_BUFFER_BIT ����ɫ�����־λ
	glLoadIdentity();                                       //���õ�ǰ����Ϊ4*4�ĵ�λ����
	gluLookAt(r*cos(c*du), h, r*sin(c*du), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);   //���ӵ㿴Զ��
	
	//��Ⱦ��ɫ
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	for (int i = 0; i <mesh.Facets.size(); i = i + 1) {
		int nowColor = max(max(mesh.Verts[mesh.Facets[i].vertIdx[0]].color,mesh.Verts[mesh.Facets[i].vertIdx[1]].color),mesh.Verts[mesh.Facets[i].vertIdx[2]].color);
		if(nowColor==-1)
		{
			for(int j=0;j<mesh.adjVert[mesh.Facets[i].vertIdx[0]].size();j++)
				nowColor = max(nowColor,mesh.Verts[mesh.adjVert[mesh.Facets[i].vertIdx[0]][j]].color);
		}
		glColor3f(1.0*COLOR1[nowColor]/255, 1.0*COLOR2[nowColor]/255, 1.0*COLOR3[nowColor]/255);		//��ɫ
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
		for(int j=1;j<regionPt[i].size();j++)
		{
			glVertex3f(regionPt[i][j-1].x, regionPt[i][j-1].y, regionPt[i][j-1].z);
			glVertex3f(regionPt[i][j].x, regionPt[i][j].y, regionPt[i][j].z);
		}
	}
	glEnd();
	glDisable(GL_COLOR_MATERIAL);
	
	glutSwapBuffers();                                      //��������������ָ��


}

void init(void)
{
	GLfloat light_position[] = { 10.0, 0.0, 0.0, 0.0 };	//��Դλ�� ���һλ 0-�����Դ��1-���Դ
	GLfloat white_light[] = { 1.0, 1.0, 1.0, 1.0 };     //ɢ���;��淴������
	GLfloat Light_Model_Ambient[] = { 0.0,0.0,0.0, 1.0 }; //���������,�׹�
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);	//����LIGHT0��λ��
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light);      //ɢ�������
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light);     //���淴���
	glLightfv(GL_LIGHT0, GL_AMBIENT, Light_Model_Ambient);//RGBAģʽ�Ļ����⣬Ϊ0 
	glShadeModel(GL_SMOOTH);           //��ɫ����ģʽ
	glEnable(GL_LIGHTING);			   //����:ʹ�ù�
	glEnable(GL_LIGHT0);			   //��0#��
	glEnable(GL_DEPTH_TEST);		   //��Ȳ���
	//glEnable(GL_DEPTH_TEST);    //������ȣ����������Զ���Զ����ر���ס��ͼ�Σ����ϣ�
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);    //��ͼ;1��2Ϊ�ӿڵ����½�;3��4Ϊ�ӿڵĿ�Ⱥ͸߶�
	glMatrixMode(GL_PROJECTION);    //����ǰ����ָ��ΪͶӰ����
	glLoadIdentity();
	gluPerspective(75.0, (float)w / h, 1.0, 1000.0); //1����Ұ��Y-Zƽ��ĽǶ�[0,180];2��ͶӰƽ������߶ȵı���;3�����ؼ��浽�ӵ�ľ���;4��Զ�ؼ��浽�ӵ�ľ���
	glMatrixMode(GL_MODELVIEW);     //��ģ���Ӿ������ջӦ�����ľ������.
}

void Mouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)  //��¼��갴��λ��
		OriX = x, OriY = y;
}

void onMouseMove(int x, int y)   //��������϶�
{
	du += x - OriX; //����ڴ���x�᷽���ϵ������ӵ��ӵ���x��ļн��ϣ��Ϳ�������ת
	h += 0.03*(y - OriY);  //����ڴ���y�᷽���ϵĸı�ӵ��ӵ�y�������ϣ��Ϳ�������ת
	if (h>1.0)   h = 1.0;  //���ӵ�y������һЩ���ƣ�����ʹ�ӵ�̫���
	else if (h<-1.0) h = -1.0;
	OriX = x, OriY = y;  //����ʱ��������Ϊ��ֵ��Ϊ��һ�μ���������׼��
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
//	Mesh orimData; //ԭʼ��������
//	Mesh smoothedData;//��˳�����������
//	readASCIstl(orifile, orimData);//��ȡ����
//	InitMesh(orimData);//������������
//     //LaplaceSmooth(smoothedfile,orimData,smoothedData);//ȫ�ֹ�˳��д���ļ�
//	concaveRegion(orimData, orimData);  //��ʱ����˳������������
//										
//	dijkstraInit(dija,orimData);
//	completeCurves(orimData);
//	setColor(orimData);
//										//----------���ӻ�----------------
//	mesh = orimData;
//	glutInit(&argc, argv);                                          //��ʼ��glut��
//	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);      //���ó�ʼ��ʾģʽ
//	glutInitWindowPosition(100, 100);
//	glutInitWindowSize(600, 600);
//	glutCreateWindow("***************");
//	init();
//	glutReshapeFunc(reshape);       //
//	glutDisplayFunc(drawSTL);           //
//	glutIdleFunc(drawSTL);          //���ò��ϵ�����ʾ����
//	glutMouseFunc(Mouse);
//	glutMotionFunc(onMouseMove);
//	glutMainLoop();//enters the GLUT event processing loop.
//
//
//	return 0;
//}
