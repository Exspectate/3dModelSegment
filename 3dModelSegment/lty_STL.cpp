#include "lty_STL.h"
#include"bsp.h"


Vector13::Vector13()
{
	x = 0; y = 0; z = 0;
}
Vector13::Vector13(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}
Vector13::~Vector13() {}
double Vector13::dot(const Vector13 &t) 
{
	return x*t.x + y*t.y + z*t.z;
}

Vector13 Vector13::crossmult(const Vector13 & t)
{
    Vector13 res;
    res.x = this->y * t.z - t.y * this->z;
    res.y = this->z * t.x - t.z * this->x;
    res.z = this->x * t.y - t.x * this->y;

    double l = res.len();
    res.x /= l;
    res.y /= l;
    res.z /= l;
    return res;
}

double Vector13::getcos(const Vector13 & t)
{
    if (this->dot(t) > 0)
        return 1;
    else if (this->dot(t) < 0)
        return -1;
    else
        return 0;
}

double Vector13::len()
{
	return sqrt(x*x + y * y + z * z);
}
TriFacet::TriFacet() {};
TriFacet::~TriFacet() {};
void TriFacet::addVertIdx(int a) {
	vertIdx.push_back(a);
}
void TriFacet::setnormV(Vector13 nv) {
	normV.x = nv.x;
	normV.y = nv.y;
	normV.z = nv.z;
}
void TriFacet::setIdx(int a) {
	idx = a;
}

Edge::Edge() {};
Edge::Edge(int a, int b, int idx) {
	if (a > b) swap(a, b);
	this->a = a;
	this->b = b;
	this->idx = idx;
}
void Edge::setFacetidx(int adj1, int adj2) {
	this->adj1 = adj1;
	this->adj2 = adj2;
};

VtCf::VtCf(double Cf, int idx) {
	this->Cf = Cf;
	this->idx = idx;
}

void Mesh::AddVertex(Point3d& pts)//添加顶点
{
	Verts.push_back(pts);
}
void Mesh::AddFacet(TriFacet& facets)//添加面片
{
	Facets.push_back(facets);
}
void Mesh::CaladjFacet() {
	int j; int temp;
	adjFacet.resize(Verts.size());
	//对每个三角面片，将该面片的索引添加到它的三个顶点的邻接面片集中
	for (size_t i = 0; i < Facets.size(); i++) {
		TriFacet& f = Facets[i];
		for (j = 0; j < 3; j++) {
			temp = f.vertIdx[j];
			adjFacet[temp].push_back(f.idx);
		}
	}
	////test
	//cout << "邻接面片集" << endl;
	//for (size_t j = 0; j < Verts.size(); j++) {
	//	for (size_t k = 0; k < adjFacet[j].size(); k++) {
	//		cout << adjFacet[j][k];
	//		cout << ' ';
	//	}
	//	cout << endl;
	//}
	////test end
}
void Mesh::CaladjVert() {
	int p0, p1, p2;
	adjVert.resize(Verts.size());
	//对每个三角面片，将该面片三个顶点的索引添加到彼此的邻接点集中
	//===========================================================================================
	for (size_t i = 0; i < Facets.size(); i++) {
		p0 = Facets[i].vertIdx[0];
		p1 = Facets[i].vertIdx[1];
		p2 = Facets[i].vertIdx[2];
		//避免重复添加
		if (find(adjVert[p0].begin(), adjVert[p0].end(), p1) == adjVert[p0].end())
		{
			adjVert[p0].push_back(p1);
			adjVert[p1].push_back(p0);
		}
		if (find(adjVert[p0].begin(), adjVert[p0].end(), p2) == adjVert[p0].end())
		{
			adjVert[p0].push_back(p2);
			adjVert[p2].push_back(p0);
		}
		if (find(adjVert[p1].begin(), adjVert[p1].end(), p2) == adjVert[p1].end())
		{
			adjVert[p1].push_back(p2);
			adjVert[p2].push_back(p1);
		}
	}
	////test
	//cout << "邻接点集" << endl;
	//for (size_t j = 0; j < Verts.size(); j++) {   
	//	for (size_t k = 0; k < adjVert[j].size(); k++) {
	//		cout << adjVert[j][k];
	//		cout << ' ';
	//	}
	//	cout << endl;
	//}
	////test end
}
void Mesh::Calvertnorm() {
	int i;
	double x, y, z;
	int temp;
	for (i = 0; i < Verts.size(); i++) {
		x = 0;
		y = 0;
		z = 0;
		if (adjFacet[i].size() == 0) {
			cout << "gulidian" << endl;
			vertnorm.push_back(Vector13(0, 0, 0));
		}
		else {
			for (size_t j = 0; j < adjFacet[i].size(); j++) {
				temp = adjFacet[i][j];
				x += Facets[temp].normV.x;
				y += Facets[temp].normV.y;
				z += Facets[temp].normV.z;
			}
			x = x / adjFacet[i].size();
			y = y / adjFacet[i].size();
			z = z / adjFacet[i].size();
			x /= sqrt(x*x + y*y + z*z);
			y /= sqrt(x*x + y*y + z*z);
			z /= sqrt(x*x + y*y + z*z);
			vertnorm.push_back(Vector13(x, y, z));
		}
	}
	////test
	//cout << "点法向量集" << endl;
	//for (size_t j = 0; j < Verts.size(); j++) {   
	//	cout << vertnorm[j].x << ' ' << vertnorm[j].y << ' ' << vertnorm[j].z << endl;
	//}
	////test end
}
void Mesh::Calfacetnorm() {
	for (size_t i = 0; i < Facets.size(); i++) {
		Point3d &p0 = Verts[Facets[i].vertIdx[0]];
		Point3d &p1 = Verts[Facets[i].vertIdx[1]];
		Point3d &p2 = Verts[Facets[i].vertIdx[2]];
		Point3d p = pVec2(p0, p1, p2);
		Facets[i].normV.x = p.x;
		Facets[i].normV.y = p.y;
		Facets[i].normV.z = p.z;
		double len = (double)sqrt(Facets[i].normV.x * Facets[i].normV.x + Facets[i].normV.y * Facets[i].normV.y + Facets[i].normV.z * Facets[i].normV.z);
		Facets[i].normV.x /= len;
		Facets[i].normV.y /= len;
		Facets[i].normV.z /= len;
	}
}

Point3d det2(const Point3d &a, const Point3d &b)
{
	return Point3d(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

Point3d pVec2(const Point3d &a, const Point3d &b, const Point3d &c)
{
	return det2(a - b, b - c);
}

void Calfacetnorm(vector<Point3d> &Verts, vector<TriFacet> &Facets) {
	for (size_t i = 0; i < Facets.size(); i++) {
		Point3d &p0 = Verts[Facets[i].vertIdx[0]];
		Point3d &p1 = Verts[Facets[i].vertIdx[1]];
		Point3d &p2 = Verts[Facets[i].vertIdx[2]];
		double v1x = p1.x - p0.x;
		double v1y = p1.y - p0.y;
		double v1z = p1.z - p0.z;
		double v2x = p2.x - p1.x;
		double v2y = p2.y - p1.y;
		double v2z = p2.z - p1.z;
		Facets[i].normV.x = v1y * v2z - v1z * v2y;
		Facets[i].normV.y = v1z * v2x - v1x * v2z;
		Facets[i].normV.z = v1x * v2y - v1y * v2x;
		double len = (double)sqrt(Facets[i].normV.x * Facets[i].normV.x + Facets[i].normV.y * Facets[i].normV.y + Facets[i].normV.z * Facets[i].normV.z);
		Facets[i].normV.x /= len;
		Facets[i].normV.y /= len;
		Facets[i].normV.z /= len;
	}
}

SparseMatrix<double> CalLapMatrix(Mesh& mesh) {
	//生成拉普拉斯矩阵L
	int Msize = mesh.Verts.size();
	SparseMatrix<double> L(Msize, Msize);
	vector<T> triplets;
	for (int i = 0; i < Msize; i++) {
		triplets.push_back(T(i, i, 1));
		for (int j = 0; j < mesh.adjVert[i].size(); j++) {
			triplets.push_back(T(i, mesh.adjVert[i][j], (-1.0) / mesh.adjVert[i].size()));
		}
	}
	L.setFromTriplets(triplets.begin(), triplets.end());
	return L;
}

//计算各顶点的凹凸度
void CalCf(Mesh& mesh) {
	Vector3d vn, vij;
	int pidx; double max;
	set<int> neighbor2;
	for (size_t i = 0; i < mesh.Verts.size(); i++) {
		neighbor2.clear();
		vn << mesh.vertnorm[i].x, mesh.vertnorm[i].y, mesh.vertnorm[i].z;
		//寻找顶点i的所有二阶邻域点
		//vector <int> neighbor2;
		//首先将顶点i的邻接点（一阶邻域内的点）加入neighbor2
		for (size_t j = 0; j < mesh.adjVert[i].size(); j++) {
			neighbor2.insert(mesh.adjVert[i][j]);
		}
		//扫描顶点i各邻接点的邻接点，若不在neighbor2中，则加入
		for (size_t j = 0; j < mesh.adjVert[i].size(); j++) {
			int temp = mesh.Verts[mesh.adjVert[i][j]].idx;
			for (size_t k = 0; k < mesh.adjVert[temp].size(); k++) {
				if (mesh.Verts[mesh.adjVert[temp][k]].idx != i)//排除顶点i自身
					neighbor2.insert(mesh.Verts[mesh.adjVert[temp][k]].idx);
			}
		}
		////从neighbor2中删除顶点i的邻接点
		//for (size_t j = 0; j < mesh.adjVert[i].size(); j++){
		//	neighbor2.erase(neighbor2.begin() + j);
		//}
		//计算每个VN*Vij,寻找最大值
		max = 0;
		for (set<int>::iterator it = neighbor2.begin(); it != neighbor2.end(); it++) {
			pidx = *it;
			vij << mesh.Verts[pidx].x - mesh.Verts[i].x, mesh.Verts[pidx].y - mesh.Verts[i].y, mesh.Verts[pidx].z - mesh.Verts[i].z;
			if (vn.dot(vij) > max) {
				max = vn.dot(vij);
			}
		}
		VtCf vtcf(max, i);
		mesh.Cf.push_back(vtcf);
		mesh.Verts[i].Cf = max;
	}
	neighbor2.clear();
}

bool cmp(VtCf& a, VtCf& b) {
	return a.Cf < b.Cf;
}

//从vector<int>中按值删除元素
void delvec(vector<int> &vec, int value) {
	vector<int>::iterator iter;
	for (iter = vec.begin(); iter != vec.end(); )
	{
		if (value == *iter)
			iter = vec.erase(iter);
		else
			++iter;
	}
}

//计算(对角元为1的)Laplace矩阵
//MatrixXd CalLapMatrix(Mesh& mesh) {
//	//生成度矩阵D和邻接矩阵A
//	MatrixXd D, A, L;
//	int Msize = mesh.Verts.size();
//	D.resize(Msize, Msize);
//	A.resize(Msize, Msize);
//	D.fill(0);
//	A.fill(0);
//	for (int i = 0; i < Msize; i++) {
//		D(i, i) = mesh.adjVert[i].size();
//		for (int j = 0; j < mesh.adjVert[i].size(); j++) {
//			A(i, mesh.adjVert[i][j]) = 1;
//		}
//	}
//	//cout << "度矩阵D=" << endl << D << endl;
//	//cout << "邻接矩阵A=" << endl << A << endl;
//	//生成Laplace矩阵
//	L = D - A;
//	//调整比例使对角元为1
//	for (int i = 0; i < Msize; i++) {
//		L.row(i) /= mesh.adjVert[i].size();
//	}
//	//cout << "Laplace矩阵L=" << endl << L << endl;
//	return L;
//}

static class myEdge {
public:
	int a, b;//两端点索引
	int idx;
	myEdge() {}
	myEdge(int a, int b, int idx) {
		if (a > b) swap(a, b);
		this->a = a;
		this->b = b;
		this->idx = idx;
	}
	friend bool operator < (const myEdge &e1, const myEdge &e2) {
		return e1.a == e2.a ? e1.b < e2.b: e1.a < e2.a;
	}
};

//从ASCII型STL文件中读取数据
vector<TriFacet> Fcdata;
vector<Point3d> Ptdata;
set<Point3d> PtSet;
set<myEdge> Eg_Set;
vector<vector<int>> edgeAdjFT;
void readASCIstl(char* filename, Mesh& mdata)
{
	FILE *fp = NULL;
	errno_t err = 0;
	err = fopen_s(&fp, filename, "r");
	//vector<int> Ptidx;//顶点的索引集
	cout << "Start reading" << endl;
	if (err != 0) {
		printf("read %s failed!\n", filename);
		system("pause");
		exit(-1);
	}
	
		char str[100], test[100];
		int facetIdx = 0, pointIdx = 0, edgeIdx = 0;
		int flag = 0;
		double x, y, z;
		while (1)
		{
			fscanf(fp, "%s", &str);
			if (!strcmp(str, "facet"))
			{
				fscanf(fp, "%s", &test);
				fscanf(fp, "%lf %lf %lf\n", &x, &y, &z);
				//cout << "Adding facet" << facetIdx << endl;
				TriFacet facet;
				facet.setIdx(facetIdx);
				Vector13 nv(x, y, z);
				facet.setnormV(nv);
				//cout << "Adding norm-vector:";
				//cout << "(" << x << "," << y << "," << z << ")" << endl;
				Fcdata.push_back(facet);
				continue;
			}
			else if (!strcmp(str, "outer"))
			{
				fscanf(fp, "%s", &test);
				int ptId[3];
				for (int i = 0; i < 3; i++)
				{
					fscanf(fp, "%s", &test);
					fscanf(fp, "%lf %lf %lf\n", &x, &y, &z);

					Point3d p(x, y, z);
					flag = 0;
					/*	for (vector<Point3d>::iterator ip = Ptdata.begin(); ip != Ptdata.end(); ip++) {
					if (*ip == p) {
					Fcdata[facetIdx].addVertIdx((*ip).idx);
					flag = 1;
					break;
					}
					}*/
					/*for (size_t i = 0; i < Ptdata.size(); i++) {
					if (Ptdata[i].x == p.x &&Ptdata[i].y == p.y && Ptdata[i].z == p.z) {
					Fcdata[facetIdx].addVertIdx(Ptdata[i].idx);
					flag = 1;
					break;
					}
					}
					if (flag == 1) continue;*/
					if (PtSet.count(p))
					{
						Fcdata[facetIdx].addVertIdx(PtSet.find(p)->idx);
						ptId[i] = PtSet.find(p)->idx;
						continue;
					}
					Fcdata[facetIdx].addVertIdx(pointIdx);
					p.setIdx(pointIdx);
					//cout << "Adding vertex:index" << pointIdx << endl;
					//cout << "(" << x << "," << y << "," << z << ")" << endl;
					Ptdata.push_back(p);
					PtSet.insert(p);
					ptId[i] = pointIdx;
					pointIdx++;
				}
				for (int i = 0; i < 3; ++i) {     //记录边出现的次数
					myEdge temp(ptId[i], ptId[(i + 1) % 3], edgeIdx);
					set<myEdge>::iterator it = Eg_Set.find(temp);
					if (it == Eg_Set.end()) {
						Eg_Set.insert(temp);
						vector<int> tt;
						tt.push_back(facetIdx);
						edgeAdjFT.push_back(tt);
						++edgeIdx;
					}
					else {
						edgeAdjFT[it->idx].push_back(facetIdx);
					}
				}
				facetIdx++;
				continue;
			}
			else if (!strcmp(str, "endsolid"))
			{
				break;
			}
			else
			{
				continue;
			}
		}
	
	mdata.badPointsId.clear();
	int bad_case_N = 0;
	for (set<myEdge>::iterator it = Eg_Set.begin(); it != Eg_Set.end(); ++it) { //找出所有坏边上的坏点
		if (edgeAdjFT[it->idx].size() > 2) {
			printf("An edge belongs %d facets!!\n", edgeAdjFT[it->idx].size());
			++bad_case_N;
			continue;
		}
		if (edgeAdjFT[it->idx].size() == 2) {
			//Vector13 v1(Fcdata[edgeAdjFT[it->idx][0]].normV), v2(Fcdata[edgeAdjFT[it->idx][1]].normV);
			//if(abs(v1.dot(v2) +1) >= 1e-3)   //判断相邻两个三角面元法向量是否反向
				continue;
		}
		mdata.badPointsId.push_back(it->a);
		mdata.badPointsId.push_back(it->b);
	}
	printf("Bad case number: %d\n", bad_case_N);

	sort(mdata.badPointsId.begin(), mdata.badPointsId.end());
	mdata.badPointsId.erase(unique(mdata.badPointsId.begin(), mdata.badPointsId.end()), mdata.badPointsId.end()); //对坏点id去重


	cout << "facet number:" << facetIdx << "\n";
	cout << "Read completed." << endl;
	cout << "V Num:" << Ptdata.size() << endl;
	fclose(fp);
	//cout<<"1"<<endl;
	mdata.Facets.assign(Fcdata.begin(), Fcdata.end());
	//cout<<"2"<<endl;
	mdata.Verts.assign(Ptdata.begin(), Ptdata.end());
	//cout<<"3"<<endl;
	Ptdata.clear();
	//cout<<"4"<<endl;
	Fcdata.clear();
	//cout<<"5"<<endl;
	PtSet.clear();
	//cout<<"6"<<endl;
	return;
}

//写数据到ASCII型STL文件
void writeASCIstl(char* filename, vector<Point3d> &Verts, vector<TriFacet> &Facets) {
	ofstream ofile;
	ofile.open(filename);
	ofile << "solid " << filename << endl;
	int j = 0;
	for (size_t i = 0; i < Facets.size(); i++) {
		ofile << " facet normal ";
		ofile << setiosflags(ios::fixed);
		ofile << setprecision(5) << Facets[i].normV.x << ' ';
		ofile << setprecision(5) << Facets[i].normV.y << ' ';
		ofile << setprecision(5) << Facets[i].normV.z << ' ';
		ofile << endl;
		ofile << "  outer loop" << endl;
		ofile << "   vertex " << setprecision(5) << Verts[Facets[i].vertIdx[0]].x << ' ' << Verts[Facets[i].vertIdx[0]].y << ' ' << Verts[Facets[i].vertIdx[0]].z << endl;
		ofile << "   vertex " << setprecision(5) << Verts[Facets[i].vertIdx[1]].x << ' ' << Verts[Facets[i].vertIdx[1]].y << ' ' << Verts[Facets[i].vertIdx[1]].z << endl;
		ofile << "   vertex " << setprecision(5) << Verts[Facets[i].vertIdx[2]].x << ' ' << Verts[Facets[i].vertIdx[2]].y << ' ' << Verts[Facets[i].vertIdx[2]].z << endl;
		ofile << "  endloop" << endl;
		ofile << " endfacet" << endl;
	}
	ofile << "endsolid " << filename << endl;
	ofile.close();
}

void writeASCIstltest(char* filename, vector<Point3d> &Verts) { //只写点
	ofstream ofile;
	ofile.open(filename);
	ofile << "solid " << filename << endl;
	int j = 0;
	int n = (Verts.size() / 3) * 3;
	while (j<n) {
		ofile << " facet normal ";
		ofile << setiosflags(ios::fixed);
		ofile << setprecision(5) << 0 << ' ';
		ofile << setprecision(5) << 0 << ' ';
		ofile << setprecision(5) << 0 << ' ';
		ofile << endl;
		ofile << "  outer loop" << endl;
		ofile << "   vertex " << setprecision(5) << Verts[j].x << ' ' << Verts[j].y << ' ' << Verts[j].z << endl; j++;
		ofile << "   vertex " << setprecision(5) << Verts[j].x << ' ' << Verts[j].y << ' ' << Verts[j].z << endl; j++;
		ofile << "   vertex " << setprecision(5) << Verts[j].x << ' ' << Verts[j].y << ' ' << Verts[j].z << endl; j++;

		ofile << "  endloop" << endl;
		ofile << " endfacet" << endl;
	}
	ofile << "endsolid " << filename << endl;
	ofile.close();
}

//计算一条边的邻接面片,并向该面片中添加该边的信息
void adjFacetForEdge(Edge &e, Mesh &mesh) {
	int flag = 1;
	for (int i = 0; i < mesh.adjFacet[e.a].size(); i++) {
		for (int j = 0; j < mesh.adjFacet[e.b].size(); j++) {
			if (mesh.adjFacet[e.a][i] == mesh.adjFacet[e.b][j]) {
				if (flag == 1) {
					e.adj1 = mesh.adjFacet[e.a][i];
					mesh.Facets[e.adj1].edgeIdx.push_back(e.idx);
					flag = 2;
					continue;
				}
				e.adj2 = mesh.adjFacet[e.a][i];
				mesh.Facets[e.adj2].edgeIdx.push_back(e.idx);
				return;
			}
		}
	}
}

//判断一条边是否处在凹区域边界   
bool isBoundaryEdge(Edge &e, Mesh &mesh, vector<int> &concavesigne, int num) {
	for (int j = 0; j < 3; j++) {
		if (concavesigne[mesh.Facets[e.adj1].edgeIdx[j]] != num || concavesigne[mesh.Facets[e.adj2].edgeIdx[j]] != num)
			return true;
	}
	return false;
}

//去掉边后添加新的边
void addnewboundEdges(list<int> &list, Edge &e, Mesh &mesh, vector<int> &concavesigne, int num) {
	//判断该边的哪个邻接面片在凹区域内
	if (concavesigne[mesh.Facets[e.adj1].edgeIdx[0]] == num && concavesigne[mesh.Facets[e.adj1].edgeIdx[1]] == num && concavesigne[mesh.Facets[e.adj1].edgeIdx[2]] == num) {
		//说明adj1在凹区域内
		if (e.idx == mesh.Facets[e.adj1].edgeIdx[0]) {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[1]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[1]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[2]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[2]);
		}
		else if
			(e.idx == mesh.Facets[e.adj1].edgeIdx[1]) {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[0]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[0]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[2]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[2]);
		}
		else {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[0]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[0]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj1].edgeIdx[1]) == list.end())
				list.push_back(mesh.Facets[e.adj1].edgeIdx[1]);
		}
	}
	else if (concavesigne[mesh.Facets[e.adj2].edgeIdx[0]] == num && concavesigne[mesh.Facets[e.adj2].edgeIdx[1]] == num && concavesigne[mesh.Facets[e.adj2].edgeIdx[2]] == num) {
		//说明adj2在凹区域内
		if (e.idx == mesh.Facets[e.adj2].edgeIdx[0]) {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[1]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[1]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[2]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[2]);
		}
		else if
			(e.idx == mesh.Facets[e.adj2].edgeIdx[1]) {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[0]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[0]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[2]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[2]);
		}
		else {
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[0]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[0]);
			if (find(list.begin(), list.end(), mesh.Facets[e.adj2].edgeIdx[1]) == list.end())
				list.push_back(mesh.Facets[e.adj2].edgeIdx[1]);
		}
		return;
	}
	else {//说明adj1和adj2都不在凹区域内,此时不添加新边
		return;
	}
}
//
bool belongsToMeshFeature(Edge &e, Mesh &mesh, vector<int> &regionEdge) {
	//condition1
	int cnta = 0;
	for (int i = 0; i < regionEdge.size(); i++) {
		if (e.a == mesh.Edges[regionEdge[i]].a || e.a == mesh.Edges[regionEdge[i]].b) {
			if (mesh.Edges[regionEdge[i]].idx != e.idx) cnta++;
		}
	}
	int cntb = 0;
	for (int i = 0; i < regionEdge.size(); i++) {
		if (e.b == mesh.Edges[regionEdge[i]].a || e.b == mesh.Edges[regionEdge[i]].b) {
			if (mesh.Edges[regionEdge[i]].idx != e.idx) cntb++;
		}
	}
	if ((cntb == 1 && cnta == 0) || (cntb == 0 && cnta == 1))  //condition 1  接受
		return true;
	else if (cntb == 0 && cnta == 0) {
		//cout << "???" << endl;
		return false;
	}
	else if ((cntb > 1 && cnta == 0) || (cntb == 0 && cnta > 1))    //condition 1  排除
		return false;
	else
	{
		//condition 2
		int cn1 = 0, cn2 = 0;
		for (int i = 0; i < regionEdge.size(); i++) {
			if (e.adj1 == mesh.Edges[regionEdge[i]].adj1 || e.adj1 == mesh.Edges[regionEdge[i]].adj2)
				cn1++;
		}
		for (int i = 0; i < regionEdge.size(); i++) {
			if (e.adj2 == mesh.Edges[regionEdge[i]].adj1 || e.adj2 == mesh.Edges[regionEdge[i]].adj2)
				cn2++;
		}
		if (cn1 == 3 || cn2 == 3) return false;
		return true;
	}
}