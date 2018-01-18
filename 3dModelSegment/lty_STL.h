#ifndef _lty_STL_H_
#define _lty_STL_H_

#include <fstream>
#include <iomanip>  
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <algorithm>
#include <set>
#include <map>
#include <iostream>
#include "Point3d.h"
#include "Util.h"
#include "dijkstra.h"
#include "3dOperator.h"


#pragma warning(disable:4996)

using namespace std;
using namespace Eigen;



typedef Eigen::Triplet<double> T;

struct Vector13 { //1x3向量
public:
	double x, y, z;
	Vector13();
	~Vector13();
	Vector13(double x, double y, double z);
	double dot(const Vector13 &t);
    Vector13 crossmult(const Vector13 &t);
    double getcos(const Vector13 &t);
    double len();
};

class TriFacet { //三角面片
public:
	//int nEdges=3;
	vector<int> vertIdx;//三个顶点索引
	vector<int> edgeIdx;//三条边索引
	Vector13 normV;//法向量
	int idx;//索引
	TriFacet();
	~TriFacet();
	void addVertIdx(int a);
	void setnormV(Vector13 nv);
	void setIdx(int a);
};

class Edge {
public:
	int a,b;//两端点索引
	int adj1, adj2;//两邻接面片索引
	int idx;
	Edge();
	Edge(int a, int b, int idx);
	void setFacetidx(int adj1, int adj2);
};

struct VtCf {
	double Cf;
	int idx;
	VtCf(double Cf, int idx);
};

class Mesh {   //网格模型
public:
	vector<Point3d> Verts;
	vector<TriFacet> Facets;
	vector<Edge> Edges;
	vector< vector<int> > adjVert;//顶点的邻接顶点索引集
	vector< vector<int> > adjFacet;//顶点的邻接面片索引集
	vector<Vector13> vertnorm;//顶点的法向量集
	vector< VtCf > Cf;//顶点的凹凸信号
	vector<int> badPointsId; //坏点id集合

	map<int, int> kind_color;

	void CaladjVert();//计算各顶点的邻接顶点
	void CaladjFacet();//计算各顶点的邻接面片
	void Calvertnorm();//计算各顶点的法向量
	void Calfacetnorm();//计算各三角面片的法向量
	void AddVertex(Point3d& pts);//添加顶点
	void AddFacet(TriFacet& facets);//添加面片
};

SparseMatrix<double> CalLapMatrix(Mesh& mesh);

//计算各顶点的凹凸度
void CalCf(Mesh& mesh);

bool cmp(VtCf& a, VtCf& b);

//从vector<int>中按值删除元素
void delvec(vector<int> &vec, int value);


//从ASCII型STL文件中读取数据
void readASCIstl(char* filename, Mesh& mdata);

//写数据到ASCII型STL文件
void writeASCIstl(char* filename, vector<Point3d> &Verts, vector<TriFacet> &Facets);

void writeASCIstltest(char* filename, vector<Point3d> &Verts);

void adjFacetForEdge(Edge &e, Mesh &mesh);//计算一条边的邻接面片,并向该面片中添加该边的信息

//判断一条边是否处在凹区域边界   
bool isBoundaryEdge(Edge &e, Mesh &mesh, vector<int> &concavesigne, int num);

//去掉边后添加新的边
void addnewboundEdges(list<int> &list, Edge &e, Mesh &mesh, vector<int> &concavesigne, int num);

//
bool belongsToMeshFeature(Edge &e, Mesh &mesh, vector<int> &regionEdge);

Point3d det2(const Point3d &a, const Point3d &b);

Point3d pVec2(const Point3d &a, const Point3d &b, const Point3d &c);

#endif