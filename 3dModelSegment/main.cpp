#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cstring>
using namespace std;
#include "dijkstra.h"
#include "Util.h"
#include "Point3d.h"
#include "3dOperator.h"
//extern const double eps;
//extern int sign(double x);
Dijkstra dij;
bool work()
{
	int n,m;
	scanf("%d%d",&n,&m);
	if(n==0&&m==0)
		return false;
	dij.init(n);
	for(int i=0;i<m;i++)
	{
		int t1,t2,t3;
		scanf("%d%d%d",&t1,&t2,&t3);
		dij.add(t1,t2,t3,false);
	}
	int startNode = 1;
	dij.work(startNode);
	printf("×î¶Ì¾àÀë£º%d\n",dij.getDist(n));
	vector<int> path;
	int now = n;
	while(now!=startNode)
	{
		path.push_back(now);
		now = dij.getFather(now);
	}
	path.push_back(startNode);
	cout<<"Â·¾¶£º";
	for(int i=path.size()-1;i>=0;i--)
		cout<<path[i]<<" ";
	cout<<endl;
	return true;
}




//²âÊÔ
/*
int main()
{
	while(1)
	{
		Point3d a,b,c,l0,l1;
		readPt(a);
		readPt(b);
		readPt(c);
		readPt(l0);
		readPt(l1);
		//Point3d t = intersection(a,b,c,l0,l1);
		//show(t);
		cout<<segByPlane(a,b,c,l0,l1);
		//show(intersection(a,b,l0,l1));
		cout<<endl;
	}
	while(work());
}*/