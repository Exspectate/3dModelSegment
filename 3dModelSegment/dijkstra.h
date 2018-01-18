#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_
#include <iostream>
#include <vector>
#include <queue>
#include <cstring>
using namespace std;

class Dijkstra
{
public:
	void init(int size=100);//初始化为size个点，边表清空
	void work(int index);//从index开始跑单源最短路
	void work(int startIndex,int endIndex); //从startIndex跑最短路，跑到endIndex为止
	void add(int node1,int node2,double weight,bool flag=false);//添加一条node1到node2，权值为weight的边，flag=true代表有向边
	double getDist(int node); //获取node的dist
	int getFather(int node); //获取node的father
private:
	struct node
	{
		int index;
		double dist;
		int father;
		bool operator < (const node &a) const
		{
			return dist>a.dist;
		}
		node(int index=-1,double dist=-1,int father=-1)
		{
			this->index = index;
			this->dist = dist;
			this->father = father;
		}
	};
	vector<vector<int> > v; //边
	vector<vector<double> > w; //边权
	priority_queue<node> pq;
	vector<double> dist;
	vector<int> father;
	int n;
};
#endif