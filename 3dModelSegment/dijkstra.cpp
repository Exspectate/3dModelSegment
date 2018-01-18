#include "dijkstra.h"

void Dijkstra::init(int size)
{
	n = size;
	vector<int> tmp;
	vector<double> tmp2;
	tmp.clear();
	v.clear();
	w.clear();
	v.reserve(size+1);
	w.reserve(size+1);
	father.reserve(size+1);
	for(int i=0;i<=size;i++)
	{
		v.push_back(tmp);
		w.push_back(tmp2);
	}
	for(int i=0;i<=n;i++)
	{
		dist.push_back(-1);
		father.push_back(-1);
	}
}

void Dijkstra::add(int node1,int node2,double weight,bool flag)
{
	v[node1].push_back(node2);
	w[node1].push_back(weight);
	if(flag)
		return;
	v[node2].push_back(node1);
	w[node2].push_back(weight);
}

void Dijkstra::work(int index)
{
	for(int i=0;i<=n;i++)
		dist[i] = -1;
	while(!pq.empty())
		pq.pop();
	pq.push(node(index,0,-1));
	while(!pq.empty())
	{
		node tmp = pq.top();
		pq.pop();
		if(dist[tmp.index]!=-1)
			continue;
		dist[tmp.index] = tmp.dist;
		father[tmp.index] = tmp.father;
		for(int i=0;i<v[tmp.index].size();i++)
			if(dist[v[tmp.index][i]]==-1)
				pq.push(node(v[tmp.index][i],tmp.dist+w[tmp.index][i],tmp.index));
	}
}


void Dijkstra::work(int startIndex,int endIndex)
{
	for(int i=0;i<=n;i++)
		dist[i] = -1;
	while(!pq.empty())
		pq.pop();
	pq.push(node(startIndex,0,-1));
	while(!pq.empty())
	{
		node tmp = pq.top();
		pq.pop();
		if(dist[tmp.index]!=-1)
			continue;
		dist[tmp.index] = tmp.dist;
		father[tmp.index] = tmp.father;
		if(tmp.index==endIndex)
			break;
		for(int i=0;i<v[tmp.index].size();i++)
			if(dist[v[tmp.index][i]]==-1)
				pq.push(node(v[tmp.index][i],tmp.dist+w[tmp.index][i],tmp.index));
	}
}


double Dijkstra::getDist(int node)
{
	if(node>=0&&node<dist.size())
		return dist[node];
	return -1;
}

int Dijkstra::getFather(int node)
{
	if(node>=0&&node<father.size())
		return father[node];
	return -1;
}