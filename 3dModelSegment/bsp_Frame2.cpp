#include "bsp.h"

bsp_Frame2::bsp_Frame2()
{
	this->featureVerts.clear();
	this->featureVerts.clear();
	this->inbadEdge.clear();
	this->inFacets.clear();
	this->inVerts.clear();
}


bsp_Frame2::~bsp_Frame2()
{
}

void bsp_Frame2::findLR(Mesh & mesh)
{
	Point3d &pfromL = getf3().L;
	Point3d &pfromR = getf3().R;
	vector<Point3d> points; //坏边上的点
	for (size_t i = 0; i < this->inbadEdge.size(); ++i)
	{
		//起止点不一定在坏线上 需单独加入
		Point3d l0 = mesh.Verts[this->inbadEdge[i].first];
		Point3d l1 = mesh.Verts[this->inbadEdge[i].second];
		points.push_back(l0);
		points.push_back(l1);
	}

	double minlentoL = DBL_MAX;
	double minlentoR = DBL_MAX;
	size_t minpostoL = -1;
	size_t minpostoR = -1;

	for (size_t k = 0; k < points.size(); ++k)
	{
		Point3d& temp = points[k];

		double templentoL = (temp - pfromL).len();
		double templentoR = (temp - pfromR).len();

		if (templentoL < minlentoL)
		{
			minpostoL = k;
			minlentoL = templentoL;
		}

		if (templentoR < minlentoR)
		{
			minpostoR = k;
			minlentoR = templentoR;
		}
	}

	this->L = points[minpostoL];
	this->R = points[minpostoR];
}

void bsp_Frame2::initUpDown(Mesh & mesh)
{
	for (size_t i = 0; i < this->inbadEdge.size(); ++i)
	{
		//起止点不一定在坏线上 需单独加入
		Point3d l0 = mesh.Verts[this->inbadEdge[i].first];
		Point3d l1 = mesh.Verts[this->inbadEdge[i].second];

		int UD1 = getf3().Up_Down(l0);
		int UD2 = getf3().Up_Down(l1);

		if (UD1 == 1 || UD2 == 1)
		{
			this->UpCurve.push_back(make_pair(l0, l1));
		}
		else
		{
			this->DownCurve.push_back(make_pair(l0, l1));
		}
	}

	/*******************************************/
	//记录Up上的点(去掉LR)
	set<Point3d> UpSet;
	for (size_t i = 0; i < this->UpCurve.size(); ++i)
	{
		Point3d l0 = this->UpCurve[i].first;
		Point3d l1 = this->UpCurve[i].second;
		if (!(l0 == this->L || l0 == this->R))
		{
			UpSet.insert(l0);
		}
		if (!(l1 == this->L || l1 == this->R))
		{
			UpSet.insert(l1);
		}
	}

	//记录Down上的点(去掉LR)
	set<Point3d> DownSet;
	for (size_t i = 0; i < this->DownCurve.size(); ++i)
	{
		Point3d l0 = this->DownCurve[i].first;
		Point3d l1 = this->DownCurve[i].second;
		if (!(l0 == this->L || l0 == this->R))
		{
			DownSet.insert(l0);
		}
		if (!(l1 == this->L || l1 == this->R))
		{
			DownSet.insert(l1);
		}
	}


	//查找与距离f1 L、R两点最近的，且在U D两曲线上的四个点
	double minlenLU = DBL_MAX;
	double minlenRU = DBL_MAX;

	double minlenLD = DBL_MAX;
	double minlenRD = DBL_MAX;

	Point3d minposLU = Point3d(DBL_MAX, DBL_MAX, DBL_MAX);
	Point3d minposRU = Point3d(DBL_MAX, DBL_MAX, DBL_MAX);

	Point3d minposLD = Point3d(DBL_MAX, DBL_MAX, DBL_MAX);
	Point3d minposRD = Point3d(DBL_MAX, DBL_MAX, DBL_MAX);

	set<Point3d>::iterator itU = UpSet.begin();
	while (itU != UpSet.end())
	{
		double tmplenLU = (*itU - this->L).len();
		double tmplenRU = (*itU - this->R).len();

		if (tmplenLU < minlenLU)
		{
			minlenLU = tmplenLU;
			minposLU = *itU;
		}
		if (tmplenRU < minlenRU)
		{
			minlenRU = tmplenRU;
			minposRU = *itU;
		}

		++itU;
	}

	set<Point3d>::iterator itD = DownSet.begin();
	while (itD != DownSet.end())
	{
		double tmplenLD = (*itD - this->L).len();
		double tmplenRD = (*itD - this->R).len();

		if (tmplenLD < minlenLD)
		{
			minlenLD = tmplenLD;
			minposLD = *itD;
		}
		if (tmplenRD < minlenRD)
		{
			minlenRD = tmplenRD;
			minposRD = *itD;
		}
		++itD;
	}

	if (minlenLU != 0)
	{
		this->UpCurve.push_back(make_pair(minposLU, this->L));
	}
	if (minlenLD != 0)
	{
		this->DownCurve.push_back(make_pair(minposLD, this->L));
	}
	if (minlenRU != 0)
	{
		this->UpCurve.push_back(make_pair(minposRU, this->R));
	}
	if (minlenRD != 0)
	{
		this->DownCurve.push_back(make_pair(minposRD, this->R));
	}
}

void bsp_Frame2::findfeatureVerts(Mesh & mesh)
{

}

//不存入当前部件featureLines中
void bsp_Frame2::findfeatureLines(Mesh & mesh)
{
	this->findLR(mesh);
	this->initUpDown(mesh);
	
};
