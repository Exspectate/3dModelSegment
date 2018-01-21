#include "bsp_Frame3.h"
bsp_Frame3::bsp_Frame3()
{

}

void bsp_Frame3::FindLine(Mesh& mesh, Point3d nowNode, Point3d endNode, set<PII> PIISet, vector<Point3d> &RES, int dir)
{
	RES.clear();

	//ptAdj储存突变线中涉及的点的邻接关系
	map<Point3d, vector<Point3d> > ptAdj;

	set<PII>::iterator it;   //定义前向迭代器 
							 //中序遍历集合中的所有元素
	for (it = PIISet.begin(); it != PIISet.end(); it++)
	{
		Point3d p1 = mesh.Verts[(*it).first];
		Point3d p2 = mesh.Verts[(*it).second];

		ptAdj[p1].push_back(p2);
		ptAdj[p2].push_back(p1);
	}

	//Point3d lastNode = Point3d(-1, -1, -1);
	set<Point3d> UsedP;

	Point3d FirstNode = nowNode;
	int times = ptAdj.size();
	while (1)
	{
		RES.push_back(nowNode);

		if (nowNode == endNode)//到达尾部则结束
		{
			cout << "GETEND" << endl;
			break;
		}


		if (ptAdj.find(nowNode) == ptAdj.end())
		{
			cout << "ERROR";
		}

		vector<Point3d> pV = ptAdj[nowNode];


		if (pV.size() == 1)//当前点是 未预料的结尾点或 度为1的起始点
		{
			bool f = nowNode == FirstNode;
			if (f == true)
			{
				UsedP.insert(nowNode);
				//lastNode = nowNode;
				nowNode = pV[0];
				continue;
			}

			else
			{
				if (dir == 1)
				{
					return;
				}
				RES.clear();
				return this->FindLine(mesh, FirstNode, endNode, PIISet, RES, 1);
			}
		}
		if (dir == 0)
		{
			for (int i = 0; i < pV.size(); i++)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}

				UsedP.insert(nowNode);
				nowNode = pV[i];

			}
		}
		else
		{
			cout << 1;
			for (int i = pV.size() - 1; i >= 0; i--)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}
				UsedP.insert(nowNode);
				nowNode = pV[i];
				break;

			}
		}

		if (nowNode == endNode)
		{
			RES.push_back(nowNode);
		}
	}

}


void bsp_Frame3::FindLine(Mesh& mesh, Point3d nowNode, Point3d endNode, vector<PII> PIISet, vector<Point3d> &RES, int dir)
{
	RES.clear();

	//ptAdj储存突变线中涉及的点的邻接关系
	map<Point3d, vector<Point3d> > ptAdj;

	vector<PII>::iterator it;   //定义前向迭代器 
								//中序遍历集合中的所有元素
	for (it = PIISet.begin(); it != PIISet.end(); it++)
	{
		Point3d p1 = mesh.Verts[(*it).first];
		Point3d p2 = mesh.Verts[(*it).second];

		ptAdj[p1].push_back(p2);
		ptAdj[p2].push_back(p1);
	}
	set<Point3d> UsedP;

	Point3d FirstNode = nowNode;
	int times = ptAdj.size();
	while (1)
	{
		RES.push_back(nowNode);

		if (nowNode == endNode)//到达尾部则结束
		{
			cout << "GETENDD" << endl;
			break;
		}


		if (ptAdj.find(nowNode) == ptAdj.end())
		{
			cout << "ERROR";
		}

		vector<Point3d> pV = ptAdj[nowNode];


		if (pV.size() == 1)//当前点是 未预料的结尾点或 度为1的起始点
		{
			bool f = nowNode == FirstNode;
			if (f == true)
			{
				UsedP.insert(nowNode);
				nowNode = pV[0];
				continue;
			}

			else
			{
				if (dir == 1)
				{
					return;
				}
				RES.clear();
				return this->FindLine(mesh, FirstNode, endNode, PIISet, RES, 1);
			}
		}
		if (dir == 0)
		{
			for (int i = 0; i < pV.size(); i++)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}

				UsedP.insert(nowNode);
				nowNode = pV[i];
				break;
			}
		}
		else
		{
			for (int i = pV.size() - 1; i >= 0; i--)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}
				UsedP.insert(nowNode);
				nowNode = pV[i];
				break;

			}
		}

		if (nowNode == endNode)
		{
			RES.push_back(nowNode);
		}
	}

}



//调用此方法需要保证f1 f2特征线计算完毕
void bsp_Frame3::findfeatureLines(Mesh & mesh)
{
	


	//两条长边界线
	vector<Point3d> BL2;//F2 Left
	BL2.push_back(this->belongTo->f1->L);
	BL2.push_back(this->belongTo->f2->L);

	vector<Point3d> BR2;//F2 Right
	BR2.push_back(this->belongTo->f1->R);
	BR2.push_back(this->belongTo->f2->R);


	vector<Point3d> BL3;
	vector<Point3d> BR3;

	//两条长边界线
	this->BorderL.first = this->L;
	this->BorderL.second = this->belongTo->f2->L;
	BL3.push_back(BorderL.first);
	BL3.push_back(BorderL.second);

	this->BorderR.first = this->R;
	this->BorderR.second = this->belongTo->f2->R;
	BR3.push_back(BorderR.first);
	BR3.push_back(BorderR.second);


	vector<Point3d> TMPres;

	/******************************************/
	Point3d F = this->belongTo->f1->L;
	Point3d L = this->belongTo->f1->R;
	this->FindLine(mesh, F, L, this->belongTo->f1->UpCurve, TMPres, 0); //L1
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L1 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f2->L;
	L = this->belongTo->f2->R;
	this->FindLine(mesh, F, L, this->belongTo->f2->UpCurve, TMPres, 0); //L2
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L2 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f3->featureVerts[0];
	L = this->belongTo->f3->featureVerts[1];
	this->FindLine(mesh, F, L, this->belongTo->f3->UpCurve, TMPres, 0); //L3
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L3 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f3->featureVerts[2];
	L = this->belongTo->f3->featureVerts[3];
	this->FindLine(mesh, F, L, this->belongTo->f3->UpCurve, TMPres, 0); //L4
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L4 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f3->featureVerts[4];
	L = this->belongTo->f3->featureVerts[5];
	this->FindLine(mesh, F, L, this->belongTo->f3->UpCurve, TMPres, 0); //L5
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L5 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f1->L;
	L = this->belongTo->f1->R;
	this->FindLine(mesh, F, L, this->belongTo->f1->DownCurve, TMPres, 1); //L6
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}

	cout << "L6 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f2->L;
	L = this->belongTo->f2->R;
	this->FindLine(mesh, F, L, this->belongTo->f2->DownCurve, TMPres, 0); //L7
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L7 计算完毕" << endl;

	/******************************************/
	F = this->belongTo->f3->featureVerts[0];
	L = this->belongTo->f3->featureVerts[5];
	this->FindLine(mesh, F, L, this->belongTo->f3->RidgeLine, TMPres, 0); //L8
	this->featureLines.push_back(TMPres);
	if (TMPres[0] == F && TMPres[TMPres.size() - 1] == L)
	{
		cout << "起始点计算正确" << endl;
	}
	else
	{
		cout << "起始点计算错误" << endl;
	}
	cout << "L8 计算完毕" << endl;

	this->featureLines.push_back(this->belongTo->f1->Curve); //L9
	cout << "L9 计算完毕" << endl;

	this->featureLines.push_back(BR2);//L10
	cout << "L10 计算完毕" << endl;

	this->featureLines.push_back(BR3);//L11
	cout << "L11 计算完毕" << endl;

	this->featureLines.push_back(BL2);//L12
	cout << "L12 计算完毕" << endl;

	this->featureLines.push_back(BL3);//L13
	cout << "L13 计算完毕" << endl;

}

void bsp_Frame3::FindLine(Mesh& mesh, Point3d nowNode, Point3d endNode, vector<pair<Point3d, Point3d>> PPSet, vector<Point3d> &RES, int dir)
{
	PPSet.erase(unique(PPSet.begin(), PPSet.end()), PPSet.end());

	RES.clear();

	//ptAdj储存突变线中涉及的点的邻接关系
	map<Point3d, vector<Point3d> > ptAdj;

	vector<pair<Point3d, Point3d>>::iterator it;   //定义前向迭代器 
												   //中序遍历集合中的所有元素
	for (it = PPSet.begin(); it != PPSet.end(); it++)
	{
		Point3d p1 = (*it).first;
		Point3d p2 = (*it).second;

		ptAdj[p1].push_back(p2);
		ptAdj[p2].push_back(p1);
	}

	set<Point3d> UsedP;

	Point3d FirstNode = nowNode;
	int times = ptAdj.size();
	while (1)
	{
		RES.push_back(nowNode);

		if (nowNode == endNode)//到达尾部则结束
		{
			cout << "GETEND" << endl;
			break;
		}


		if (ptAdj.find(nowNode) == ptAdj.end())
		{
			cout << "找不到该点" << endl;
		}

		vector<Point3d> pV = ptAdj[nowNode];//暂未查明为何会有相同边
		pV.erase(unique(pV.begin(), pV.end()), pV.end());


		if (pV.size() <= 1)//当前点是 未预料的结尾点或 度为1的起始点
		{
			bool f = nowNode == FirstNode;
			if (f == true)
			{
				UsedP.insert(nowNode);
				nowNode = pV[0];
				continue;
			}
			else
			{
				if (dir == 1)
				{
					return;
				}
				RES.clear();
				return this->FindLine(mesh, FirstNode, endNode, PPSet, RES, 1);
			}
		}
		if (dir == 0)
		{
			for (int i = 0; i < pV.size(); i++)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}

				UsedP.insert(nowNode);
				nowNode = pV[i];

			}
		}
		else
		{
			for (int i = pV.size() - 1; i >= 0; i--)
			{

				if (UsedP.find(pV[i]) != UsedP.end())
				{
					continue;
				}
				UsedP.insert(nowNode);
				nowNode = pV[i];
				break;

			}
		}

		if (nowNode == endNode)
		{
			RES.push_back(nowNode);
		}
	}

}

void bsp_Frame3::findfeatureVerts(Mesh & mesh)
{
	this->featureVerts.push_back(this->L); //P1

										   //V型舵上点
	this->Pnear2 = this->belongTo->leftVRudder->featureVerts[4];
	this->Pnear3 = this->belongTo->leftVRudder->featureVerts[0];
	this->Pnear4 = this->belongTo->rightVRudder->featureVerts[0];
	this->Pnear5 = this->belongTo->rightVRudder->featureVerts[4];

	double minlento2 = DBL_MAX;
	double minlento3 = DBL_MAX;
	double minlento4 = DBL_MAX;
	double minlento5 = DBL_MAX;

	Point3d minposto2;
	Point3d minposto3;
	Point3d minposto4;
	Point3d minposto5;

	for (size_t k = 0; k < this->RidgeLine.size(); ++k)
	{
		Point3d& temp1 = mesh.Verts[this->RidgeLine[k].first];
		Point3d& temp2 = mesh.Verts[this->RidgeLine[k].second];

		double templento2 = (temp1 - Pnear2).len();
		double templento3 = (temp1 - Pnear3).len();
		double templento4 = (temp1 - Pnear4).len();
		double templento5 = (temp1 - Pnear5).len();

		if (templento2 < minlento2)
		{
			minposto2 = mesh.Verts[this->RidgeLine[k].first];
			minlento2 = templento2;
		}
		if (templento3 < minlento3)
		{
			minposto3 = mesh.Verts[this->RidgeLine[k].first];
			minlento3 = templento3;
		}
		if (templento4 < minlento4)
		{
			minposto4 = mesh.Verts[this->RidgeLine[k].first];
			minlento4 = templento4;
		}
		if (templento5 < minlento5)
		{
			minposto5 = mesh.Verts[this->RidgeLine[k].first];
			minlento5 = templento5;
		}

		templento2 = (temp2 - Pnear2).len();
		templento3 = (temp2 - Pnear3).len();
		templento4 = (temp2 - Pnear4).len();
		templento5 = (temp2 - Pnear5).len();

		if (templento2 < minlento2)
		{
			minposto2 = mesh.Verts[this->RidgeLine[k].second];
			minlento2 = templento2;
		}
		if (templento3 < minlento3)
		{
			minposto3 = mesh.Verts[this->RidgeLine[k].second];
			minlento3 = templento3;
		}
		if (templento4 < minlento4)
		{
			minposto4 = mesh.Verts[this->RidgeLine[k].second];
			minlento4 = templento4;
		}
		if (templento5 < minlento5)
		{
			minposto5 = mesh.Verts[this->RidgeLine[k].second];
			minlento5 = templento5;
		}
	}

	this->UpCurve.push_back(make_pair(minposto2, Pnear2));
	this->UpCurve.push_back(make_pair(minposto3, Pnear3));
	this->UpCurve.push_back(make_pair(minposto4, Pnear4));
	this->UpCurve.push_back(make_pair(minposto5, Pnear5));

	this->featureVerts.push_back(Pnear2);//P2
	this->featureVerts.push_back(Pnear3);//P3
	this->featureVerts.push_back(Pnear4);//P4
	this->featureVerts.push_back(Pnear5);//P5

	this->featureVerts.push_back(this->R);  //P6

	this->featureVerts.push_back(this->belongTo->f2->R);//P7
	this->featureVerts.push_back(this->belongTo->f1->R);//P8
	this->featureVerts.push_back(this->belongTo->f1->L);//P9
	this->featureVerts.push_back(this->belongTo->f2->L);//P10




}

void bsp_Frame3::findRidgeLine(Mesh & mesh)
{
	//突变线
	vector<PII> RidgeLine;

	RidgeLine.clear();
	map<PII, bool> vis;   //边是否被访问
	map<PII, PII> eage;   //边及其相邻面片的关系
	map<PII, int> ma;
	for (int i = 0; i < inFacets.size(); ++i) {
		PII a, b, c;
		a.first = c.second = mesh.Facets[inFacets[i]].vertIdx[0];
		a.second = b.first = mesh.Facets[inFacets[i]].vertIdx[1];
		b.second = c.first = mesh.Facets[inFacets[i]].vertIdx[2];
		if (a.first > a.second)
			swap(a.first, a.second);
		if (b.first > b.second)
			swap(b.first, b.second);
		if (c.first > c.second)
			swap(c.first, c.second);

		if (!vis[a]) {
			vis[a] = 1;
			eage[a].first = inFacets[i];
			if (!vis[b]) {
				vis[b] = 1;
				eage[b].first = inFacets[i];
				if (!vis[c]) {
					vis[c] = 1;
					eage[c].first = inFacets[i];
				}
				else
					eage[c].second = inFacets[i];
			}
			else {
				eage[b].second = inFacets[i];
				if (!vis[c]) {
					vis[c] = 1;
					eage[c].first = inFacets[i];
				}
				else
					eage[c].second = inFacets[i];
			}
		}
		else {
			eage[a].second = inFacets[i];
			if (!vis[b]) {
				vis[b] = 1;
				eage[b].first = inFacets[i];
				if (!vis[c]) {
					vis[c] = 1;
					eage[c].first = inFacets[i];
				}
				else
					eage[c].second = inFacets[i];
			}
			else {
				eage[b].second = inFacets[i];
				if (!vis[c]) {
					vis[c] = 1;
					eage[c].first = inFacets[i];
				}
				else
					eage[c].second = inFacets[i];
			}
		}
		ma[a]++;
		ma[b]++;
		ma[c]++;
	}

	for (map<PII, PII>::iterator it = eage.begin(); it != eage.end(); it++) {
		if (ma[it->first] == 1)
			continue;
		Vector13 norm1, norm2;  //相邻面片法向量
		norm1 = mesh.Facets[it->second.first].normV;
		norm2 = mesh.Facets[it->second.second].normV;
		if (norm1.dot(norm2) / norm1.len() / norm2.len() > 0.70711)
			continue;
		else {
			RidgeLine.push_back(it->first);
		}
	}

	this->RidgeLine = RidgeLine;

	map<int, int> point_times;

	for (size_t i = 0; i < this->inFacets.size(); i++)
	{
		for (int j = 0; j < 3; ++j)
		{
			int idx = mesh.Facets[this->inFacets[i]].vertIdx[j];
			point_times[idx]++;
		}

	}

	//Find度数最大点
	map<int, int>::iterator it;
	it = point_times.begin();

	int maxtimes = -1;
	int maxpos = -1;

	while (it != point_times.end())
	{
		if (it->second > maxtimes)
		{
			maxtimes = it->second;
			maxpos = it->first;
		}
		it++;
	}

	this->bigpoint = mesh.Verts[maxpos];

	double maxdis1 = -1;
	double maxdis2 = -1;

	Point3d maxpos1;
	Point3d maxpos2;

	for (int i = 0; i < RidgeLine.size(); ++i)
	{
		Point3d p1 = mesh.Verts[RidgeLine[i].first];
		Point3d p2 = mesh.Verts[RidgeLine[i].second];

		double tempdis1 = (this->bigpoint - p1).len();
		double tempdis2 = (this->bigpoint - p2).len();

		if (tempdis1 > maxdis1)
		{
			maxdis1 = tempdis1;
			maxpos1 = p1;
			continue;
		}
		if (tempdis2 > maxdis2)
		{
			maxdis2 = tempdis2;
			maxpos2 = p2;
			continue;
		}

	}

	//根据左右V舵辨别左右
	double LEN1 = (this->belongTo->leftVRudder->featureVerts[4] - maxpos1).len();
	double LEN2 = (this->belongTo->rightVRudder->featureVerts[4] - maxpos1).len();
	if (LEN1 < LEN2)
	{
		this->L = maxpos1;
		this->R = maxpos2;
	}
	else if (LEN1 > LEN2)
	{
		this->R = maxpos1;
		this->L = maxpos2;
	}
	else
	{
		cout << "Can not fnid left or right!" << endl;
	}

	////计算上下
	//Point3d t1 = this->R - this->L;   //L->R
	//Point3d t2 = this->belongTo->f2->L - this->L;    //曲线2左端点->L(->R)

	//Vector13 tV1 = Vector13(t1.x, t1.y, t1.z);
	//Vector13 tV2 = Vector13(t2.x, t2.y, t2.z);

	//this->Up = tV1.crossmult(tV2);


}

int bsp_Frame3::Up_Down(Point3d p)
{
	Point3d CenterOfF2;
	CenterOfF2.x = (this->belongTo->f2->L.x + this->belongTo->f2->R.x) / 2;
	CenterOfF2.y = (this->belongTo->f2->L.y + this->belongTo->f2->R.y) / 2;
	CenterOfF2.z = (this->belongTo->f2->L.z + this->belongTo->f2->R.z) / 2;

	Point3d temp = p - CenterOfF2;

	double res = temp.x * this->Up.x + temp.y * this->Up.y + temp.z * this->Up.z;

	if (res > 0)
	{
		return 1;
	}
	else if (res < 0)
	{
		return -1;
	}
	else
	{
		return 0;

	}
}

void bsp_Frame3::initUpDirection()
{
	//计算上方向
	Point3d t1 = this->R - this->L;   //L->R
	Point3d t2 = this->belongTo->f2->L - this->L;    //曲线2左端点->L(->R)

	Vector13 tV1 = Vector13(t1.x, t1.y, t1.z);
	Vector13 tV2 = Vector13(t2.x, t2.y, t2.z);
	this->Up = tV1.crossmult(tV2);
}

void bsp_Frame3::initUpDown(Mesh & mesh)
{
	//区分上半部分、下半部分
	for (size_t i = 0; i < this->RidgeLine.size(); ++i)
	{
		//起止点不一定在坏线上 需单独加入
		Point3d l0 = mesh.Verts[this->RidgeLine[i].first];
		Point3d l1 = mesh.Verts[this->RidgeLine[i].second];

		int UD1 = this->belongTo->f3->Up_Down(l0);
		int UD2 = this->belongTo->f3->Up_Down(l1);

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
	//将边界点与最近点连线存入对应曲线上
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

bsp_Frame3::~bsp_Frame3()
{
}