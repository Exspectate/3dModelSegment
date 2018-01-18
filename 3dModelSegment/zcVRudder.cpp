#include "zcVRudder.h"

void VRudder::findFeatureLines(Mesh & mesh)
{
    int featureLinesNum = 11; //特征线个数
    featureLines.clear();
    featureLines.resize(featureLinesNum);

    //找与机身的交线
    vector<Point3d> pts;
    for (int i = 0; i<inVerts.size(); i++)
        pts.push_back(mesh.Verts[inVerts[i]]);
    vector<Point3d> fPlane;
    fittingPlane(pts, fPlane);
    for (int ii = 0; ii<inFacets.size(); ii++)
    {
        Point3d p1, p2;
        int i = inFacets[ii];
        if (PlaneByTri(fPlane[0], fPlane[1], fPlane[2], mesh.Verts[mesh.Facets[i].vertIdx[0]], mesh.Verts[mesh.Facets[i].vertIdx[1]], mesh.Verts[mesh.Facets[i].vertIdx[2]], p1, p2))
        {
            tappedLine.push_back(make_pair(p1, p2));
        }
    }

    //坏边上的2个拐点
    vector<Point3d> tempLine1;
    int keyPoint1[10];					//坏边上的关键点
    Point3d  keyPoint2[3];				//V型舵上的交线关键点 有序
    memset(keyPoint1, -1, sizeof(keyPoint1));
    tempLine1.clear();
    set<PII> badEdgeSet;
    for (int i = 0; i<inbadEdge.size(); i++)
        badEdgeSet.insert(inbadEdge[i]);
    dfsFindBadLine(mesh, inbadEdge[0].first, -1, tempLine1, -1, badEdgeSet);
    //找keyPoint1[0] keyPoint1[4]
    int now = 0;
    int pnt[2] = { 0,4 };
    for (int i = 0; i + 1<tempLine1.size(); i++)
    {
        Point3d p1, p2, p3;
        if (i == 0)
            p1 = tempLine1[tempLine1.size() - 2];
        else
            p1 = tempLine1[i - 1];
        p2 = tempLine1[i];
        p3 = tempLine1[i + 1];
        Point3d pv1, pv2;
        pv1 = p2 - p1;
        pv2 = p3 - p2;
        double cos12 = dot(pv1, pv2) / pv1.len() / pv2.len();
        if (cos12>0.70711)//cos(pi/4)
            continue;
        keyPoint1[pnt[now++]] = i;
    }
    int PitchPoint = findPartsPoint(mesh, mesh.kind_color[4]);
    Point3d v1 = mesh.Verts[PitchPoint];
    if ((tempLine1[keyPoint1[0]] - v1).len() > (tempLine1[keyPoint1[4]] - v1).len())
        swap(keyPoint1[0], keyPoint1[4]);


    //find keyPoint2[0][1][2]  8 9 10 
    //找在交线上的3个关键点
    map<Point3d, vector<Point3d> > ptAdj;
    for (int i = 0; i<tappedLine.size(); i++)
    {
        ptAdj[tappedLine[i].first].push_back(tappedLine[i].second);
        ptAdj[tappedLine[i].second].push_back(tappedLine[i].first);
    }
    vector<Point3d> pt11And12;
    double maxcos = -2;
    int pos[2] = { 0,2 };
    int count = 0;
    for (map<Point3d, vector<Point3d> >::iterator it = ptAdj.begin(); it != ptAdj.end(); it++)
    {
        if (it->second.size() == 1)
        {
            pt11And12.push_back(it->first);
            continue;
        }
        Point3d p1 = it->first;
        Point3d p2 = it->second[0];
        Point3d p3 = it->second[1];
        Point3d pv1 = p2 - p1;
        Point3d pv2 = p3 - p1;
        double cos12 = dot(pv1, pv2) / pv1.len() / pv2.len();
        if (sign(cos12 + 1) != 0) {
            keyPoint2[pos[count++]] = it->first;
        }
    }
    if ((pt11And12[0] - tempLine1[keyPoint1[0]]).len()>((pt11And12[1] - tempLine1[keyPoint1[0]]).len()))
        swap(pt11And12[0], pt11And12[1]);
    keyPoint2[1] = pt11And12[1];
    //for (int i = 0; i < 3; ++i)
    //    cout << keyPoint2[i].x << " " << keyPoint2[i].y << " " << keyPoint2[i].z << endl;

    //line 4 8
    Point3d nowPt = keyPoint2[1];
    Point3d lastPt = Point3d(1e9, 1e9, 1e9);
    int LineFlag = 1;
    while (true) {
        if (LineFlag == 1)
            featureLines[4].push_back(nowPt);
        else
            featureLines[8].push_back(nowPt);
        if (nowPt == keyPoint2[2])
            break;
        Point3d temp1 = nowPt;
        Point3d temp2 = lastPt;
        for (int i = 0; i < ptAdj[temp1].size(); ++i) {
            if (ptAdj[temp1][i] == temp2)
                continue;
            lastPt = temp1;
            nowPt = ptAdj[temp1][i];
        }
        if (nowPt == keyPoint2[0]) {
            featureLines[4].push_back(nowPt);
            LineFlag = 0;
        }
    }

    //find keyPoint1[2] 4 根据交线的末点 找坏边上最前端的点
    int minPos4 = 0;
    for (int i = 0; i + 1<tempLine1.size(); i++)
    {
        Point3d p = tempLine1[i];
        double nowLenTo11 = (p - keyPoint2[1]).len();
        double minLenTo12 = (tempLine1[minPos4] - keyPoint2[1]).len();
        if (nowLenTo11<minLenTo12)
            minPos4 = i;
    }
    keyPoint1[2] = minPos4;

    /*********************************************************/
    //find keyPoint1[1][3] 坏边上两个小偏角点
    keyPoint1[1] = this->findDePoint(keyPoint1[0], keyPoint1[2], tempLine1);
   for (int i = keyPoint1[4] - 1; i != keyPoint1[2]; i--)
    {
        Point3d p1, p2, p3;
        if (i == 0) {
            p1 = tempLine1[tempLine1.size() - 2];
        }
        else
            p1 = tempLine1[i - 1];
        p2 = tempLine1[i];
        p3 = tempLine1[i + 1];
        Point3d pv1, pv2;
        pv1 = p2 - p1;
        pv2 = p3 - p2;
        double cos12 = dot(pv1, pv2) / pv1.len() / pv2.len();
        if (i == 0)
            i = tempLine1.size() - 1;
        if (sign(1 - cos12) == 0)
            continue;
        keyPoint1[3] = i;
        break;
    }

    //find line 1 2 3 4
    int posLine1[4] = { 3,1,2,4 };
    now = 0;
    for (int i = 0; i < 4; ++i) {
        if (i < 3)
            for (int j = keyPoint1[i]; j <= keyPoint1[i + 1]; ++j)
                this->featureLines[posLine1[now] - 1].push_back(tempLine1[j]);
        else
            for (int j = keyPoint1[i]; j != keyPoint1[i + 1] + 1; ++j) {
                if (j<tempLine1.size())
                    this->featureLines[posLine1[now] - 1].push_back(tempLine1[j]);
                else {
                    j = 0;
                    this->featureLines[posLine1[now] - 1].push_back(tempLine1[j]);
                }
            }
        now++;
    }
    //find line 6
    for (int i = keyPoint1[4]; i <= keyPoint1[0]; ++i) {
        this->featureLines[5].push_back(tempLine1[i]);
    }

    //find 7 8 10 11
    this->findRidgeLine(mesh);
    int keyPoint3[5];				//侧棱上的5个关键点
    map<int, int> degree;			//侧棱上点的度数
    this->findPointDegree(this->RidgeLine, degree);
    int cnt = 0;
    int pos2[2] = { 0,4 };
    for (map<int, int>::iterator it = degree.begin(); it != degree.end(); it++) {
        if (it->second == 1)
            keyPoint3[pos2[cnt++]] = it->first;
    }
    if ((mesh.Verts[keyPoint3[0]] - tempLine1[keyPoint1[0]]).len() > (mesh.Verts[keyPoint3[4]] - tempLine1[keyPoint1[0]]).len())
        swap(keyPoint3[0], keyPoint3[4]);
    vector<Point3d> tempLine2;			//存储异面棱
    tempLine2.clear();
    set<PII> ridgeEdgeSet;
    for (int i = 0; i<this->RidgeLine.size(); i++)
        ridgeEdgeSet.insert(this->RidgeLine[i]);
    dfsFindBadLine(mesh, keyPoint3[0], -1, tempLine2, keyPoint3[4], ridgeEdgeSet);
    int minTo0 = 0;
    int minTo4 = 0;
    int minTo10 = 0;
    for (int i = 0; i <tempLine2.size(); i++)
    {
        Point3d p = tempLine2[i];
        double nowLenTo0 = (p - tempLine1[keyPoint1[0]]).len();
        double nowLenTo4 = (p - tempLine1[keyPoint1[4]]).len();
        double nowLenTo10 = (p - keyPoint2[2]).len();
        double minLenTo0 = (tempLine2[minTo0] - tempLine1[keyPoint1[0]]).len();
        double minLenTo4 = (tempLine2[minTo4] - tempLine1[keyPoint1[4]]).len();
        double minLenTo10 = (tempLine2[minTo10] - keyPoint2[2]).len();
        if (nowLenTo0<minLenTo0)
            minTo0 = i;
        if (nowLenTo4<minLenTo4)
            minTo4 = i;
        if (nowLenTo10<minLenTo10)
            minTo10 = i;
    }
    keyPoint3[0] = minTo0;
    keyPoint3[4] = minTo4;
    keyPoint3[2] = minTo10;
    keyPoint3[1] = this->findDePoint(keyPoint3[0], keyPoint3[4], tempLine2);
    cout << this->findRDePoint(keyPoint3[4], keyPoint3[0], tempLine2);
    keyPoint3[3] = this->findRDePoint(keyPoint3[4], keyPoint3[0], tempLine2);
    int lineId[4] = { 8,11,10,7 };
    now = 0;
    for (int i = 0; i < 4; ++i) {
        for (int k = keyPoint3[i]; k <= keyPoint3[i + 1]; ++k)
            this->featureLines[lineId[now] - 1].push_back(tempLine2[k]);
        now++;
    }
    //将各条线做最后的连接拟合
    featureLines[4][0] = tempLine1[keyPoint1[2]];
    featureLines[8][featureLines[8].size() - 1] = tempLine2[keyPoint3[2]];

    this->featureVerts.clear();
    this->featureVerts.resize(9);
    for (int i = 0; i < this->featureVerts.size(); ++i) {
        if (i < 5)
            featureVerts[i] = tempLine1[keyPoint1[i]];
        else if (i < 8)
            featureVerts[i] = tempLine2[keyPoint3[8 - i]];
        else
            featureVerts[i] = keyPoint2[0];
    }
}


void VRudder::findRidgeLine(Mesh & mesh)
{
    RidgeLine.clear();
    map<PII, bool> vis;			//边是否被访问
    map<PII, PII> eage;			//边及其相邻面片的关系
    map<PII, int> ma;
    for (int i = 0; i < inFacets.size(); ++i) {
        PII a, b, c;
        a.first = c.second = mesh.Facets[inFacets[i]].vertIdx[0];
        a.second = b.first = mesh.Facets[inFacets[i]].vertIdx[1];
        b.second = c.first = mesh.Facets[inFacets[i]].vertIdx[2];
        if (a.first>a.second)
            swap(a.first, a.second);
        if (b.first>b.second)
            swap(b.first, b.second);
        if (c.first>c.second)
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
        Vector13 norm1, norm2;		//相邻面片法向量
        norm1 = mesh.Facets[it->second.first].normV;
        norm2 = mesh.Facets[it->second.second].normV;
        if (norm1.dot(norm2) / norm1.len() / norm2.len() > 0.70711)
            continue;
        else {
            RidgeLine.push_back(it->first);
            mesh.Verts[it->first.first].isRidge = true;
            mesh.Verts[it->first.second].isRidge = true;
        }
    }
}

void VRudder::findPointDegree(vector<PII> Line, map<int, int>& ma)
{
    for (int i = 0; i < Line.size(); ++i) {
        ma[Line[i].first]++;
        ma[Line[i].second]++;
    }
}



void VRudder::dfsFindBadLine(Mesh & mesh, int nowNode, int fa, vector<Point3d>& line, int endNode, set<PII> badEdgeSet)
{
    line.push_back(mesh.Verts[nowNode]);
    if (nowNode == endNode)
        return;
    if (endNode == -1)
        endNode = nowNode;
    for (int i = 0; i < mesh.adjVert[nowNode].size(); i++)
    {
        int adj = mesh.adjVert[nowNode][i];
        if (adj == fa)
            continue;
        /*if (!(mesh.Verts[adj].isBad))
        continue;*/
        if (!(mesh.Verts[adj].isBad) && !(mesh.Verts[adj].isRidge))
            continue;
        PII pii = make_pair(nowNode, adj);
        if (pii.first > pii.second)
            swap(pii.first, pii.second);
        if (!(badEdgeSet.count(pii)))
            continue;
        //存疑 是否只有一个
        dfsFindBadLine(mesh, adj, nowNode, line, endNode, badEdgeSet);
        break;
    }
}

int VRudder::findDePoint(int startNode, int endNode, vector<Point3d>& Line)
{
    for (int i = startNode + 1; i<endNode; i++)
    {
        Point3d p1, p2, p3;
        if (i == 0)
            p1 = Line[Line.size() - 2];
        else
            p1 = Line[i - 1];
        p2 = Line[i];
        p3 = Line[i + 1];
        Point3d pv1, pv2;
        pv1 = p2 - p1;
        pv2 = p3 - p2;
        double cos12 = dot(pv1, pv2) / pv1.len() / pv2.len();
        if (sign(1 - cos12) == 0)
            continue;

        return i;
    }
    return 0;
}

int VRudder::findRDePoint(int startNode, int endNode, vector<Point3d>& Line)
{
    for (int i = startNode - 1; i>endNode; i--)
    {
        Point3d p1, p2, p3;
        if (i == 0)
            p1 = Line[Line.size() - 2];
        else
            p1 = Line[i - 1];
        p2 = Line[i];
        p3 = Line[i + 1];
        Point3d pv1, pv2;
        pv1 = p2 - p1;
        pv2 = p3 - p2;
        double cos12 = dot(pv1, pv2) / pv1.len() / pv2.len();
        if (sign(1 - cos12) == 0)
            continue;
        return i;
    }
    return 0;
}


