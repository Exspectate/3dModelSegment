#ifndef _zc_VRudder_H_
#define _zc_VRudder_H_
#include "bsp_part.h"
#include "gy_fitting.h"


class VRudder :public bsp_part {
public:
    vector<PII> RidgeLine;								//异面交线
    vector<pair<Point3d, Point3d> > tappedLine;			//平面与机身交线
public:
    virtual void findFeatureLines(Mesh &mesh);
    void findRidgeLine(Mesh & mesh);
    void findPointDegree(vector<PII> Line, map<int, int> &ma);
    void dfsFindBadLine(Mesh &mesh, int nowNode, int fa, vector<Point3d> &line, int endNode, set<PII> badEdgeSet);
    int findDePoint(int startNode, int endNode, vector<Point3d> &Line);	//找偏转小角度点
    int findRDePoint(int startNode, int endNode, vector<Point3d> &Line); //反向找小角度点
};
#endif;
