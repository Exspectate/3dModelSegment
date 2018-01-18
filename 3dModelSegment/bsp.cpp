#include "bsp.h"

bsp_Frame1 f1;
bsp_Frame2 f2;
bsp_Frame3 f3;
VRudder leftVRudder;	//左V型舵
VRudder rightVRudder;	//右V型舵



bsp_Frame1& getf1()
{
	return f1;
}

bsp_Frame2& getf2()
{
	return f2;
}

bsp_Frame3& getf3()
{
	return f3;
}

VRudder& getvrudderL()
{
    return leftVRudder;
}

VRudder& getvrudderR()
{
    return rightVRudder;
}

void callbsp(Mesh& mesh)
{
    getvrudderL().initSelectData(mesh,mesh.kind_color[5]);
    getvrudderL().findFeatureLines(mesh);

    getvrudderR().initSelectData(mesh, mesh.kind_color[6]);
    getvrudderR().findFeatureLines(mesh);

	getf1().initSelectData(mesh, 1);
	getf2().initSelectData(mesh, 2);
	getf3().initSelectData(mesh, 4);


	//下述顺序不可调整
	getf3().findRidgeLine(mesh);
	getf3().findfeatureVerts(mesh);

	getf2().findfeatureLines(mesh);
	getf1().findfeatureLines(mesh);

	getf3().findfeatureLines(mesh);
	getf3().writeFeatureLinesToFile("DataFile/Frame/");

}