#pragma once


#include "bsp_Frame1.h"
#include "bsp_Frame2.h"
#include "bsp_Frame3.h"
#include "zcVRudder.h"

bsp_Frame1& getf1(); 
bsp_Frame2& getf2(); 
bsp_Frame3& getf3(); 
VRudder& getvrudderL();
VRudder& getvrudderR();

void callbsp(Mesh& mesh);
