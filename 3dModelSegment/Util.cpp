#include "Util.h"
int sign(double x)
{
	const double eps = 1e-8;
	if(x>eps)
		return 1;
	else if(x<-eps)
		return -1;
	return 0;
}