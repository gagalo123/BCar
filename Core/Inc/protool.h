#ifndef __PROTOOL_H__
#define __PROTOOL_H__
#include"run_main.h"
extern char protool[124];
extern int protool_length;
extern int CarState;
extern short turnning;
extern short moving;
struct TargetPoint
{
	double x[2];
	double cx;
	u8 start;
};
extern struct TargetPoint target;
void ProcessProtool(void);

#endif