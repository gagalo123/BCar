#include"protool.h"
#include"speed.h"
#include"motor.h"
#include<stdio.h>
#include"run_main.h"
struct TargetPoint target;

char protool[124];
int protool_length;
short turnning;
extern int TurnValue;

short moving;
int FindNthComma(int n)
{
	int i = 0;
	while(n > 0 && i < protool_length)
	{
		if(protool[i] == ',') --n;
		++i;
	}
	return i - 1;
}
void ProcessProtool(void)
{
	u8 fg = 0,neg = 0;
	double x = 0, y = 0, base = 0.1;
	printf("r%s\n", protool);
	int pos, i, j;
	float sp;
	
	switch(protool[0])
	{
		case '0':
			MotorTurnStill();
			break;
		case '1':
			MotorTurnForward();
			break;
		case '2':
			MotorTurnBackward();
			break;
		case '3':
			MotorTurnLeft(1000);
			break;
		case '4':
			MotorTurnRight(1000);
			break;
		case '5':
			MotorTurnSLeft(340);
			break;
		case '6':
			MotorTurnSRight(340);
			break;
		case '7':
			
			pos = FindNthComma(1);
			sp = 0;
			for(i = pos + 1; protool[i] != ','; ++i)
			{
					sp = sp * 10 + protool[i] - '0';
			}
			MotorChangeSpeed(sp / 10);
			break;
		case '8':

			
			pos = FindNthComma(1);
			neg = 0;
			for(i = pos + 1; protool[i] != ','; ++i)
			{
				if(protool[i] == '-')
				{
						neg = 1;
						continue;
				}
				if(protool[i] == '.')
				{
					fg = 1;
					continue;
				}
				if(fg)
				{
					x += base * (protool[i] - '0');
					base *= 0.1;
				}
				else
				{
					x = x * 10 + protool[i] - '0';
				}
			}
			if(neg) x *= -1;
			neg = 0;
			fg = 0;
			base = 0.1;
			for(i = i + 1; protool[i] != ','; ++i)
			{
			if(protool[i] == '-')
				{
						neg = 1;
						continue;
				}
				if(protool[i] == '.')
				{
					fg = 1;
					continue;
				}
				if(fg)
				{
					y += base * (protool[i] - '0');
					base *= 0.1;
				}
				else
				{
					y = y * 10 + protool[i] - '0';
				}
			}
			if(neg) y *= -1;
			x /= 100;
			y /= 100;
//			printf("go to Point(%f,%f)", x, y);
			target.x[0] = x;
			if(target.x[0] > 0) y = -y;
			target.x[1] = y;
			target.start = 1;
			target.cx =  0;
			break;
	}
}