#include <iostream>
#include <cmath>
#include <string>
#include <unistd.h>
#include <stdlib.h>
using namespace std;

struct copter
{
	public:
		void takeoff(string s)
		{
			cout<<s<<" taking off"<<endl;
		}
		void landed(string s)
		{
			cout<<s<<" landed"<<endl;
		}
		void hover(string s)
		{
			cout<<s<<" hovering"<<endl;
		}
};

void showcoor(double x,double y)
{
	cout<<"("<<x<<","<<y<<")";
}

int main()
{
	copter c;char inp;
	while(inp!='q')
	{
		cout<<"Press 'q' to start simulation"<<endl;
		cin>>inp;
	}

	cout<<"Blimp taking off"<<endl;
	
	cout<<"Enter range to perform detection:"<<endl;
	double xmin,xmax,ymin,ymax;
	cout<<"x_min : ";cin>>xmin;
	cout<<"x_max : ";cin>>xmax;
	cout<<"y_min : ";cin>>ymin;
	cout<<"y_max : ";cin>>ymax;	

	cout<<"Blimp is currently at ";showcoor((xmax+xmin)/2,(ymax+ymin)/2);cout<<" performing detection"<<endl;
	usleep(3000000);

	double f1x,f1y,f2x,f2y;
	f1x=xmin + (rand()%(int)(floor(xmax-xmin)));
	cout<<f1x<<endl;
	f2x=xmin+rand()%(int)(floor(xmax-xmin));
	f1y=ymin+rand()%(int)(floor(ymax-ymin));
	f2y=ymin+rand()%(int)(floor(ymax-ymin));


	cout<<"Fire detected at ";showcoor(f1x,f1y);cout<<" and ";showcoor(f2x,f2y);cout<<endl;
	
}