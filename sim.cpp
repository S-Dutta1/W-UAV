#include <iostream>
#include <cmath>
#include <string>
#include <unistd.h>
#include <stdlib.h>
using namespace std;


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

void showcoor(double x,double y)
{
	cout<<"("<<x<<","<<y<<")";
}

int main()
{
	string c1="Copter 1";string c2="Copter 2";
	char inp;
	while(inp!='q')
	{
		cout<<"Press 'q' to start simulation"<<endl;
		cin>>inp;
	}
	cout<<"----SIMULATION STARTING----"<<endl;cout<<endl;

	cout<<"Blimp taking off"<<endl;
	
	cout<<"Enter range to perform detection:"<<endl;
	double xmin,xmax,ymin,ymax;
	cout<<"x_min : ";cin>>xmin;
	cout<<"x_max : ";cin>>xmax;
	cout<<"y_min : ";cin>>ymin;
	cout<<"y_max : ";cin>>ymax;	

	usleep(3000000);
	cout<<"Blimp is currently at ";showcoor((xmax+xmin)/2,(ymax+ymin)/2);cout<<" performing detection"<<endl;
	usleep(5000000);cout<<endl;

	double f1x,f1y,f2x,f2y;
	f1x=xmin + (rand()%(int)(floor(xmax-xmin)));
	f2x=xmin+rand()%(int)(floor(xmax-xmin));
	f1y=ymin+rand()%(int)(floor(ymax-ymin));
	f2y=ymin+rand()%(int)(floor(ymax-ymin));


	cout<<"Fire detected at ";showcoor(f1x-(xmax+xmin)/2,f1y-(ymax+ymin)/2);cout<<" and ";showcoor(f2x-(xmax+xmin)/2,f2y-(ymax+ymin)/2);
	cout<<" with respect to blimp"<<endl;cout<<endl;

	takeoff(c1);
	takeoff(c2);
	cout<<endl;

	usleep(5000000);

	cout<<"Copter 1 reached at ";showcoor(f1x,f1y);cout<<endl;
	cout<<"Copter 2 reached at ";showcoor(f2x,f2y);cout<<endl;cout<<endl;
	cout<<"All are set in hovering mode"<<endl;cout<<endl;

	usleep(5000000);

	//int rr=rand()%2+1; // randomly select true fire;
	int rr;
	while(!(rr==1||rr==2))
	{
		cout<<"Choice 1 - ";showcoor(f1x,f1y);cout<<endl;
		cout<<"Choice 2 - ";showcoor(f2x,f2y);cout<<endl;
		cout<<"Select the true alarm :";cin>>rr;cout<<endl;
	}


	if(rr==2)
	{
		cout<<"Fire alarm over ";showcoor(f1x,f1y);cout<<" is false"<<endl;
		cout<<"Copter 1 is moving over to ";showcoor(f2x,f2y);cout<<endl;
		usleep(4000000);
		cout<<endl;
		cout<<"Both copters are monitoring fire over ";showcoor(f2x,f2y);cout<<endl;cout<<endl;
	}
	else
	{
		cout<<"Fire alarm over ";showcoor(f2x,f2y);cout<<" is false"<<endl;
		cout<<"Copter 2 is moving over to ";showcoor(f1x,f1y);cout<<endl;
		usleep(5000000);
		cout<<"Both copters are monitoring fire over ";showcoor(f1x,f1y);cout<<endl;
	}

	usleep(5000000);

	cout<<endl;
	cout<<"Now all UAVs are returning to their base station"<<endl;cout<<endl;
	usleep(5000000);
	cout<<"All UAVs have landed at base station"<<endl;cout<<endl;

	cout<<"----SIMULATION OVER----"<<endl;cout<<endl;
	/////////////////////////
}