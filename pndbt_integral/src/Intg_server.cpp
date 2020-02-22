#include "ros/ros.h"
#include<iostream>
#include<cmath>
#include <math.h>
using namespace std;
#include "pndbt_integral/Intg.h"
#include "pndbt_integral/IntgRequest.h"
#include "pndbt_integral/IntgResponse.h"

int iter_simp = 100;

float f_psi(float x)
{
	return  (0.00154375*sin(x)) / (0.0030875*cos(x) + 0.00651975); 	// beta/alpha
}

float simpsons_psi( float a, float b, int n)
{
	float h, x[n+1], sum = 0;
	int j;
	h = (b-a)/n;
	
	x[0] = a;
	
	for(j=1; j<=n; j++)
	{
		x[j] = a + h*j;
	}
	
	for(j=1; j<=n/2; j++)
	{
		sum += f_psi(x[2*j - 2]) + 4*f_psi(x[2*j - 1]) + f_psi(x[2*j]);
	}
	
	return sum*h/3;
}

float int_psi(float s0, float s)
{
	float int1, psi;
	if (s == s0)
		return 1;
	else
	{
		int1 = simpsons_psi( s0,s,iter_simp );
		psi = exp(-2*int1);
		return psi;
	}  
}


float f_fi(float s)
{
	float out;
	out = (0.242307*cos((3*s)/2 - M_PI/2)) / (0.0030875*cos(s) + 0.00651975);  // gamma/alpha  
	return  out; 	
}

float int_fi(float s0, float s)
{
	float fi;
    fi = int_psi(s,s0) * 2 * f_fi(s);
    return fi;
}   


float f_int(float x, float s_0)
{
	float out;
	out = int_fi(s_0, x);
	return 	out; //Define the function f(x)
}


float simpsons_int( float a, float b, int n)
{
	float h, x[n+1], sum = 0;
	int j;
	h = (b-a)/n;
	
	x[0] = a;
	
	for(j=1; j<=n; j++)
	{
		x[j] = a + h*j;
	}
	
	for(j=1; j<=n/2; j++)
	{
		sum += f_int(x[2*j - 2], a) + 4*f_int(x[2*j - 1], a) + f_int(x[2*j], a);
	}
	
	return sum * h/3;
}


bool Integ(pndbt_integral::IntgRequest &req, pndbt_integral::IntgResponse &res)
{
	float int1, I;
	if (req.s == req.s_0)
	{	
    	I = pow(req.s_d, 2) - pow(req.s_d0, 2);
    	res.I = I;
	} else {
		int1 = simpsons_int( req.s_0,req.s,iter_simp );
		I = pow(req.s_d,2) - int_psi(req.s_0, req.s) * (pow(req.s_d0,2) - int1);
	  	res.I = I;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Intg_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("Intg", Integ);
    ROS_INFO("Ready to compute the integral.");
    ros::spin();
    return 0;
}


