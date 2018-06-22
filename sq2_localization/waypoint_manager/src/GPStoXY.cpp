//
//
//

#include "waypoint_manager/GPStoXY.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;

const double _GPStoXY::D2P=M_PI/180.0;
//---------------------------------------------------------------------------
// Constractor
//---------------------------------------------------------------------------
_GPStoXY::_GPStoXY(double _lon0, double _lat0)
:lon0(_lon0),lat0(_lat0),x(0),y(0){
}

//---------------------------------------------------------------------------
// void Convert(double longitude, double latitude)
//---------------------------------------------------------------------------
void
_GPStoXY::Convert(double longitude, double latitude){

	pre_x=x;	pre_y=y;
	
	x = zahyou_x(latitude, longitude, lat0,lon0);
	y = zahyou_y(latitude, longitude, lat0,lon0);
}

//---------------------------------------------------------------------------
// double Dist()
//---------------------------------------------------------------------------
double
_GPStoXY::Dist(){
	return sqrt((x-pre_x)*(x-pre_x)+(y-pre_y)*(y-pre_y));
}

//---------------------------------------------------------------------------
// double Direct()
//---------------------------------------------------------------------------
double
_GPStoXY::Direct(){
	return atan2((y-pre_y),(x-pre_x));
}


//----------------------------------------------------------------------------
// del_ramuda(double keido,double kei0)
//----------------------------------------------------------------------------
double
_GPStoXY::del_ramuda(double keido,double kei0)
{
	double del_keido = 0;
	del_keido = keido - kei0;
	
	return del_keido;
}

//----------------------------------------------------------------------------
// double TT(double ido)
//----------------------------------------------------------------------------
double
_GPStoXY::TT(double ido)
{
	double i = 0;
	
	i = tan(ido);
	
	return i;
}


//----------------------------------------------------------------------------
// double SSS(double ido)
//----------------------------------------------------------------------------
double
_GPStoXY::SSS(double ido)
{
	double SS = 0;
	double A = 6378137;
	double F = 298.257222101;
	double e = (sqrt(2*F-1))/F;
	double AAA = A*(1 - e*e);

	
	SS = AAA*(1 + 3/4*pow(e,2)+45/64*pow(e,4)+175/256*pow(e,6)+11025/16384*pow(e,8)+43659/65536*pow(e,10)+693693/1048576*pow(e,12)
	      //+19324305/29360128*pow(e,14)+4927697775/7516192768*pow(e,16))*ido
	     +0.6581819057464599609375*pow(e,14)+0.655610882677137851715087890625*pow(e,16))*ido
	     -AAA*(3/4*pow(e,2)+15/16*pow(e,4)+525/512*pow(e,6)+2205/2048*pow(e,8)+72765/65536*pow(e,10)+297297/262144*pow(e,12)
	       +135270135/117440512*pow(e,14)+547521975/469762048*pow(e,16))/2*sin(2*ido)
	     +AAA*(15/64*pow(e,4)+105/256*pow(e,6)+2205/4096*pow(e,8)+10395/16384*pow(e,10)+1486485/2097152*pow(e,12)
	       +45090045/58720256*pow(e,14)+766530765/939524096*pow(e,16))/4*sin(4*ido)
	     -AAA*(35/512*pow(e,6)+315/2048*pow(e,8)+31185/131072*pow(e,10)+165165/524288*pow(e,12)+45090045/117440512*pow(e,14)+209053845/469762048*pow(e,16))/6*sin(6*ido)
	     +AAA*(315/16384*pow(e,8)+3465/65536*pow(e,10)+99099/1048576*pow(e,12)+4099095/29360128*pow(e,14)+348423075/1879048192*pow(e,16))/8*sin(8*ido)
	     -AAA*(693/131072*pow(e,10)+9009/524288*pow(e,12)+4099095/117440512*pow(e,14)+26801775/469762048*pow(e,16))/10*sin(10*ido)
	     +AAA*(3003/2097152*pow(e,12)+315315/58720256*pow(e,14)+11486475/939524096*pow(e,16))/12*sin(12*ido)
	     //-AAA*(45045/117440512*pow(e,14)+765765/469762048*pow(e,16))/14*sin(14*ido)
	     -AAA*(0.000383555889129638671875*pow(e,14)+0.00163011252880096435546875*pow(e,16))/14*sin(14*ido)
	     //+AAA*(765765/7516192768*pow(e,16))/16*sin(16*ido);
	     +AAA*(0.000101882*pow(e,16))/16*sin(16*ido);
	return SS;
}



//------------------------------------------------------------------------------
// double NN(double ido)
//------------------------------------------------------------------------------
double
_GPStoXY::NN(double ido)
{
	double j = 0;

	double A = 6378137;
	double F = 298.257222101;
	double e2 = (sqrt(2*F-1))/(F-1);

	
	j = ((A*F/(F-1))/sqrt(1+pow(e2,2)*(pow(cos(ido),2))));
	
	return j;
}


//------------------------------------------------------------------------------
// double _GPStoXY::nnyu(double ido)
//------------------------------------------------------------------------------
double
_GPStoXY::nnyu(double ido)
{
	double k = 0;

	double F = 298.257222101;
	double e2 = (sqrt(2*F-1))/(F-1);

	
	k = e2*cos(ido);
	
	return k;
}


//------------------------------------------------------------------------------
// double zahyou_y(double ido,double kei,double ido0,double kei0)
//------------------------------------------------------------------------------
double
_GPStoXY::zahyou_y(double ido,double kei,double ido0,double kei0)
{
	double x, S, So, N, sig_ramuda,fai,t,nyu;
	ido0 = ido0 * D2P;

	
	ido = ido * D2P;
	S = SSS(ido);
	So = SSS(ido0);
	N = NN(ido);
	sig_ramuda = del_ramuda(kei,kei0)*D2P;
	fai = ido;
	t = TT(ido);
	nyu = nnyu(ido);
	
	x = ((S - So) + N*pow(cos(fai),2)*t*pow(sig_ramuda,2)/2 + N/24*pow(cos(fai),4)*t*(5-pow(t,2)+9*pow(nyu,2)+4*pow(nyu,4))*(pow(sig_ramuda,4))
	      -N/720*pow(cos(fai),6)*t*(-61+58*pow(t,2)-pow(t,4)-270*pow(nyu,2)+330*pow(t,2)*pow(nyu,2))*(pow(sig_ramuda,6))
	      -N/40320*pow(cos(fai),8)*t*(-1385+3111*pow(t,2)-543*pow(t,4)+pow(t,6))*pow(sig_ramuda,8))*0.9999;

	return x;
}



//------------------------------------------------------------------------------
// double zahyou_x(double ido,double kei,double ido0,double kei0)
//------------------------------------------------------------------------------
double 
_GPStoXY::zahyou_x(double ido, double kei,double ido0,double kei0)
{
	double y ,N, sig_ramuda,fai,t,nyu;

	ido = ido * D2P;
	
	N = NN(ido);
	sig_ramuda = del_ramuda(kei,kei0)*D2P;
	fai = ido;
	t = TT(ido);
	nyu = nnyu(ido);
	
	y = (N * cos(fai)*sig_ramuda - N/6*pow(cos(fai),3)*(-1 + pow(t,2) -pow(nyu,2))*pow(sig_ramuda,3)
		-N/120*pow(cos(fai),5)*(-5 + 18*pow(t,2) -pow(t,4) -14*pow(nyu,2) +58*pow(t,2)*pow(nyu,2))*pow(sig_ramuda,5)
		-N/5040*pow(cos(fai),7)*(-61 +479*pow(t,2) -179*pow(t,4) +pow(t,6))*pow(sig_ramuda,7))*0.9999;
		
	return y;
}

