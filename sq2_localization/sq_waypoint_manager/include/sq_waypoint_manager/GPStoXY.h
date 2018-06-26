//
//
//


#ifndef GPS_TO_XY_H
#define GPS_TO_XY_H

typedef class _GPStoXY{
	
private:
  static const double D2P;
	double lon0, lat0;
	double x, y;
	double pre_x, pre_y;
	
	double del_ramuda(double keido,double kei0);
	double TT(double ido);
	double NN(double ido);
	double SSS(double ido);
	double nnyu(double ido);
	double zahyou_x(double ido,double kei,double ido0,double kei0);
	double zahyou_y(double ido, double kei,double ido0,double kei0);
public:

	_GPStoXY(double _x0=139.549145, double _y0=35.613415);
	~_GPStoXY(){};

	void SetRoot(double longitude, double latitude);
	
	// Convert GPS coordinate to local coordinate
	void Convert(double longitude, double latitude);
	double Dist();
	double Direct();
	
	// Accessor
	double X(){	return x;}			//ロボット座標に合わせる
	double Y(){	return y;}	

}GPStoXY;


#endif//GPS_TO_XY
