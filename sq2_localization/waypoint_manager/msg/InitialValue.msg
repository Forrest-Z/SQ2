Header header

#地図上ある一点の緯度・経度およびピクセル座標(U, V)
float64 Latitude	
float64 Longitude
int32 U
int32 V
float64 OFFSET#地磁気のoffset

#スタート位置の緯度・経度
float64 InLat
float64 InLon
float64 IniPoseX
float64 IniPoseY
float64 IniAz
#生田か筑波かを判断するフラグ
int32 LocateFlag#0 -> IKUTA  1 -> TSUKUBA
int32 course_num
