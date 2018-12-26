#ifndef WINDCUBE__
#define WINDCUBE__

#include<iostream>
#include<cstring>
#include<fstream>
#include<sstream>
#include<vector>
#include<array>
#include<iterator>
#include<algorithm>

#define N_ALTITUDE 12   // Maximum number of altitude levels

// Struct variable for the V2 Lidar RTD formats (.rtd and .stdrtd)
struct V2LidarRTD {
  std::vector<std::string> HeaderItem;
  std::vector<std::string> HeaderValue;
  std::vector<std::string> Datum;
  std::vector<std::string> Uhrzeit;
  std::vector<float> Height;
  std::vector<std::array<std::vector<float>, N_ALTITUDE> > WIND_DATA;
  std::vector<float> Position;
  std::vector<float> Temperature;
  std::vector<float> Alpha;
  std::vector<float> Beta;
  std::vector<float> Gamma;
  std::vector<float> Nwiper;   // experimental!
};

// Struct variable for the V2 Lidar STA formats (.sta and .stdsta)
struct V2LidarSTA {
  std::vector<std::string> HeaderItem;
  std::vector<std::string> HeaderValue;
  std::vector<std::string> Datum;
  std::vector<std::string> Uhrzeit;
  std::vector<float> Height;
  std::vector<std::array<std::vector<float>, N_ALTITUDE> > WIND_DATA;  
  std::vector<float> Temperature;
  std::vector<float> ExtTemp;
  std::vector<float> Pressure;
  std::vector<float> RH;
  std::vector<float> Nwiper;
  std::vector<float> DUMMY;  // Dummy variable only used to pair with other struct variable.
};

// Struct variable for the V2 Lidar Gyro data format (.gyro)
struct V2Gyro {
  std::vector<std::string> Datum;
  std::vector<std::string> Uhrzeit;
  std::vector<int> Treset;
  std::vector<float> Pitch;
  std::vector<float> Roll;
  std::vector<float> Yaw;
  std::vector<float> GPS_heading;
  std::vector<std::array<float, 3> > SBG_LLA;
  std::vector<std::array<float, 3> > SBG_Vxyz;
  std::vector<float> Temperature;
  std::vector<int> Status;
  std::vector<std::array<float, 5> > GPS_Accu;
  std::vector<int> NSat;
  std::vector<std::array<float, 3> > SBG_Accu;
  std::vector<std::array<float, 7> > GPS_Time;
};

template<typename V2Lidar>
void ReadWindCubeLidar(std::string, V2Lidar &);

extern template void ReadWindCubeLidar(std::string, V2LidarRTD &);
extern template void ReadWindCubeLidar(std::string, V2LidarSTA &);

template<typename V2Lidar>
void PrintV2Lidar(std::string, V2Lidar &);

extern template void PrintV2Lidar(std::string, V2LidarSTA &);
extern template void PrintV2Lidar(std::string, V2LidarRTD &);

unsigned int GetExtensionItem(std::string);

void ConvertWindCube_Date(std::vector<std::string> &, std::vector<std::string> &, double [][6]);

void ReadWindCubeGyro(std::string, V2Gyro &);

void ShowGNUPL();

#endif

