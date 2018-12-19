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
  std::vector<float> Position;
  std::vector<float> Temperature;
  std::vector<float> Alpha;
  std::vector<float> Beta;
  std::vector<float> Gamma;
  std::vector<float> Height;
  std::vector<std::array<std::vector<float>, N_ALTITUDE> > WIND_DATA;
};

// Struct variable for the V2 Lidar STA formats (.sta and .stdsta)
struct V2LidarSTA {
  std::vector<std::string> HeaderItem;
  std::vector<std::string> HeaderValue;
  std::vector<std::string> Datum;
  std::vector<std::string> Uhrzeit;
  std::vector<float> Temperature;
  std::vector<float> ExtTemp;
  std::vector<float> Pressure;
  std::vector<float> RH;
  std::vector<float> Vbatt;
  std::vector<float> Height;
  std::vector<std::array<std::vector<float>, N_ALTITUDE> > WIND_DATA;
};

template<typename V2Lidar>
void ReadWindCubeLidar(std::string, V2Lidar &);

extern template void ReadWindCubeLidar(std::string, V2LidarRTD &);
extern template void ReadWindCubeLidar(std::string, V2LidarSTA &);

template<typename V2Lidar>
void PrintV2Lidar(std::string, V2Lidar &);

extern template void PrintV2Lidar(std::string, V2LidarSTA &);
extern template void PrintV2Lidar(std::string, V2LidarRTD &);

#endif

