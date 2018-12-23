// Test-bed program for the WindCube library.
//
// Usage:
// ./read_V2Lidar /home/pga082/GFI/data/Legacy/Windcube/WLS866-14_2018_09_15__00_00_00.rtd
// ./read_V2Lidar /home/pga082/GFI/data/Legacy/Windcube/WLS866-14_2018_09_15__00_00_00.sta
// ./read_V2Lidar /home/pga082/GFI/data/Legacy/Windcube/2018_09_15__12_00_00.gyro

#include "windcube.h"

main(int argc, char *argv[]){

  if(argc>2) {std::cout<<"ERROR!!!"<<std::endl; return 0;}
  //std::string fname = "/home/pga082/GFI/data/Legacy/Windcube/WLS866-14_2018_09_15__00_00_00.rtd"; //rtd";

  std::string fname = argv[1];
  std::cout<<fname<<std::endl;
  
  
  V2LidarSTA STA;
  V2LidarRTD RTD;
  V2Gyro GYRO;

  // trying extension support:
  unsigned int TT = GetExtensionItem(fname);
  std::cout<<"Extension detected: "<<TT<<std::endl;
  
  // Figuring out which kind of file is the input:
  int idx = fname.find(".");
  std::string auxstr = fname.substr(idx,fname.length());
  
  if(!strcmp(auxstr.c_str(), ".sta")){
    //typename std::conditional<true, V2LidarSTA, V2LidarRTD>::type KK;
    ReadWindCubeLidar<V2LidarSTA>(fname, STA);
    PrintV2Lidar<V2LidarSTA>(fname, STA);
    std::cout<<"STA Mudou?"<<STA.WIND_DATA.size()<<std::endl;
  }

  if(!strcmp(auxstr.c_str(), ".rtd")){
    //typename std::conditional<false, V2LidarSTA, V2LidarRTD>::type KK;
    ReadWindCubeLidar<V2LidarRTD>(fname, RTD);
    std::cout<<"RTD"<<std::endl;
    PrintV2Lidar<V2LidarRTD>(fname,RTD);
    double Datum[(int) RTD.Datum.size()][6];

    ConvertWindCube_Date(RTD.Datum,RTD.Uhrzeit, Datum);
    //std::cout<<std::is_object<RTD.RH>::value;
    std::cout<<"RTD Mudou?"<<RTD.WIND_DATA[0][0].size()<<std::endl;
  }

  if(TT==5){
    ReadWindCubeGyro(fname,GYRO);
    std::cout<<"GYRO size: "<<GYRO.Datum.size()<<std::endl;
    for(int k=0;k<7;++k) std::cout<<GYRO.Datum[k]<<" "<<GYRO.Uhrzeit[k]<<" "<<GYRO.Pitch[k]<<" "<<GYRO.Roll[k]<<" "<<GYRO.Yaw[k]<<std::endl;
  }
    
}   // end of program.

  // std::vector<float> V2LidarSTA::*p;
  // std::vector<float> V2LidarRTD::*r;

  // V2LidarSTA *H;
  // std::vector<float> *rr = &H->Pressure;

