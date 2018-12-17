#include "windcube.h"



main(){
  
  std::string fname = "/home/pga082/GFI/data/Legacy/Windcube/WLS866-14_2018_09_15__00_00_00.rtd";
  std::cout<<fname<<std::endl;

  V2LidarRTD VV;
  ReadV2Lidar<V2LidarRTD>(fname, VV);
  PrintV2Lidar(fname,VV);
  std::cout<<"Mudou?"<<VV.Datum.back()<<std::endl;
}   // end of program.


