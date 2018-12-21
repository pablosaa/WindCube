#include "windcube.h"

template<typename T>
std::vector<T> String2Vector(std::string VARIN){
  std::vector<T> VEC;
  std::istringstream ss(VARIN);
  std::copy(std::istream_iterator<T>(ss),std::istream_iterator<T>(),std::back_inserter(VEC));
  return(VEC);
}


template void ReadWindCubeLidar(std::string, V2LidarRTD &);
template void ReadWindCubeLidar(std::string, V2LidarSTA &);

template<typename V2Lidar>
void ReadWindCubeLidar(std::string FileName, V2Lidar &KK){

  int NHead = -99;
  int NVAR_ALT = 0;
  bool RTDTYPE = false, STATYPE = false;
  std::vector<std::string> list, value;
  std::string garbage;
  std::string header;
  std::vector<float> Hm;

  // Open File to read V2 Lidar data files:
  std::fstream in;
  in.open(FileName.c_str(),std::ios::in);

  // Reading the Headers part:
  std::getline(in,header,'=');
  in>>NHead;
  for(int i=0,j=0;i<NHead;++i){
    std::getline(in,garbage);
    int idx = garbage.find("=");
    if(idx==-1) continue;

    // Converting the Altitude Profile values into a vector:
    //if(std::find(garbage.begin(),garbage.end(),"Altitudes (m)")!=garbage.end()){
    if(garbage.find("Altitudes (m)")!=-1){
      Hm = String2Vector<float>(garbage.substr(idx+1,garbage.length())); //(value.back());
      continue;
    }
    
    list.push_back(garbage.substr(0,idx));
    value.push_back(garbage.substr(idx+1,garbage.length()));
      
  }
  const size_t Nalt = Hm.size();
  // Finish to read the Headers.
  
  std::getline(in,garbage);   // getting out **************
  std::getline(in,garbage);   // getting out Profile Headers

  // Defining storage variables for the column data;
  std::vector<std::string> Datum, Uhrzeit;
  std::vector<float> Position;
  std::vector<float> Temperature;
  std::vector<float> Alpha, Beta, Gamma;
  //- std::array<std::vector< std::vector<float> >, N_ALTITUDE> WIND_DATA;
  std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA;
  // auxiliary variables to read every row:
  char date[10];
  char hour[8];
  char pos[3];
  float T, Euler[3];
  char wiper;
  int j=0;

  RTDTYPE = std::is_same<V2Lidar,V2LidarRTD>::value;
  STATYPE = std::is_same<V2Lidar,V2LidarSTA>::value;

  if(RTDTYPE) NVAR_ALT = 8;
  if(STATYPE) NVAR_ALT = 11;
  
  // Starting to read Time- and Altitude-dependent variables:
  while(!in.eof()){
    std::getline(in,garbage);
    std::stringstream ss(garbage);
    // Casting stream into auxiliary variables:
    if(RTDTYPE)
      ss>>date>>hour>>pos>>T>>wiper>>Euler[0]>>Euler[1]>>Euler[2];
    else if(STATYPE)
      ss>>date>>hour>>T>>Euler[0]>>Euler[1]>>Euler[2]>>wiper>>pos;
    else{
      std::cout<<"ERROR: unknow LIDAR variable type passed to Template!";
      KK = {};
      return;
    }
    // Assigning auxiliary variables into vectors:
    Datum.push_back(date);
    Uhrzeit.push_back(hour);
    Position.push_back(std::strcmp(pos,"V")?atof(pos):-1);
    Temperature.push_back(T);
    Alpha.push_back(Euler[0]);  // for STA is External Temperature [C]
    Beta.push_back(Euler[1]);   // for STA is External Pressure [hPa]
    Gamma.push_back(Euler[2]);  // for STA is External RH [%]

    // Reading set of eight-columns for every altitude:
    std::array<std::vector<float>, N_ALTITUDE> AltWind;
    std::istream_iterator<float> ii(ss);
    for(int i=0;i<Nalt;++i){
      std::vector<float> TMP;
      for(int j=0; j<NVAR_ALT; ++j) TMP.push_back(*ii++);

      AltWind.at(i) = TMP;
    }
    WIND_DATA.push_back(AltWind);
    
  
  }  // end loop over Time stamps (in File)
 
  if(RTDTYPE)
    KK = {list,value,Datum,Uhrzeit,Position,Temperature,Alpha,Beta,Gamma,Hm,WIND_DATA};
  if(STATYPE)
    KK = {list,value,Datum,Uhrzeit,Temperature,Alpha,Beta,Gamma,Position,Hm,WIND_DATA};
 
}
// ************* END OF READING SUBROUTINE *****************************


// Simple subroutine to display on screen part of the data:
template void PrintV2Lidar(std::string, V2LidarSTA &);
template void PrintV2Lidar(std::string, V2LidarRTD &);

template<typename V2Lidar>
void PrintV2Lidar(std::string FileName,V2Lidar &KK){
  size_t ND = KK.Height.size();
  std::vector<std::string> Datum = KK.Datum;
  std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA = KK.WIND_DATA;
  std::cout<<FileName<<std::endl;
  for(int j=0; j<KK.HeaderItem.size(); ++j)
    std::cout<<j<<")"<<KK.HeaderItem.at(j)<<"--"<<KK.HeaderValue.at(j)<<std::endl;
  
  std::cout<<ND<<std::endl;
  std::cout<<"Size of all: "<<Datum.size()<<" "<<WIND_DATA.size()<<std::endl;
  
  for(int k=0;k<6;++k){
    std::cout<<Datum.at(k)<<"--"<<KK.Uhrzeit.at(k)<<"--";
    if(std::is_same<V2Lidar,V2LidarRTD>::value)
      std::cout<<" T_rtd:"<<KK.Temperature.at(k)<<std::endl;
    if(std::is_same<V2Lidar,V2LidarSTA>::value)
      std::cout<<" T_sta:"<<KK.Temperature.at(k)<<std::endl;
    for(int i=0;i<4;++i) std::cout<<WIND_DATA[k][0][i]<<" ";
    std::cout<<std::endl;
  }

  std::cout<<"H size:"<<KK.Height.size()<<std::endl;
  std::copy(KK.Height.begin(),KK.Height.end(),std::ostream_iterator<float>(std::cout,", "));
  std::cout<<std::endl;
}

// *********************************************************************
// Subroutine for the extraction of the input file extension,
// the output is an iten number indicating the type of file to read:
// Output:
// 1 <- sta: Statistical Data (processed with compendation altorithm).
// 2 <- stdsta: Standard statistical Data.
// 3 <- rtd: Real Time Data (processed with compendation altorithm).
// 4 <- stdrtd: Standard real time data.
// 0 <- Indicates the input file name is unknown or not yet supported.
unsigned int GetExtensionItem(std::string fname){

  unsigned int EXTITEM=0;
  int idx = fname.find(".");
  std::string auxstr = fname.substr(idx,fname.length());
  if(!strcmp(auxstr.c_str(), ".sta")) EXTITEM = 1;
  else if(!strcmp(auxstr.c_str(), ".stastd")) EXTITEM = 2;
  else if(!strcmp(auxstr.c_str(), ".rtd")) EXTITEM = 3;
  else if(!strcmp(auxstr.c_str(), ".rtdstd")) EXTITEM = 4;
  else std::cout<<"ERROR: Extension not supported!"<<std::endl;
  return(EXTITEM);
}

// ************************************************************************
// Subroutine to convert from Date and Hour from string to array double:
void ConvertWindCube_Date(std::vector<std::string> &inDate, std::vector<std::string> &inHour,double outDate[][6]){
  for(int i=0;i<inDate.size(); ++i){
    outDate[i][0] = atof(inDate[i].substr(0,4).c_str());
    outDate[i][1] = atof(inDate[i].substr(5,6).c_str());
    outDate[i][2] = atof(inDate[i].substr(8,9).c_str());
    outDate[i][3] = atof(inHour[i].substr(0,2).c_str());
    outDate[i][4] = atof(inHour[i].substr(3,5).c_str());
    outDate[i][5] = atof(inHour[i].substr(6,8).c_str());
  }
}

// End of Library.
