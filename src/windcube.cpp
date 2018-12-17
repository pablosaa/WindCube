#include "windcube.h"

template<typename T>
std::vector<T> String2Vector(std::string VARIN){
  std::vector<T> VEC;
  std::istringstream ss(VARIN);
  std::copy(std::istream_iterator<T>(ss),std::istream_iterator<T>(),std::back_inserter(VEC));
  return(VEC);
}

template void ReadV2Lidar(std::string, V2LidarRTD &);

template<typename V2Lidar>
void ReadV2Lidar(std::string FileName, V2Lidar &KK){

  int NHead=-99;
  int NVAR_ALT;
  bool REALTIME = false;
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
    
    list.push_back(garbage.substr(0,idx));
    value.push_back(garbage.substr(idx+1,garbage.length()));
    std::cout<<j<<","<<idx<<")"<<list.back()<<"--"<<value.back()<<std::endl;
    // Converting the Profile values into a vector:
    if(std::find(list.begin(),list.end(),"Altitudes (m)")!=list.end())
      Hm = String2Vector<float>(value.back());
      
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
  int idx = FileName.find(".");
  std::string auxstr = FileName.substr(idx,FileName.length());
  if(!strcmp(auxstr.c_str(), ".rtd")) {NVAR_ALT = 8; REALTIME = true; std::cout<<"RTD"<<std::endl;}
  if(!strcmp(auxstr.c_str(), ".sta")) {NVAR_ALT = 11;std::cout<<"STA"<<std::endl;}
  if(!strcmp(auxstr.c_str(), ".stdrtd")) {
    NVAR_ALT = 8;
    REALTIME = true;
    std::cout<<"STD RTD"<<std::endl;
  }
  if(!strcmp(auxstr.c_str(), ".stdsta")) {
    NVAR_ALT = 11;
    std::cout<<"STD STA"<<std::endl;
  }
  
  // Starting to read Time- and Altitude-dependent variables:
  while(!in.eof()){
    std::getline(in,garbage);
    std::stringstream ss(garbage);
    // Casting stream into auxiliary variables:
    if(REALTIME)
      ss>>date>>hour>>pos>>T>>wiper>>Euler[0]>>Euler[1]>>Euler[2];
    else
      ss>>date>>hour>>T>>Euler[0]>>Euler[1]>>Euler[2]>>wiper>>pos;

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
  //if(REALTIME)
  KK = {Nalt,Datum,Uhrzeit,Position,Temperature,Alpha,Beta,Gamma,Hm,WIND_DATA};
  //else
    //KK = {Datum,Uhrzeit,Temperature,Alpha,Beta,Gamma,Hm,WIND_DATA};
  //return(KK);
}
// ************* END OF READING SUBROUTINE *****************************


// Simple subroutine to display on screen part of the data:
void PrintV2Lidar(std::string FileName,V2LidarRTD &KK){
  size_t ND = KK.Nalt;
  std::vector<std::string> Datum = KK.Datum;
  std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA = KK.WIND_DATA;
  std::cout<<FileName<<std::endl;
  std::cout<<ND<<std::endl;
  std::cout<<"Size of all: "<<Datum.size()<<" "<<WIND_DATA.size()<<std::endl;
  for(int k=0;k<6;++k){
    std::cout<<Datum.at(k)<<"--"<<KK.Uhrzeit.at(k)<<"--"<<KK.Position.at(k)<<" T:"<<KK.Temperature.at(k)<<std::endl;
    for(int i=0;i<4;++i) std::cout<<WIND_DATA[k][0][i]<<" ";
    std::cout<<std::endl;
  }
  KK.Datum.push_back("susanita");
  std::cout<<"H size:"<<KK.Height.size()<<std::endl;
  std::copy(KK.Height.begin(),KK.Height.end(),std::ostream_iterator<float>(std::cout,", "));
  std::cout<<std::endl;
}

//

// End of Library.
