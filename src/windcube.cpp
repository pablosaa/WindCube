// *****************************************************************************
//  LIBRARY TO READ DATA FILE FROM LEO-SPHERE LIDARS
// THE SUPPORTED LIDAR MODELS ARE: V1, V2, ...
// 
// 
// (c) 2018, Pablo Saavedra G.
// pablo.saa@uib.no 
// University of Bergen, Norway
// SEE LICENCE.TXT
// *****************************************************************************

#include "windcube.h"


// *****************************************************************************
// Template Function to convert a sequence of numbers inside a string into a
// vector any type
template<typename T>
std::vector<T> String2Vector(std::string VARIN){
  std::vector<T> VEC;
  std::istringstream ss(VARIN);
  std::copy(std::istream_iterator<T>(ss),std::istream_iterator<T>(),std::back_inserter(VEC));
  return(VEC);
}
// ============= End of Template Function String2Vector ========================



// *****************************************************************************
// Template SUBROUTINE to read V2 Lidar File types: .sta, .rtd
template void ReadWindCubeLidar(std::string, V2LidarRTD &);
template void ReadWindCubeLidar(std::string, V2LidarSTA &);

template<typename V2Lidar>
void ReadWindCubeLidar(std::string FileName, V2Lidar &KK){

  int NHead = -99;
  int NVAR_ALT = 0;
  bool RTDTYPE = false, STATYPE = false;
  bool V1RTDTYPE = false, V1STATYPE = false;
  std::vector<std::string> list, value;
  std::string garbage;
  std::string header;
  std::vector<float> Hm;
  std::string IDsystem;

  // Open File to read V2 Lidar data files:
  std::fstream in;
  in.open(FileName.c_str(),std::ios::in|std::ios::binary);
  if(!in){
    std::cout<<"ERROR: File cannot be open!"<<std::endl;
    return;
  }
  
  // Reading the Headers part:
  std::getline(in,header,'=');
  in>>NHead;
  for(int i=0,j=0;i<NHead;++i){
    std::getline(in,garbage);
    int idx = garbage.find("=");
    if(idx==-1) continue;

    // Converting the Altitude Profile values into a vector:
    //if(std::find(garbage.begin(),garbage.end(),"Altitudes (m)")!=garbage.end()){
    if(garbage.find("Altitudes")!=-1){
      Hm = String2Vector<float>(garbage.substr(idx+1,garbage.length())); //(value.back());
      continue;
    }
    // Checking the Lidar version:
    if(garbage.find("ID System")!=-1){
      //strcpy(IDsystem,garbage.substr(idx+1,garbage.length()).c_str());
      IDsystem = garbage.substr(idx+1,garbage.length());
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
  std::vector<float> Temp, Press, RH, Nwiper;
  //- std::array<std::vector< std::vector<float> >, N_ALTITUDE> WIND_DATA;
  std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA;
  // auxiliary variables to read every row:
  char date[10];
  char hour[8];
  char pos[3];
  float T, Euler[3];
  char temp[5], press[5], rh[5];
  int wiper; //char wiper;
  int j=0;

  RTDTYPE = std::is_same<V2Lidar,V2LidarRTD>::value && !IDsystem.compare(0,6,"WLS866");
  STATYPE = std::is_same<V2Lidar,V2LidarSTA>::value && !IDsystem.compare(0,6,"WLS866");
  V1RTDTYPE = std::is_same<V2Lidar,V2LidarRTD>::value && !IDsystem.compare(0,4,"WLS7");
  V1STATYPE = std::is_same<V2Lidar,V2LidarSTA>::value && !IDsystem.compare(0,4,"WLS7");
  
  if(RTDTYPE || V1RTDTYPE) NVAR_ALT = 8;
  if(STATYPE) NVAR_ALT = 11;
  if(V1STATYPE) NVAR_ALT = 18;
  
  // Starting to read Time- and Altitude-dependent variables:
  while(!in.eof()){
    std::getline(in,garbage);
    std::stringstream ss(garbage);
    // Casting stream into auxiliary variables:
    if(RTDTYPE)
      ss>>date>>hour>>pos>>T>>wiper>>Euler[0]>>Euler[1]>>Euler[2];
    else if(V1RTDTYPE){
      //ss>>date>>hour>>pos>>T>>wiper;
      std::cout<<"ERROR: RTD version 1 LIDAR not yet supported!"<<std::endl;
      KK = {};
      return;
    }
    else if(STATYPE)
      ss>>date>>hour>>T>>temp>>press>>rh>>wiper; //>>pos;
    else if(V1STATYPE)
      ss>>date>>hour>>wiper>>T;
    else{
      std::cout<<"ERROR: unknow LIDAR variable type passed to Template!"<<std::endl;
      KK = {};
      return;
    }
    // Assigning common auxiliary variables into vectors:
    Datum.push_back(date);
    Uhrzeit.push_back(hour);
    Temperature.push_back(T);
    Nwiper.push_back(wiper);    // Number of Wiper for STA and RTD
    // Assigning File-type specific variables: 
    if(RTDTYPE){
      Position.push_back(std::strcmp(pos,"V")?atof(pos):-1);
      Alpha.push_back(Euler[0]);  // for STA is External Temperature [C]
      Beta.push_back(Euler[1]);   // for STA is External Pressure [hPa]
      Gamma.push_back(Euler[2]);  // for STA is External RH [%]
    }
    if(V1RTDTYPE)
      Position.push_back(std::strcmp(pos,"V")?atof(pos):-1);
    
    if(STATYPE){
      Temp.push_back(atof(temp));
      Press.push_back(atof(press));
      RH.push_back(atof(rh));
    }
    if(V1STATYPE){
    }
  
    
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

  // This works only if V2LidarRTD and V2LidarSTA have the same number and type members!
  if(RTDTYPE || V1RTDTYPE)
    KK = {IDsystem,list,value,Datum,Uhrzeit,Hm,WIND_DATA,Position,Temperature,Alpha,Beta,Gamma,Nwiper};
  if(STATYPE)
    KK = {IDsystem,list,value,Datum,Uhrzeit,Hm,WIND_DATA,Temperature,Temp,Press,RH,Nwiper};
  if(V1STATYPE)
    KK = {IDsystem,list,value,Datum,Uhrzeit,Hm,WIND_DATA,Temperature,Temp,Press,RH,Nwiper};
}
// ================ END OF V2 LIDAR READING SUBROUTINE ===============================



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
  int idx = fname.find_last_of(".");
  std::string auxstr = fname.substr(idx,fname.length());
  if(!strcmp(auxstr.c_str(), ".sta")) EXTITEM = 1;
  else if(!strcmp(auxstr.c_str(), ".stdsta")) EXTITEM = 2;
  else if(!strcmp(auxstr.c_str(), ".rtd")) EXTITEM = 3;
  else if(!strcmp(auxstr.c_str(), ".stdrtd")) EXTITEM = 4;
  else if(!strcmp(auxstr.c_str(), ".gyro")) EXTITEM = 5;
  else std::cout<<"ERROR: Extension not supported!"<<std::endl;
  return(EXTITEM);
}


// ************************************************************************
// Subroutine to convert from Date and Hour from string to array double:
void ConvertWindCube_Date(std::vector<std::string> &inDate, std::vector<std::string> &inHour,double outDate[][6]){
  std::cout<<"inside converting date "<<inHour[0].size()<<" "<<inHour[0]<<std::endl;
  for(int i=0;i<inDate.size(); ++i){
    outDate[i][0] = atof(inDate[i].substr(0,4).c_str());
    outDate[i][1] = atof(inDate[i].substr(5,2).c_str());
    outDate[i][2] = atof(inDate[i].substr(8,2).c_str());
    outDate[i][3] = atof(inHour[i].substr(0,2).c_str());
    outDate[i][4] = atof(inHour[i].substr(3,2).c_str());
    if(inHour[i].size()==8)   // case of .sta files:
      outDate[i][5] = atof(inHour[i].substr(6,2).c_str());
    if(inHour[i].size()==11)  // case of .rtd or gyro files
      outDate[i][5] = atof(inHour[i].substr(6,5).c_str());
  }
}
// =========== End of subroutine convert Date string to array ==============


// *************************************************************************
// SUBROUTINE TO READ GYRO DATA FILES
//
void ReadWindCubeGyro(std::string FileName, V2Gyro &KK){

  std::string garbage;
  // auxiliary variables to read every row:
  char date[10];
  char hour[12];
  //std::vector<float> tmp;
  long int Treset;
  float Pitch, Roll, Yaw;
  
  // Open File to read V2 Lidar GYRO data files:
  std::fstream in;
  in.open(FileName.c_str(),std::ios::in);
  if(!in){
    std::cout<<"ERROR: File cannot be open!"<<std::endl;
    return;
  }
  
  std::getline(in,garbage);   // getting out Profile Headers

  while(!in.eof()){
    std::getline(in,garbage);
    // checking that got a line with at least 180 characters:
    if(garbage.length()<180) continue;
    std::stringstream ss(garbage);
    // Casting stream into auxiliary variables:
    ss>>date>>hour>>Treset>>Pitch>>Roll>>Yaw;

    // Assigning auxiliary variables into vectors:
    KK.Datum.push_back(date);
    KK.Uhrzeit.push_back(hour);
    KK.Treset.push_back(Treset);
    KK.Pitch.push_back(Pitch);
    KK.Roll.push_back(Roll);
    KK.Yaw.push_back(Yaw);

    std::istream_iterator<float> ii(ss);
    KK.GPS_heading.push_back(*ii++);
    int i;
    std::array<float,3> tmp;
    for(i=0;i<3;++i) tmp.at(i) = (*ii++);
    KK.SBG_LLA.push_back(tmp);

    for(int i=0;i<3;++i) tmp.at(i) = (*ii++);
    KK.SBG_Vxyz.push_back(tmp);
    
    KK.Temperature.push_back(*ii++);
    KK.Status.push_back(*ii++);

    std::array<float,5> tmp5;
    for(int i=0;i<5;++i) tmp5.at(i) = (*ii++);
    KK.GPS_Accu.push_back(tmp5);
    
    KK.NSat.push_back(*ii++);

    for(int i=0;i<3;++i) tmp.at(i) = (*ii++);
    KK.SBG_Accu.push_back(tmp);

    std::array<float,7> tmp7;
    for(int i=0;i<7;++i) tmp7.at(i) = (*ii++);
    KK.GPS_Time.push_back(tmp7);
    
  }  // end of while !EOF
}
// =========== End of GYRO reading ================================




// *******************************************************************************
// SUBROUTINE to show the GNU Public Lincense notice:
//
void ShowGNUPL(){
  std::cout<<"'WindCubMEX Library'  Copyright (C) 2018  Pablo Saavedra G."<<std::endl;
  std::cout<<"This program comes with ABSOLUTELY NO WARRANTY; for details see GNUPLv3 `LICENCE'."<<std::endl;
  std::cout<<"This is free software, and you are welcome to redistribute it"<<std::endl;
  std::cout<<"under certain conditions; see GNUPLv3 `LICENCE' or <http://www.gnu.org/licenses/> for details.\n"<<std::endl;
  return;
}
// ============= End of GNU Showing Licence =====================================

// ******************************************************************************
// Simple subroutine to display on screen part of the data:
template void PrintV2Lidar(std::string, V2LidarSTA &);
template void PrintV2Lidar(std::string, V2LidarRTD &);

template<typename V2Lidar>
void PrintV2Lidar(std::string FileName,V2Lidar &KK){
  size_t ND = KK.Height.size();
  std::vector<std::string> Datum = KK.Datum;
  std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA = KK.WIND_DATA;
  std::cout<<FileName<<std::endl;
  std::cout<<"Data type and version:"<<KK.IDSystem<<std::endl;
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
// ================== End of Subroutine PrintV2Lidar =================================

// ______________________________________________-
// End of Library.


//  std::vector<std::string>::iterator ii;
//  ii = std::find(list.begin(),list.end(),"ID System");
//  int aa = std::distance(list.begin(),ii);
//  std::cout<<"ID System is in: "<<value.at(aa)<<std::endl;
  
