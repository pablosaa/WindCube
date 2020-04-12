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

// -----------------------------------------------
// Function to add seconds to the Initial Base Time
// time_ms type variable (e.g.T0 = dd/mm/yyyy HH:MM:SS.sss)
// the functions will return a new time_ms added to T0 the
// number of seconds. NOTE: T0 does not change.
time_ms AddSeconds(time_ms T0, float secs){

  time_ms Zeit = T0;
  int tmp = (int) secs;
  Zeit.sec = T0.sec + tmp;
  Zeit.msecs = T0.msecs + (secs - (float) tmp)*1000;
  if(Zeit.msecs>999) {Zeit.msecs -= 1000; Zeit.sec++; }
  if(Zeit.sec>59) {Zeit.sec -= 60; Zeit.min++;}
  if(Zeit.min>59) {Zeit.min -= 60; Zeit.hour++;}
  if(Zeit.hour>23) {Zeit.hour -= 24; Zeit.day++;}
  return(Zeit);
}
// ----/

// --------------------------------------
// Template to convert to Big-endian byte
template<typename T>
T BigEndian(T xx){
  char *cptr = (char *) &xx;
  char tmp = cptr[0];
  if(std::is_same<T,uint16_t>::value || std::is_same<T,int16_t>::value){
    cptr[0] = cptr[1];
    cptr[1] =tmp;
  }
  if(std::is_same<T,uint32_t>::value){
    cptr[0] = cptr[3];
    cptr[3] =tmp;
    tmp = cptr[1];
    cptr[1] = cptr[2];
    cptr[2] =tmp;
  }
  return(xx);
}
// ----/

// -----------------------------------------------------
// Function to vectorize V1 Lidar profile (one per time)
std::vector<float> MapV1ProfileData(size_t Nh, size_t Nvar,
				    int16_t DATA[], float Gain,
				    float CNRthr){
  std::vector<float> PRO;
  
  for(size_t i=0; i<Nh; ++i){
    bool VALID = true;
    float SNR=NAN, WS=NAN, WD=NAN, U=NAN, V=NAN;
    for(size_t k=0; k<Nvar; ++k){
  
      float XX = (float) BigEndian(DATA[i + k*Nh]);
      XX /= Gain;
      // Checking of CNR is below Threshold or > 70dB:
      if(k==0){
	SNR = XX - CNRthr;
	if(XX <= CNRthr || XX>30) VALID = false;
      }
      
      // Checking Threshold RWS/CNR>1:
      if(k==1 && abs(XX/SNR)>1) VALID = false;

      if(k==2) XX /= 10.0;
      // Checking for Horizontal wind speed:
      if(k==3){
	WS = XX;
	if(XX<-500 || XX>50) VALID = false;
      }
      
      if(k==4){
	// For Wind direction:
	if(XX>=0 && XX<2*PI) {
	  WD = XX;
	  XX *= 180.0/PI;  // Azimuth [DEG]
	}
	else VALID = false;
      }
      if(k==5) {
	// Calculating U and V components
	U = WS*cos(WD);  // Wind U [m/s]
	V = WS*sin(WD);  // Wind V [m/s]

	// storing U, V, and then W:=XX(k=5)
	PRO.push_back(U);
	PRO.push_back(V);
      }
      PRO.push_back(XX);
    }
    if(!VALID) std::fill(PRO.begin()+i*Nvar, PRO.end(), NAN);
  }
  
  return(PRO);
}
// ----/

// --------------------------------------------------------
// Template to retrieve the value of the Header list:

void RetrieveValue(std::string V, float &xx){xx = (float) atof(V.c_str());}
void RetrieveValue(std::string V, std::string &xx){ xx = V;}
void RetrieveValue(std::string V, time_ms &tt){
  // get numbers: {0,3,6,11,14,17} date string= "07/11/2019 01:00:00.49"
  if(V.find_first_of("/")==4){
    tt.year  = atoi(V.substr(0,4).c_str());
    tt.month = atoi(V.substr(5,2).c_str());
    tt.day   = atoi(V.substr(8,2).c_str());
  }
  else if(V.find_first_of("/")==2){
  tt.day   = atoi(V.substr(0,2).c_str());
  tt.month = atoi(V.substr(3,2).c_str());
  tt.year  = atoi(V.substr(6,4).c_str());
  }
  else std::cout<<"ERROR Time Stamp unrecognized! "<<V<<std::endl;
  tt.hour  = atoi(V.substr(11,2).c_str());
  tt.min   = atoi(V.substr(14,2).c_str());
  float tmp   = atof(V.substr(17).c_str());
  tt.sec = (int) tmp;
  tt.msecs  = (tmp - (tt.sec))*1000;  // msec
}
void RetrieveValue(std::string V, std::vector<float> &Hvec){
  std::istringstream ss(V);
  std::copy(std::istream_iterator<float>(ss),std::istream_iterator<float>(),std::back_inserter(Hvec));
}

template<typename T>
T GetHeaderValue(std::vector<std::string> List,
		 std::vector<std::string> Value,
		 const char VAR[]){

  T OUT;

  std::vector<std::string>::iterator ii;
  
  ii = find_if(List.begin(), List.end(),
   	       [=](const std::string& str) {
   		 return str.find(VAR) != std::string::npos; });

  if(ii == List.end())
    std::cout<<" ERROR "<<VAR<<" not found!!!"<<std::endl;
  else{
    size_t idx = distance(List.begin(), ii);
    RetrieveValue(Value.at(idx), OUT);
  }
  return(OUT);
}
// ----/



// *****************************************************************************
// Template SUBROUTINE to read V2 Lidar File types: .sta, .rtd
template void ReadWindCubeLidar(std::string, V2LidarRTD &);
template void ReadWindCubeLidar(std::string, V2LidarSTA &);

template<typename V2Lidar>
void ReadWindCubeLidar(std::string FileName, V2Lidar &KK){

  size_t NHead = -99;
  size_t NVAR_ALT = 0;
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
  
  // Getting number of lines of Metadata:
  std::getline(in,header,'=');
  in>>NHead;
  // Reading the Headers part:
  for(int i=0, j=0; i<NHead; ++i){
    std::getline(in,garbage);
    size_t idx = garbage.find("=");
    if(idx==-1) continue;

    // Converting the Altitude Profile values into a vector:
    //if(std::find(garbage.begin(),garbage.end(),"Altitudes (m)")!=garbage.end()){
    if(garbage.find("Altitudes")!=-1){
      RetrieveValue(garbage.substr(idx+1,garbage.length()), Hm);
      continue;
    }
    // Checking the Lidar version:
    if(garbage.find("ID System")!=-1){
      IDsystem = garbage.substr(idx+1,garbage.length());
    }
    
    list.push_back(garbage.substr(0,idx));
    value.push_back(garbage.substr(idx+1,garbage.length()));
    
  }
  // Finish to read the Headers.
  in.ignore(300,'\n');   // getting out **************
  
  RTDTYPE = std::is_same<V2Lidar,V2LidarRTD>::value && !IDsystem.compare(0,6,"WLS866");
  STATYPE = std::is_same<V2Lidar,V2LidarSTA>::value && !IDsystem.compare(0,6,"WLS866");
  V1RTDTYPE = std::is_same<V2Lidar,V2LidarRTD>::value && !IDsystem.compare(0,4,"WLS7");
  V1STATYPE = std::is_same<V2Lidar,V2LidarSTA>::value && !IDsystem.compare(0,4,"WLS7");

  if(!V1RTDTYPE) std::getline(in,garbage); // dismiss Profile Headers
         
  // Defining storage variables for the column data;
  const size_t Nalt = Hm.size();
  std::vector<time_ms> DieZeit;
  std::vector<float> Position;
  std::vector<float> Temperature;
  std::vector<float> Alpha, Beta, Gamma;
  std::vector<float> Temp, Press, RH, Nwiper;
  std::vector<std::vector<float> > DATA;
  // auxiliary variables to read every row:
  std::string date;
  std::string hour;

  float Gain, RIP, CNRthr;
  time_ms Dag0;
  float T;
  int wiper; //char wiper;

  if(RTDTYPE || V1RTDTYPE) NVAR_ALT = 8;
  if(STATYPE) NVAR_ALT = 11;
  if(V1STATYPE) NVAR_ALT = 18;
  if(V1RTDTYPE){
    NVAR_ALT = 6;
    Gain = GetHeaderValue<float>(list, value, "Gain");
    CNRthr = GetHeaderValue<float>(list, value, "CNRThreshold");
    RIP = GetHeaderValue<float>(list, value, "RelativeInitialPosition");
    Dag0 = GetHeaderValue<time_ms>(list, value, "InitialDate/Time");
    // std::cout<<"Gain= "<<Gain<<" RIP="<<RIP<<" CNRthr="<<CNRthr<<std::endl;
    // std::cout<<" TT "<<Dag0.day<<"- "<<Dag0.month<<"-"<<Dag0.year;
    // std::cout<<" "<<Dag0.hour<<":"<<Dag0.min<<":"<<Dag0.sec<<"."<<Dag0.msecs<<std::endl;

  }
  //std::cout<<" Altitudes:"<<Nalt<<std::endl;

  // Starting to read Time- and Altitude-dependent variables:
  while(!in.eof()){

    // Casting stream into auxiliary variables:
    if(V1RTDTYPE){
      uint32_t mytime;
      uint16_t T2m;
      uint8_t mywiper;
      int32_t dim1, dim2;
      
      in.read((char *) &mytime, sizeof(mytime));
      in.read((char *) &T2m, sizeof(T2m) );
      in.read((char *) &mywiper, sizeof(mywiper) );
      in.read((char *) &dim1, sizeof(dim1) );
      in.read((char *) &dim2, sizeof(dim2) );

      time_ms qq = AddSeconds(Dag0, ((float) BigEndian(mytime))/10);
      DieZeit.push_back(qq);
      
      T = 0.1*((float) BigEndian(T2m));
      wiper = (int) BigEndian(mywiper);

      // Reading set of eight-columns for every altitude:
      size_t Nkakes = Nalt*NVAR_ALT;
      int16_t kakes[Nkakes];
      in.read((char *) kakes, sizeof(kakes));

      std::vector<float> TMP = MapV1ProfileData(Nalt, NVAR_ALT, kakes, Gain, CNRthr);
      DATA.push_back(TMP);
      
      // std::for_each( TMP.begin(),TMP.end(),[](float v){std::cout << v << " ";});
      // std::cout<<std::endl;
      
      Position.push_back(RIP);
      Temperature.push_back(T);
      Nwiper.push_back(wiper);
      Alpha.push_back(-99);  // for STA is External Temperature [C]
      Beta.push_back(-99);   // for STA is External Pressure [hPa]
      Gamma.push_back(-99);

      // Adding +90deg clockwise to Ray Initial Position :
      RIP += RIP!=270?90:-270;
      continue;
    }  // End of block for V1 RTD type.

    // starting block for RTD V2 and STA V1 & V2 Lidars:
    time_ms Dage;
    std::getline(in,garbage);
    if(garbage.length()<40) continue;
    RetrieveValue(garbage.substr(0,22), Dage);
    std::stringstream ss(garbage);
    
    if(RTDTYPE){
      char pos[3];
      float Euler[3];
      ss>>date>>hour>>pos>>T>>wiper>>Euler[0]>>Euler[1]>>Euler[2];
      Position.push_back(std::strcmp(pos,"V")?atof(pos):-1);
      Alpha.push_back(Euler[0]);  // for STA is External Temperature [C]
      Beta.push_back(Euler[1]);   // for STA is External Pressure [hPa]
      Gamma.push_back(Euler[2]);  // for STA is External RH [%]
    }
    else if(STATYPE){
      char temp[5], press[5], rh[5];
      ss>>date>>hour>>T>>temp>>press>>rh>>wiper; //>>pos;
      Temp.push_back(atof(temp));
      Press.push_back(atof(press));
      RH.push_back(atof(rh));
    }
    else if(V1STATYPE)
      ss>>date>>hour>>wiper>>T;
    else{
      std::cout<<"ERROR: unknow LIDAR type passed to Template!"<<std::endl;
      KK = {};
      return;
    }
    
    // Assigning common auxiliary variables into vectors:
    // Common variables
    DieZeit.push_back(Dage);
    Temperature.push_back(T);
    Nwiper.push_back(wiper);    // Number of Wiper for STA and RTD

    // Reading set of eight-columns for every altitude:
    std::istream_iterator<std::string> ii(ss);
    std::vector<float> TMP;
    for(int i=0;i<Nalt;++i)
      for(int j=0; j<NVAR_ALT; ++j)
	TMP.push_back(atof((*ii++).c_str()));
    
    DATA.push_back(TMP); //AltWind);
  }
  // end loop over Time stamps (in File) [while(!in.eof())...]
  in.close();

  // This works only if V2LidarRTD and V2LidarSTA have the same number and type members!
  //  KK = {IDsystem,list,value,Datum,Uhrzeit,Hm,DATA,Position,Temperature,Alpha,Beta,Gamma,Nwiper};
  if(RTDTYPE || V1RTDTYPE)
    KK = {IDsystem,list,value,DieZeit,Hm,DATA,Position,Temperature,Alpha,Beta,Gamma,Nwiper};
  if(STATYPE)
    KK = {IDsystem,list,value,DieZeit,Hm,DATA,Temperature,Temp,Press,RH,Nwiper};
  if(V1STATYPE)
    KK = {IDsystem,list,value,DieZeit,Hm,DATA,Temperature,Temp,Press,RH,Nwiper};

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
//void ConvertWindCube_Date(std::vector<std::string> &inDate, std::vector<std::string> &inHour,double outDate[][6]){
void ConvertWindCube_Date(std::vector<time_ms> &inDate, double outDate[][6]){
  std::cout<<"inside converting date "<<inDate.size()<<std::endl;
  for(int i=0;i<inDate.size(); ++i){

    //std::string d = inDate[i];
    time_ms d = inDate.at(i);
    //std::string h = inHour[i];
    std::string::size_type n;
    bool YrLast = false;      // false := YEAR/MONTH/DAY. true := DAY/MONTH/YEAR format

    outDate[i][0] = (double) d.year;
    outDate[i][1] = (double) d.month;
    outDate[i][2] = (double) d.day;
    outDate[i][3] = (double) d.hour;
    outDate[i][4] = (double) d.min;
    outDate[i][5] = (double) d.sec + d.msecs/1000;
  }
    // Checking whether Date is longer than 10 characters:
  //   if(d.size()>10) d = d.substr(d.size()-10);

  //   for (int j=0, k=2; j<3; j++, k--){
  //     // For Hour:
  //     n = h.find_last_of(":");
  //     if(j==2 && n!=std::string::npos)
  // 	std::cout<<"ERROR!! hour: "<<inDate[i]<<"-"<<inHour[i]<<std::endl;
  //     outDate[i][k+3] = atof(h.substr(n+1).c_str());
  //     h = h.substr(0,n); //n+1);

  //     // For Date:
  //     n = d.find_last_of("/");
  //     if(j==2 && n!=std::string::npos)
  // 	std::cout<<"ERROR!! hour: "<<inDate[i]<<"-"<<inHour[i]<<std::endl;

  //     if(d.substr(n+1).size()==4 && j==0) YrLast = true;
  //     outDate[i][YrLast?j:k] = atof(d.substr(n+1).c_str());
  //     d = d.substr(0,n);
  //   }

  // }
}
// =========== End of subroutine convert Date string to array ==============


// *************************************************************************
// SUBROUTINE TO READ GYRO DATA FILES
//
void ReadWindCubeGyro(std::string FileName, V2Gyro &KK){

  std::string garbage;
  // auxiliary variables to read every row:
  std::string date;
  std::string hour;
  time_ms Dage;
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
    std::getline(in, garbage);

    // checking that got a line with at least 180 characters:
    if(garbage.length()<180) continue;

    // Converting Time Stamp to time_ms format:
    RetrieveValue(garbage.substr(0,22), Dage);
    std::stringstream ss(garbage);
    // Casting stream into auxiliary variables:
    ss>>date>>hour>>Treset>>Pitch>>Roll>>Yaw;

    // Assigning auxiliary variables into vectors:

    KK.Uhrzeit.push_back(Dage); //hour);
    KK.Treset.push_back(Treset);
    KK.Pitch.push_back(Pitch);
    KK.Roll.push_back(Roll);
    KK.Yaw.push_back(Yaw);

    std::istream_iterator<std::string> ii(ss);
    KK.GPS_heading.push_back(atof((*ii++).c_str()));
    int i;
    std::array<float,3> tmp;
    for(i=0;i<3;++i) tmp.at(i) = (atof((*ii++).c_str()));
    KK.SBG_LLA.push_back(tmp);

    for(int i=0;i<3;++i) tmp.at(i) = (atof((*ii++).c_str()));
    KK.SBG_Vxyz.push_back(tmp);
    
    KK.Temperature.push_back(atof((*ii++).c_str()));
    KK.Status.push_back(atof((*ii++).c_str()));

    std::array<float,5> tmp5;
    for(int i=0;i<5;++i) tmp5.at(i) = (atof((*ii++).c_str()));
    KK.GPS_Accu.push_back(tmp5);
    
    KK.NSat.push_back(atof((*ii++).c_str()));

    for(int i=0;i<3;++i) tmp.at(i) = (atof((*ii++).c_str()));
    KK.SBG_Accu.push_back(tmp);

    std::array<float,7> tmp7;
    for(int i=0;i<7;++i) tmp7.at(i) = (atof((*ii++).c_str()));
    KK.GPS_Time.push_back(tmp7);
    
  }  // end of while !EOF
  in.close();

}
// =========== End of GYRO reading ================================



// *******************************************************************************
// SUBROUTINE to show the GNU Public Lincense notice:
//
void ShowGNUPL(){
  std::cout<<"'WindCube Library v.1.0' Copyright (C) 2018 by Pablo Saavedra G. (pablosaa@uib.no)"<<std::endl;
  std::cout<<"Off-shore Boundary Layer Observatory (OBLO), Geophyiscal Institute, University of Bergen."<<std::endl;
  std::cout<<std::endl;
  std::cout<<"This program comes with ABSOLUTELY NO WARRANTY; as GNU software."<<std::endl;
  std::cout<<"Details on using, changing or distributing, see GNUPLv2 `LICENCE' conditions,"<<std::endl;
  std::cout<<"or at <http://www.gnu.org/licenses/> \n"<<std::endl;
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
  std::vector<time_ms> Datum = KK.Uhrzeit;
  //std::vector<std::string> Datum = KK.Datum;
  //std::vector<std::array<std::vector<float>, N_ALTITUDE>> WIND_DATA = KK.WIND_DATA;
  std::vector<std::vector<float> > WIND_DATA = KK.WIND_DATA;
  std::cout<<FileName<<std::endl;
  std::cout<<"Data type and version:"<<KK.IDSystem<<std::endl;
  for(int j=0; j<KK.HeaderItem.size(); ++j)
    std::cout<<j<<")"<<KK.HeaderItem.at(j)<<" >--> "<<KK.HeaderValue.at(j)<<std::endl;
  
  std::cout<<"Size of all: "<<Datum.size()<<" "<<WIND_DATA.size()<<std::endl;
  
  for(int k=0;k<6;++k){
    std::cout<<Datum.at(k).day<<"--"<<Datum.at(k).hour<<":"<<Datum.at(k).min<<":"<<Datum.at(k).sec<<"."<<Datum.at(k).msecs;
    if(std::is_same<V2Lidar,V2LidarRTD>::value)
      std::cout<<" T_rtd: "<<KK.Temperature.at(k)<<std::endl;
    if(std::is_same<V2Lidar,V2LidarSTA>::value)
      std::cout<<" T_sta:"<<KK.Temperature.at(k)<<std::endl;
    for(int i=0;i<8;++i) std::cout<<WIND_DATA[k][i]<<" ";  //WIND_DATA[k][0][i]<<" ";
    std::cout<<std::endl;
  }

  std::cout<<"Height("<<KK.Height.size()<<")= ";
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
  
// *****************************************************************************
// Template Function to convert a sequence of numbers inside a string into a
// vector any type
// template<typename T>
// std::vector<T> String2Vector(std::string VARIN){
//   std::vector<T> VEC;
//   std::istringstream ss(VARIN);
//   std::copy(std::istream_iterator<T>(ss),std::istream_iterator<T>(),std::back_inserter(VEC));
//   return(VEC);
// }
// ============= End of Template Function String2Vector ========================
