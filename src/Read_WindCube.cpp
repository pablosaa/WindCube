// ::::::::::: MAIN PROGRAM STARTS ::::::::::::::::::
#include "mex.h"
#include "windcube.h"
#include<functional>

#define MAXSIZEVAR 20

using namespace std;

template<typename V2Lidar>
mxArray *VARLIDAR_MATLAB_OUT(V2Lidar &T){

  mxArray *OutVar;
  mwSize Ndat = T.Datum.size();
  mwSize Nalt = T.Height.size();
  mwSize Nwin = T.WIND_DATA[0][0].size();
  cout<<"Number of Wind variables: "<<Nwin<<endl;
  mwSize dims[3] = {Ndat,Nalt,Nwin};
  double Datum[Ndat][6];

  // Auxiliary Variables:
  const char *FieldsIN[MAXSIZEVAR];
  // converting the Date from string to numerial:
  ConvertWindCube_Date(T.Datum,T.Uhrzeit,Datum);

  V2LidarSTA *H;
  H = (V2LidarSTA *) &T;
  V2LidarRTD *P;
  P = (V2LidarRTD *) &T;

  bool ISRTD = is_same<V2Lidar,V2LidarRTD>::value;

  //   function<vector<float>(V2LidarSTA&)> col1 = &V2LidarSTA::Vbatt;
  
  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *ALTI = mxCreateNumericMatrix(Nalt,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *WIND = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);
  // temporal variables:

  int NFields;

  if(is_same<V2Lidar,V2LidarRTD>::value){
    const char *fields[] = {"TIME","HEIGHT","WIND","ALPHA","BETA","GAMMA","POS","INTEMP"};      
    NFields = sizeof(fields)/sizeof(fields[0]);
    for(int i=0;i<NFields; ++i) FieldsIN[i] = fields[i];    
  }

  if(is_same<V2Lidar,V2LidarSTA>::value){
    const char *fields[] = {"TIME","HEIGHT","WIND","INTEMP","TEMP","PRESS","RH","WIPER"};
    NFields = sizeof(fields)/sizeof(fields[0]);
    for(int i=0;i<NFields; ++i) FieldsIN[i] = fields[i];
  }

  OutVar = mxCreateStructMatrix(1,1,NFields,FieldsIN);
  for(int i=0; i<6; ++i)
    for(int j=0; j<Ndat; ++j)
      *(mxGetPr(DATE)+j+i*Ndat) = Datum[j][i];

  for(int i=0; i<Nalt; ++i) *(mxGetPr(ALTI)+i) = (double) T.Height[i];

  for(int k=0; k<Nwin; ++k)
    for(int i=0; i<Nalt; ++i)
      for(int j=0; j<Ndat; ++j)
	*(mxGetPr(WIND) + j + i*Ndat + k*Ndat*Nalt) = (double) T.WIND_DATA[j][i][k];

  for(int k=0; k<NFields; ++k){
    mxArray *var = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
    for(int i=0; i<Ndat; ++i){
      switch(k){
      case 0:
      case 1:
      case 2:
	break;
      case 3:
	*(mxGetPr(var)+i) = (double) ISRTD?P->Alpha[i]:(double) H->Temperature[i];
	break;
      case 4:
	*(mxGetPr(var)+i) = (double) ISRTD?P->Beta[i]: (double) H->ExtTemp[i];
	break;
      case 5:
	*(mxGetPr(var)+i) = (double) ISRTD?P->Gamma[i]:(double) H->Pressure[i];
	break;
      case 6:
	*(mxGetPr(var)+i) = (double) ISRTD?P->Position[i]:(double) H->RH[i];
	break;
      case 7:
	*(mxGetPr(var)+i) = (double) ISRTD?P->Temperature[i]:(double) H->Nwiper[i];
	break;
      default:
	cout<<"ERROR: assigning MATLAB structure variable!"<<endl;
      }
    }
    mxSetFieldByNumber(OutVar,0,k,var);
  }
  mxSetFieldByNumber(OutVar,0,0,DATE);
  mxSetFieldByNumber(OutVar,0,1,ALTI);
  mxSetFieldByNumber(OutVar,0,2,WIND);

  return(OutVar);
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){

  char *tmpname;
  size_t buflen = mxGetN(prhs[0]) + 1;
  tmpname = (char *) mxMalloc(buflen);  // to extract the file extension
  mxGetString(prhs[0], tmpname, (mwSize) buflen);
  string fname(tmpname);

  unsigned int ExtType = GetExtensionItem(fname);
  cout<<"Opening File "<<fname<<" "<<ExtType<<endl;

  V2LidarSTA STA;
  V2LidarRTD RTD;

  switch(ExtType){
  case 1:
    ReadWindCubeLidar<V2LidarSTA>(fname, STA);
    PrintV2Lidar<V2LidarSTA>(fname, STA);
    plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarSTA>(STA);
    break;
  case 2:
  case 3:
    ReadWindCubeLidar<V2LidarRTD>(fname, RTD);
    PrintV2Lidar<V2LidarRTD>(fname, RTD);
    plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarRTD>(RTD);
    break;
  case 4:
  default:
    cout<<"Sorry... No assignment done :("<<endl;
  }

  return;
}
