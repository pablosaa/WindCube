// ::::::::::: MAIN PROGRAM STARTS ::::::::::::::::::
#include "mex.h"
#include "windcube.h"

#define MAXSIZEVAR 20

using namespace std;

template<typename V2Lidar>
mxArray *VARLIDAR_MATLAB_OUT(V2Lidar &T){

  mxArray *OutVar;
  mwSize Ndat = T.Datum.size();
  mwSize Nalt = T.Height.size();
  mwSize Nwin = T.WIND_DATA[0][0].size();
  mwSize dims[3] = {Ndat,Nalt,Nwin};
  double Datum[Ndat][6];

  // Auxiliary Variables:
  const char *FieldsIN[MAXSIZEVAR];
  // converting the Date from string to numerial:
  ConvertWindCube_Date(T.Datum,T.Uhrzeit,Datum);

  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *ALTI = mxCreateNumericMatrix(Nalt,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *WIND = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);
  // temporal variables:
  mxArray *var1 = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *var2 = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *var3 = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);

  int NFields;
  bool ISRTD;
  if(is_same<V2Lidar,V2LidarRTD>::value){
    ISRTD=true;
    //#undef ISSTA
    const char *fields[] = {"TIME","HEIGHT","WIND","ALPHA","BETA","GAMMA","POS","INTEMP"};      
    NFields = sizeof(fields)/sizeof(fields[0]);
    for(int i=0;i<NFields; ++i) FieldsIN[i] = fields[i];
    
  }

  if(is_same<V2Lidar,V2LidarSTA>::value){
#undef ISRTD
#define ISSTA
    const char *fields[] = {"TIME","HEIGHT","WIND","INTEMP","TEMP","PRESS","RH","VBATT"};
    NFields = sizeof(fields)/sizeof(fields[0]);
    for(int i=0;i<NFields; ++i) FieldsIN[i] = fields[i];
  }

  OutVar = mxCreateStructMatrix(1,1,NFields,FieldsIN);
  for(int i=0; i<6; ++i)
    for(int j=0; j<Ndat; ++j)
      *(mxGetPr(DATE)+j+i*Ndat) = Datum[j][i];

  for(int i=0; i<Nalt; ++i) *(mxGetPr(ALTI)+i) = (double) T.Height[i];
  //memcpy(mxGetPr(ALTI), &T.Height,Nalt*sizeof(float));

  for(int k=0; k<Nwin; ++k)
    for(int i=0; i<Nalt; ++i)
      for(int j=0; j<Ndat; ++j)
	*(mxGetPr(WIND) + j + i*Ndat + k*Ndat*Nalt) = T.WIND_DATA[j][i][k];
  
  for(int k=0; k<NFields; ++k){
    mxArray *var = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
    for(int i=0; i<Ndat; ++i){
      switch(k){
      case 0:
	break;
      case 1:
	break;
      case 2:
	break;
	//#ifdef ISRTD
      case 3:
	*(mxGetPr(var)+i) = (double) T.Alpha[i];
	break;
      case 4:
	*(mxGetPr(var)+i) = (double) T.Beta[i];
	break;
      case 5:
	*(mxGetPr(var)+i) = (double) T.Gamma[i];
	break;
      case 6:
	*(mxGetPr(var)+i) = (double) T.Position[i];
	break;
      case 7:
	*(mxGetPr(var)+i) = (double) T.Temperature[i];
	break;
	//#endif
#ifdef ISSTA
      case 3:
      	*(mxGetPr(var)+i) = (double) T.Temperature[i];
      	break;
      case 4:
      	*(mxGetPr(var)+i) = (double) T.ExtTemp[i];
      	break;
      case 5:
      	*(mxGetPr(var)+i) = (double) T.Pressure[i];
      	break;
      case 6:
      	*(mxGetPr(var)+i) = (double) T.RH[i];
      	break;
      case 7:
      	*(mxGetPr(var)+i) = (double) T.Vbatt[i];
      	break;
#endif
      }
    }
    mxSetFieldByNumber(OutVar,0,k,var);
  }
  mxSetFieldByNumber(OutVar,0,0,DATE);
  mxSetFieldByNumber(OutVar,0,1,ALTI);
  mxSetFieldByNumber(OutVar,0,2,WIND);
  //mxSetFieldByNumber(OutVar,0,3,var1);
  //mxSetFieldByNumber(OutVar,0,4,var2);
  //mxSetFieldByNumber(OutVar,0,5,var3);

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
  //plhs[0] = mxSetString(fname);


  return;
}
