// ****************************************************************************
// ::::::::::: MAIN MEX PROGRAM TO WORK WITH LEOSPHERE LIDAR ::::::::::::::::::
//
// Part of 'WindCubeMEX' repository.
// (c) 2018, Pablo Saavedra G.
// pablo.saa@uib.no
// University of Bergen, Norway
// SEE LICENCE.TXT
// ***************************************************************************

#include "mex.h"
#include "windcube.h"

#define MAXSIZEVAR 20

using namespace std;

// Definition of Functions and Subroutines:
int GetInputFile_Lidar(char *&, char *&);
mxArray *VARGYRO_MATLAB_OUT(V2Gyro &);
template<typename V2Lidar>
mxArray *VARLIDAR_MATLAB_OUT(V2Lidar &);
// End of Function definition.



// *************************************************************************
//    MAIN MEX FUNCTION TO INTERFACE LIRARY WITH GNU OCTAVE/MATLAB
//
// INPUT: string with the full path to the Lidar data file to read.
// OUTPUT: Structure with Lidar variables as structure fields according to
//         the input file.
// TODO:
// * allow multiple outputs e.g. .STA and .GYRO data structures
//
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]){


  // ---------------------------
  // Checking output parameters:
  if(nlhs<1 || nlhs>2) mexErrMsgTxt("Need at least one output variable, try again ;)");

  // Checking the input parameters:
  char *filen, *OUTDIR;
  if (nrhs>=0 && nrhs<4){
    switch(nrhs){
    case 1:
      // first input: file name:
      if(mxIsChar(prhs[0])){
	size_t FileLength = mxGetN(prhs[0])+1;
	filen = (char *) mxCalloc(FileLength, sizeof(char));
	mxGetString(prhs[0], filen, FileLength);
	mexPrintf("LIDAR file to open: %s\n",filen);
      }
      else mexErrMsgTxt("First input needs to be a string FILENAME.");
      break;
    case 0:
      if(GetInputFile_Lidar(filen, OUTDIR)!=0)
	mexErrMsgTxt("Wrong input LIDAR file!");
      break;
    default:
      mexErrMsgTxt("Ups! something is wrong with the input variables!");
    }
  }

  string fname(filen);

  unsigned int ExtType = GetExtensionItem(fname);
  cout<<"Opening File "<<fname<<" "<<ExtType<<endl;

  V2LidarSTA STA;
  V2LidarRTD RTD;
  V2Gyro GYRO;
  
  switch(ExtType){
  case 1:
    ReadWindCubeLidar<V2LidarSTA>(fname, STA);
    PrintV2Lidar<V2LidarSTA>(fname, STA);
    plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarSTA>(STA);
    break;
  case 2:
    break;
  case 3:
    ReadWindCubeLidar<V2LidarRTD>(fname, RTD);
    PrintV2Lidar<V2LidarRTD>(fname, RTD);
    plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarRTD>(RTD);
    break;
  case 4:
    break;
  case 5:
    ReadWindCubeGyro(fname,GYRO);
    plhs[0] = VARGYRO_MATLAB_OUT(GYRO);
    break;
  default:
    cout<<"Sorry... No assignment done :("<<endl;
  }

  return;
}
// =========== End of MAIN MATLAB/OCTAVE MEX FUNCTION =======================

// --------------------------------------------------------------------------
// HERE STARTS THE AUXILIARY FUNCTIONS AND SUBROUTINES:


// *************************************************************************
// Auxiliary Function to create mxArray with the Header info from the Lidar
// files .sta, .rtd, .stastd, .rtdstd
//
mxArray *CreateHeaderCell(vector<string> &Item, vector<string> &Value){
  mwSize m = Item.size();
  mwSize n = 2;
  mxArray *OutCell = mxCreateCellMatrix(m, n);
  for(int i=0; i<n*m; ++i)
    mxSetCell(OutCell,i,mxCreateString(i<m?Item.at(i).c_str():Value.at(i-m).c_str()));
  
  return(OutCell);
}

// ***************************************************************************
// Template Function to assign V2-Lidar data to MEX MATLAB structure
// Used either for .sta, .rtd, .stastd or .rtdstd data files.
//
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

  V2LidarSTA *H;
  H = (V2LidarSTA *) &T;
  V2LidarRTD *P;
  P = (V2LidarRTD *) &T;

  bool ISRTD = is_same<V2Lidar,V2LidarRTD>::value;

  //   function<vector<float>(V2LidarSTA&)> col1 = &V2LidarSTA::Vbatt;

  // Converting Date and Hour from string to numeric array:
  ConvertWindCube_Date(T.Datum,T.Uhrzeit,Datum);  
  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *ALTI = mxCreateNumericMatrix(Nalt,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *WIND = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);
  mxArray *HEADER; //= mxCreateCellMatrix(mwSize m, mwSize n);
  // temporal variables:

  int NFields;

  if(is_same<V2Lidar,V2LidarRTD>::value){
    const char *fields[] = {"TIME","HEIGHT","WIND","ALPHA","BETA","GAMMA","POS","INTEMP","HEADER"};      
    NFields = sizeof(fields)/sizeof(fields[0]);
    for(int i=0;i<NFields; ++i) FieldsIN[i] = fields[i];    
  }

  if(is_same<V2Lidar,V2LidarSTA>::value){
    const char *fields[] = {"TIME","HEIGHT","WIND","INTEMP","TEMP","PRESS","RH","WIPER","HEADER"};
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
      case 8:
	HEADER = CreateHeaderCell(T.HeaderItem,T.HeaderValue);
	break;
      default:
	cout<<"ERROR: assigning MATLAB structure variable!"<<endl;
      }
    }
    mxSetFieldByNumber(OutVar,0,k,k<8?var:HEADER);
  }
  mxSetFieldByNumber(OutVar,0,0,DATE);
  mxSetFieldByNumber(OutVar,0,1,ALTI);
  mxSetFieldByNumber(OutVar,0,2,WIND);

  return(OutVar);
}
// =========== End of Template Function for V2-Lidar assignment ============


// **************************************************************************
// Function to assign Gyro data struct to MEX variables as MATLAB structure
// Part of the WindCubeMEX Project:
// TODO:
// * Include all variables
mxArray *VARGYRO_MATLAB_OUT(V2Gyro &G){

  mxArray *OutVar;
  mwSize Ndat = G.Datum.size();
  double Datum[Ndat][6];

  const char *FieldsIN[] = {"TIME","PITCH","ROLL","YAW","COOR","VEL","SBGACCU","GPSACCU","GPSTIME","NSAT"};
  mwSize NFields = sizeof(FieldsIN)/sizeof(FieldsIN[0]);
  
  OutVar = mxCreateStructMatrix(1,1,NFields,FieldsIN);
  
  ConvertWindCube_Date(G.Datum,G.Uhrzeit,Datum);  
  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *PITCH = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *ROLL  = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *YAW   = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *COOR  = mxCreateNumericMatrix(Ndat,3,mxDOUBLE_CLASS, mxREAL);
  mxArray *VEL   = mxCreateNumericMatrix(Ndat,3,mxDOUBLE_CLASS, mxREAL);
  mxArray *SBGAC = mxCreateNumericMatrix(Ndat,3,mxDOUBLE_CLASS, mxREAL);
  mxArray *GPSAC = mxCreateNumericMatrix(Ndat,5,mxDOUBLE_CLASS, mxREAL);
  mxArray *GPSTI = mxCreateNumericMatrix(Ndat,7,mxDOUBLE_CLASS, mxREAL);
  mxArray *NSAT  = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  
  for(int i=0; i<Ndat; ++i){
    for(int j=0; j<6; ++j)
      *(mxGetPr(DATE)+i+j*Ndat) = Datum[i][j];

    *(mxGetPr(PITCH)+i) = (double) G.Pitch[i];
    *(mxGetPr(ROLL) +i) = (double) G.Roll[i];
    *(mxGetPr(YAW)  +i) = (double) G.Yaw[i];
    *(mxGetPr(NSAT) +i) = (double) G.NSat[i];
    
    for(int j=0; j<7; ++j){
      if(j<3){
      *(mxGetPr(COOR)+i+j*Ndat) = (double) G.SBG_LLA[i][j];
      *(mxGetPr(VEL)+i+j*Ndat) = (double) G.SBG_Vxyz[i][j];
      *(mxGetPr(SBGAC)+i+j*Ndat) = (double) G.SBG_Accu[i][j];
      }
      if(j<5) *(mxGetPr(GPSAC)+i+j*Ndat) = (double) G.GPS_Accu[i][j];

      *(mxGetPr(GPSTI)+i+j*Ndat) = (double) G.GPS_Time[i][j];
    }
  }  // end over Ndat
  mxSetFieldByNumber(OutVar,0,0,DATE);
  mxSetFieldByNumber(OutVar,0,1,PITCH);
  mxSetFieldByNumber(OutVar,0,2,ROLL);
  mxSetFieldByNumber(OutVar,0,3,YAW);
  mxSetFieldByNumber(OutVar,0,4,COOR);
  mxSetFieldByNumber(OutVar,0,5,VEL);
  mxSetFieldByNumber(OutVar,0,6,SBGAC);
  mxSetFieldByNumber(OutVar,0,7,GPSAC);
  mxSetFieldByNumber(OutVar,0,8,GPSTI);
  mxSetFieldByNumber(OutVar,0,9,NSAT);
  return(OutVar);
}
// ========= End of Function to assign Gyro data struct ======================


// ****************************************************************
//  ROUTINE TO OPEN AN INPUT-FILE DIALOG BOX
// ARGUMENTS:
// * filen:  input DBL file name (output arg).
// * OUTDIR: directory where filen is located (output arg).
// * BOXLIM: 4-element vector (lat_min,lon_min,lat_max,lon_max) (output atg).
// RETURN: status=0 -> OK, status!=0 -> wrong procedure.
int GetInputFile_Lidar(char *& filen, char *& OUTDIR){

  int strLength, status;
  mxArray *INVAR, *OUTVAR[2];

  ShowGNUPL();      // displaying License, it is free!.
  INVAR = mxCreateString("*.rtd; *.sta; *.rtdstd; *.stastd; *.gyro");
  status = mexCallMATLAB(2,OUTVAR,1,&INVAR,"uigetfile");
  if (status!=0) mexErrMsgTxt("File selection not possible!");

  // passing the input and output path
  strLength = mxGetN(OUTVAR[0])+mxGetN(OUTVAR[1])+1;
  if (strLength<4)  mexErrMsgTxt("File selection empty or canceled!");
  filen = (char *) mxCalloc(strLength, sizeof(char));
  mxGetString(OUTVAR[1],filen,strLength);

  // passing the file name
  strLength = mxGetN(OUTVAR[0])+1;
  mxGetString(OUTVAR[0],filen+mxGetN(OUTVAR[1]),strLength);
  strLength = mxGetN(OUTVAR[1])+1;
  OUTDIR = (char *) mxCalloc(strLength,sizeof(char));
  mxGetString(OUTVAR[1], OUTDIR, strLength);
  mexPrintf("LIDAR file chosen: %s\n",filen);
  return(status);
}
// ================ End of Lidar File Dialog-Box ========================
