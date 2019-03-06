// ****************************************************************************
// ::::::::::: MAIN MEX PROGRAM TO WORK WITH LEOSPHERE LIDAR ::::::::::::::::::
//
// Part of 'WindCubeLib' repository.
// (c) 2018, Pablo Saavedra G.
// pablo.saa@uib.no
// University of Bergen, Norway
// SEE LICENCE.TXT
// ***************************************************************************


// VARIABLES DESCRIPTION:
// RTD:
// Fields:
// * WIND 8-columns: CNR [dB]; Radial Wind [m/s]; Radial Wind Dispersion [m/s]; Horizontal Wind [m/s]; Wind Dir. [°]; X-, Y-, Z-Wind component [m/s]

#include "mex.h"
#include "windcube.h"

using namespace std;

// Definition of Functions and Subroutines:
////int GetInputFile_Lidar(char *&, char *&);
int GetInputFile_Lidar(vector<string> &);
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
  char *filen=NULL; //, *OUTDIR;
  vector<string> InFiles;
  bool MULTIFILES=false;
  unsigned int NTotalFiles=0;
  if (nrhs>=0 && nrhs<4){
    switch(nrhs){
    case 1:
      // first input: file name:
      if(mxIsChar(prhs[0])){
	NTotalFiles = 1;
	size_t FileLength = mxGetN(prhs[0])+1;
	filen = (char *) mxCalloc(FileLength, sizeof(char));
	mxGetString(prhs[0], filen, FileLength);
	InFiles.push_back(filen);
	mexPrintf("LIDAR file to open: %s\n",filen);
      }
      else if(mxIsCell(prhs[0])){
	NTotalFiles = mxGetN(prhs[0]);
	for(unsigned int i=0; i<NTotalFiles; ++i){
	  mxArray *aFile = mxGetCell(prhs[0],i);
	  if(!mxIsChar(aFile)) mexErrMsgTxt("Sorry input file is not a string!");
	  size_t FileLength = mxGetN(aFile)+1;
	  mxGetString(aFile,filen,FileLength);
	  InFiles.push_back(filen);
	}
      }
      else mexErrMsgTxt("First input needs to be a string FILENAME.");
      break;
    case 0:
      if(GetInputFile_Lidar(InFiles)!=0)
	mexErrMsgTxt("Wrong input LIDAR file!");
      NTotalFiles = InFiles.size();
      if(NTotalFiles==1)
	filen = &(InFiles.at(0)[0]);
      else
	MULTIFILES = true;

      break;
    default:
      mexErrMsgTxt("Ups! something is wrong with the input variables!");
    }
  }

  cout<<"Number of Selected Files is: "<<NTotalFiles<<endl;
  if(NTotalFiles>1) plhs[0] = mxCreateCellMatrix((mwSize) NTotalFiles, 1);
  for(unsigned int i=0; i<NTotalFiles; ++i){
    //string fname(filen);
    string fname = InFiles.at(i); //NTotalFiles==1?filen:InFiles.at(i);
    // quick check if input file exist?
    fstream f(InFiles.at(i).c_str());
    if(!f) mexErrMsgTxt("File does not exist!");
    else f.close();
    
    unsigned int ExtType = GetExtensionItem(fname);
    
    cout<<"Opening File "<<fname<<" "<<ExtType<<endl;

    V2LidarSTA STA;
    V2LidarRTD RTD;
    V2Gyro GYRO;
  
    switch(ExtType){
    case 1:
    case 2:
      ReadWindCubeLidar<V2LidarSTA>(fname, STA);
      //PrintV2Lidar<V2LidarSTA>(fname, STA);
      if(MULTIFILES)
	mxSetCell(plhs[0],i,VARLIDAR_MATLAB_OUT<V2LidarSTA>(STA));
      else
	plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarSTA>(STA);
      break;
    case 3:
    case 4:
      ReadWindCubeLidar<V2LidarRTD>(fname, RTD);
      //PrintV2Lidar<V2LidarRTD>(fname, RTD);
      if(MULTIFILES)
	mxSetCell(plhs[0],i,VARLIDAR_MATLAB_OUT<V2LidarRTD>(RTD));
      else
	plhs[0] = VARLIDAR_MATLAB_OUT<V2LidarRTD>(RTD);
      break;
    case 5:
      ReadWindCubeGyro(fname,GYRO);
      if(MULTIFILES)
	mxSetCell(plhs[0],i,VARGYRO_MATLAB_OUT(GYRO));
      else
	plhs[0] = VARGYRO_MATLAB_OUT(GYRO);
      break;
    default:
      cout<<"Sorry... No assignment done :("<<endl;
    }  // end for command switch(ExtType)

  } // end for Number of Input Files
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
  mwSize dims[3] = {Ndat,Nalt,3};
  double Datum[Ndat][6];

  // List of possible structure fields for all type of data files:
  const char *TotFields[] = {"HEADER",    // 0
			     "TIME",      // 1
			     "HEIGHT",    // 2
			     "ALPHA",    // 3
			     "BETA",    // 4
			     "GAMMA",     // 5
			     "POS",    // 6
			     "IN_TEMP",    // 7
			     "TEMPERATURE",    // 8
			     "PRESSION",    // 9
			     "RH",    // 10
			     "WIPER",    // 11
			     "WIND_DIRECTION",    // 12
			     "HORIZONTAL_WIND_SPEED",    // 13
			     "HORIZONTAL_WIND_STD",     // 14			     
			     "VERTICAL_WIND_SPEED",    // 15
			     "VERTICAL_WIND_STD",    // 16
			     "XYZ_WIND_VECTOR",      // 17
			     "XYZ_WIND_STD",         // 18
			     "RADIAL_WIND_SPEED",    // 19
			     "RADIAL_WIND_STAT",     // 20
			     "CNR",    // 21
			     "CNR_STD",        // 22
			     "DOPPLER_SPECTRA_BROAD",    // 23
			     "STD_DOPPLER_SPECTRA_BROAD",   // 24
			     "DATA_AVAL"    // 25
  };

  // vector to hold the indexes of TotFields corresponding for every data type: 
  vector<int> idxField;
  
  // Auxiliary Template variables to match struct variables:
  V2LidarSTA *H;
  H = (V2LidarSTA *) &T;
  V2LidarRTD *P;
  P = (V2LidarRTD *) &T;

  // checkeing whether Lidar variable is STA or RTD?
  bool ISRTD = is_same<V2Lidar,V2LidarRTD>::value;
  bool V2;

  V2 = !T.IDSystem.compare(0,6,"WLS866");   // true for Lidar Version 2
  V2 = T.IDSystem.compare(0,4,"WLS7");      // false for Lidar Version 1

  //   function<vector<float>(V2LidarSTA&)> col1 = &V2LidarSTA::Vbatt;

  // Converting Date and Hour from string to numeric array:
  ConvertWindCube_Date(T.Datum,T.Uhrzeit,Datum);

  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *ALTI = mxCreateNumericMatrix(Nalt,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *HEADER; //= mxCreateCellMatrix(mwSize m, mwSize n);

  // temporal index for matlabt structure string names:
  if(is_same<V2Lidar,V2LidarRTD>::value && V2)
    idxField = {0,1,2,3,4,5,6,7,12,13,17,19,20,21};

  if(is_same<V2Lidar,V2LidarSTA>::value && V2)
    idxField = {0,1,2,7,8,9,10,11,12,13,14,15,16,23,21,25};

  if(is_same<V2Lidar,V2LidarSTA>::value && !V2)
    idxField = {0,1,2,7,11,12,13,14,17,18,21,22,23,25};

  if(is_same<V2Lidar,V2LidarRTD>::value && !V2)
    idxField = {0,1,2,6,7,12,13,17,19,20,21};
  

  // Total number of possible fields:  
  mwSize NFields = idxField.size(); //sizeof(fields)/sizeof(fields[0]);
  cout<<"Number of fields: "<<NFields<<endl;
  const char *FieldsIN[NFields];
  for(int i=0;i<NFields; ++i) FieldsIN[i] = TotFields[idxField.at(i)];

  // Creating Matlab output variable:
  OutVar = mxCreateStructMatrix(1,1,NFields,FieldsIN);

  // Sorting out the WIND variables depending on data type: RTD or STA
  //for(int j=0; j<Ndat; ++j)
  //  for(int i=0; i<Nalt; ++i)
  //    for(int k=0; k<Nwin; ++k)
  //	*(mxGetPr(WIND2V) + j + i*Ndat + k*Ndat*Nalt) = (double) T.WIND_DATA[j][i][k];

  // Fields from 0:HEADER, 1:DATE and 2:HEIGHT are filled here:
  // HEADER cell for all type of data file:
  HEADER = CreateHeaderCell(T.HeaderItem,T.HeaderValue);
  mxSetFieldByNumber(OutVar,0,0,HEADER);
  
  // TIME:
  for(int i=0; i<6; ++i)
    for(int j=0; j<Ndat; ++j)
      *(mxGetPr(DATE)+j+i*Ndat) = Datum[j][i];

  mxSetFieldByNumber(OutVar,0,1,DATE);

  // ALTITUDE:
  for(int i=0; i<Nalt; ++i) *(mxGetPr(ALTI)+i) = (double) T.Height[i];
  mxSetFieldByNumber(OutVar,0,2,ALTI);

  // from Field number 3 are filled in the following loop:
  for(int k=3; k<NFields; ++k){
    //mexPrintf("assigning WIND array for %s",FieldsIN[k]);
    mxArray *var = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);

    for(int i=0; i<Ndat; ++i){
      switch(idxField.at(k)){
      case 3:
	*(mxGetPr(var)+i) = (double) P->Alpha[i];
	break;
      case 4:
	*(mxGetPr(var)+i) = (double) P->Beta[i];
	break;
      case 5:
	*(mxGetPr(var)+i) = (double) P->Gamma[i];
	break;
      case 6:
	*(mxGetPr(var)+i) = (double) P->Position[i];
	break;
      case 7:
	*(mxGetPr(var)+i) = (double) T.Temperature[i];
	break;
      case 8:
	*(mxGetPr(var)+i) = (double) H->ExtTemp[i];
	break;
      case 9:
	*(mxGetPr(var)+i) = (double) H->Pressure[i];
	break;
      case 10:
	*(mxGetPr(var)+i) = (double) H->RH[i];
	break;
      case 11:
	*(mxGetPr(var)+i) = (double) H->Nwiper[i];
	break;
      case 12:
	// Wind direction:
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var) + i + h*Ndat) = (double) T.WIND_DATA[i][h][4];
	break;
      case 13:
	// Horizontal wind speed:
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][ISRTD?3:0];
	break;
      case 14:
	// STATS-Horizontal wind speed (spread, min, max):
	if(i==0)  var = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);
	
	for(int h=0; h<Nalt; ++h)
	  for(int l=0; l<3; ++l)
	    *(mxGetPr(var)+i + h*Ndat + l*Ndat*Nalt) = (double) T.WIND_DATA[i][h][ISRTD?(l+5):(l+1)];
	break;
      case 15:
	// STA: Vertical wind velocity.
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][5];
	break;
      case 16:
	// STA: Vertical wind velocity statistics (Dispersion).
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][6];
	break;
      case 17:
	// STA_V1 & RTD_V2: XYZ wind speed
	if(i==0) var = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);

	for(int h=0; h<Nalt; ++h)
	  for(int l=0; l<3; ++l)
	    *(mxGetPr(var)+i + h*Ndat + l*Ndat*Nalt) = (double) T.WIND_DATA[i][h][ISRTD?(l+5):(2*l+5)];
	break;
      case 18:
	// STA_V1: XYZ statistics wind speed
	if(i==0) var = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);

	for(int h=0; h<Nalt; ++h)
	  for(int l=0; l<3; ++l)
	    *(mxGetPr(var)+i + h*Ndat + l*Ndat*Nalt) = (double) T.WIND_DATA[i][h][(2*l+6)];
	break;
      case 19:
	// RTD: Wind radial velocity
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][1];
	break;
      case 20:
	// RTD: Wind Statistics radial velocity
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][2];
	break;
      case 21:
	// STA (V1. V2) & RTD (V2): CNR
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	{int icol = V2?7:11;
	
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][ISRTD?0:icol];
	}
	break;
      case 22:
	// STA:  Statistics CNR (CNR_STD, min, max).
	if(i==0 && !V2)  var = mxCreateNumericArray(3,dims,mxDOUBLE_CLASS, mxREAL);
	if(i==0 && V2)  var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	{int icol = V2?0:3;
	for(int h=0; h<Nalt; ++h)
	  for(int l=0; l<icol; ++l)
	    *(mxGetPr(var)+i + h*Ndat + l*Ndat*Nalt) = (double) T.WIND_DATA[i][h][V2?8:(l+12)];
	}
	break;
      case 23:
	// STA (V1,V2): Doppler spectra broad.
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][V2?9:15];
	break;
      case 24:
	// STA (V1): Statistics Doppler spectra broad.
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][16];
	break;
      case 25:
	// STA (V1,V2): Data availability.
	if(i==0) var = mxCreateNumericMatrix(Ndat,Nalt,mxDOUBLE_CLASS, mxREAL);
	for(int h=0; h<Nalt; ++h)
	  *(mxGetPr(var)+i + h*Ndat) = (double) T.WIND_DATA[i][h][V2?10:17];
	break;
      default:
	cout<<"ERROR: assigning MATLAB structure variable!"<<endl;	
      }  // end of switch
      
    }   // end loop over Ndat
    mxSetFieldByNumber(OutVar,0,k,var);
  }   // end loop over NFields

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

  const char *FieldsIN[] = {"TIME","PITCH","ROLL","YAW","GPSHEADING","COOR","VEL","SBGACCU","GPSACCU","GPSTIME","NSAT"};
  mwSize NFields = sizeof(FieldsIN)/sizeof(FieldsIN[0]);
  
  OutVar = mxCreateStructMatrix(1,1,NFields,FieldsIN);
  
  ConvertWindCube_Date(G.Datum,G.Uhrzeit,Datum);  
  // Common variables:
  mxArray *DATE = mxCreateNumericMatrix(Ndat,6,mxDOUBLE_CLASS, mxREAL);
  mxArray *PITCH = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *ROLL  = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *YAW   = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
  mxArray *GPSHEAD = mxCreateNumericMatrix(Ndat,1,mxDOUBLE_CLASS, mxREAL);
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
    *(mxGetPr(GPSHEAD) + i ) = (double) G.GPS_heading[i];
    
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
  mxSetFieldByNumber(OutVar,0,4,GPSHEAD);
  mxSetFieldByNumber(OutVar,0,5,COOR);
  mxSetFieldByNumber(OutVar,0,6,VEL);
  mxSetFieldByNumber(OutVar,0,7,SBGAC);
  mxSetFieldByNumber(OutVar,0,8,GPSAC);
  mxSetFieldByNumber(OutVar,0,9,GPSTI);
  mxSetFieldByNumber(OutVar,0,10,NSAT);
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
//int GetInputFile_Lidar(char *& filen, char *& OUTDIR){
int GetInputFile_Lidar(vector<string> &InFiles){

  char *filen, *OUTDIR;
  int strLength, DirLength, status;
  mxArray *INVAR[4], *OUTVAR[2];

  ShowGNUPL();      // displaying License, it is free!.
  mxArray *FilterCell = mxCreateString("*.rtd; *.sta; *.stdrtd; *.stdsta; *.gyro");
  INVAR[0] = mxCreateCellMatrix(1, 2);
  mxSetCell(INVAR[0],0,FilterCell);
  mxSetCell(INVAR[0],1,mxCreateString("V2 Supported Files"));
  //INVAR[0] = mxCreateString("*.rtd; *.sta; *.rtdstd; *.stastd; *.gyro");
  INVAR[1] = mxCreateString("Select a LIDAR input file...");
  INVAR[2] = mxCreateString("MultiSelect");
  INVAR[3] = mxCreateString("on");
  status = mexCallMATLAB(2,OUTVAR,4,INVAR,"uigetfile");
  if (status!=0) mexErrMsgTxt("File selection not possible!");

  // Getting the Directory where files are located:
  DirLength = mxGetN(OUTVAR[1]);
  if (DirLength<4)  mexErrMsgTxt("File selection empty or canceled!");
  OUTDIR = (char *) mxCalloc(DirLength+1,sizeof(char));
  mxGetString(OUTVAR[1], OUTDIR, DirLength);

  // Check if cell (mulltiple input files selected) or if string (single file selected)
  if (mxIsCell(OUTVAR[0])){
    mwSize Ninfile = mxGetN(OUTVAR[0]);
    for(int i=0; i<Ninfile; ++i){
      mxArray *InFile = mxGetCell(OUTVAR[0],i);
      strLength = mxGetN(InFile) +1 ;
      filen = (char *) mxCalloc(DirLength + strLength, sizeof(char));
      mxGetString(OUTVAR[1], filen, DirLength+1);
      mxGetString(InFile,filen + DirLength, strLength);
      cout<<filen<<endl;
      InFiles.push_back(filen);
    }
  }
  // passing the input and output path
  else{
    //strLength = mxGetN(OUTVAR[0])+mxGetN(OUTVAR[1])+1;
    //if (strLength<4)  mexErrMsgTxt("File selection empty or canceled!");
    strLength = DirLength + mxGetN(OUTVAR[0]) +1 ;
    filen = (char *) mxCalloc(strLength, sizeof(char));
    mxGetString(OUTVAR[1],filen,strLength);
    // passing the file name
    strLength = mxGetN(OUTVAR[0])+1;
    mxGetString(OUTVAR[0],filen+mxGetN(OUTVAR[1]),strLength);
    InFiles.push_back(filen);
  }
  
  mexPrintf("LIDAR file chosen: %s\n",filen);
  return(status);
}
// ================ End of Lidar File Dialog-Box ========================
