# WindCube_Libre

A **Windcube** **Lib**rary for **Re**search  

A library to manage and process data from the LeoSphere WindCube suit of Lidars. This library has been developed as a tool for the instrumentation part of the [OBLO](https://oblo.w.uib.no/) (Off-shore Boundary Layer Observatory) project by the Geophysical Institute, University of Begen, Norway.

The repository also contains code for utilities to make the library used by MATLAB/GNU Octave and Python3 (not yet tested) under Linux OS.

## Compatibility
The library is compatible for Linux, tested with OpenSUSE Leap 42.3 and Ubuntu 14. The data files supported are for the Windcuve V2 (all data files) and V1 (10 min. average).

## Compilation
For compilation run the following at the command line:

  ``> make``
  
and the static library will be compiled with the name windcubelin.so, which can be linked by any other code.
When using the stand-alone version, after compiling and before run the program it is needed to include the library directory to the LIBRARY_PATH, in Linux this can be done as: 

  ``> export LD_LIBRARY_PATH=/home/username/WindCube/lib:$LD_LIBRARY_PATH``


For the MATLAB function, the compilation needs to be as follow:

  ``> make matlab``
  
the above command will then create the MEX MATLAB function named ``Read_WindCube.mexa64`` in the /bin directory. See section USAGE to read how to use it within MATLAB.

Fot the GNU Octave version of the MEX function, the compilation is:

  ``> make octave``
  
then the Octave function ``ReadWinCube.mex`` will be created to be used within the Octave workspace.

## Usage

In order to use the library, your code needs to be linked at the compilation command as any other library

  ``> g++ myprogram.cpp -std=gnu++11 -lwindcubelib -L/PATH/WindCube/lib -o myexec.exe ``

this will create an executable binary named ``myexec.exe`` which runs the code within ``myprogram.cpp`` and linked to the library WindCubeLib.

## Examples
In MATLAB or GNU/Octave, a data file from LeoSphere Lidars can be read as follow from the workspace:

  ``> data=Read_WindCube('/tmp/WLS866-14_2019_01_01__00_00_00.sta');``

this will read the data file named ``WLS866-14_2019_01_01__00_00_00.sta`` and load the data into the structure variable ``data``. The field member of the structure ``data`` depends on the type of data file read. For the LeoSphere V2 Lidar there is five possible data files, e.g. _.sta_, _.stdsta_, _.rtd_, _.stdrtd_, and _.gyro_. In the example above a 10 minutes average data file (_.sta_) is loaded and the structure has the following members:

## Folters Structure
The structure of the repository is as follow:

## Contact
(c) 2018. Pablo Saavedra G.

Geophysical Institute

University of Bergen, 
Norway

See LICENCE
