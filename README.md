# anklesprain

Installation:

1) Download and install OpenSim v3.3
      https://simtk.org/frs/index.php?group_id=91

2) In your OpenSim installation directory, locate the MATLAB-OpenSim API configuration script
   C:\OpenSim 3.3\Scripts\Matlab\configureOpenSim.m
   Be sure to run in Windows Administrator mode, otherwise MATLAB may not have permission to edit necessary system path files
   
3) Copy DLL into your OpenSim plugin sub-directory (i.e. C:\OpenSim 3.3\plugins\)
   \setup\ReflexControllersPlugin.dll\ 

   If your installation location differs from the above, you must also modify this line in osimExecute.m:
      Model.LoadOpenSimLibrary("C:\OpenSim 3.3\plugins\ReflexControllersPlugin.dll")
   Note the DLL was compiled from source using Visual Studio 13 and the C++ code here: 
      https://github.com/msdemers/opensim-reflex-controllers
   If you want to re-compile on your own, you must ensure that you use the same version of Visual Studio as used to compile your installation of OpenSim v3.3
   Which can be confirmed in C:\OpenSim 3.3\sdk\buildinfo.txt

4) Run Matlab Script "osimExecute.m" to run a single forward dynamic simulation with the pre-defined inputs 
   in the header. This script can be wrapped with automation software (e.g. NESSUS), or built in MATLAB functionality, to perform sensitivity analyses, e.g. Monte Carlo
   
5) See script headers of "parse_mechanics_JCS.m" and "plot_Nessus_CDF_Sensitivity.m" for their usage


Acknowledgments:

This project extends on the OpenSim model and custom muscle controllers developed by Matt DeMers and colleagues

DeMers MS, Hicks JL, Delp SL. Preparatory co-activation of the ankle muscles may prevent ankle inversion injuries. J Biomech. 2017;52(2017):17-23. 

doi:10.1016/j.jbiomech.2016.11.002

https://simtk.org/projects/ankle-sprains
