# anklesprain

This project extends upon the open source contributions of (DeMers et al 2017):

https://simtk.org/projects/ankle-sprains
https://doi.org/10.1016/j.jbiomech.2016.11.002


Installation:

1) Download and install OpenSim v3.3
   https://simtk.org/frs/index.php?group_id=91
   
2) Drop \setup\ReflexControllersPlugin.dll\ into your local OpenSim installation plugin sub-directory (i.e. C:\OpenSim 3.3\plugins\ )

   If your installation location differs from the above, you must also modify this line in osimExecute.m:
      Model.LoadOpenSimLibrary("C:\OpenSim 3.3\plugins\ReflexControllersPlugin.dll")
   Note that this DLL was compiled from source using Visual Studio 13, via the instructions here: https://github.com/msdemers/opensim-reflex-controllers
   You must ensure that you use the same version of Visual Studio, as was used to compile your installation of OpenSim v3.3
   Which can be confirmed in C:\OpenSim 3.3\sdk\buildinfo.txt


3) Navigate and execute this script to configure MATLAB-OpenSim API: C:\OpenSim 3.3\Scripts\Matlab\configureOpenSim.m

4) Run "osimExecute." 
