# anklesprain

Installation:

1) Download and install OpenSim v3.3
      https://simtk.org/frs/index.php?group_id=91

2) In the OpenSim installation directory, this the MATLAB-OpenSim APIconfiguration script
   C:\OpenSim 3.3\Scripts\Matlab\configureOpenSim.m
   Be sure to run in Windows Administrator mode, otherwise MATLAB may not have permission to edit necessary system path files
   
3) Copy DLL file on this GitHub page, into into your OpenSim plugin sub-directory (i.e. C:\OpenSim 3.3\plugins\)
   \setup\ReflexControllersPlugin.dll\ 

   If your installation location differs from the above, you must also modify this line in osimExecute.m:
      Model.LoadOpenSimLibrary("C:\OpenSim 3.3\plugins\ReflexControllersPlugin.dll")
   Note the DLL on this GitHub page was compiled from source using Visual Studio 13, from the code here: 
      https://github.com/msdemers/opensim-reflex-controllers
   You must ensure that you use the same version of Visual Studio as used to compile your installation of OpenSim v3.3
   Which can be confirmed in C:\OpenSim 3.3\sdk\buildinfo.txt

4) From this GitHub project, run Matlab Script "osimExecute.m" 

Acknowledgments:

This project extends on the OpenSim model and custom muscle controllers developed by Matt DeMers and colleagues

DeMers MS, Hicks JL, Delp SL. Preparatory co-activation of the ankle muscles may prevent ankle inversion injuries. J Biomech. 2017;52(2017):17-23. 

doi:10.1016/j.jbiomech.2016.11.002

https://simtk.org/projects/ankle-sprains
