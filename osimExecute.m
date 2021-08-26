% clear all
% close all

PLOT_RESULTS = 1; % turn off to just run the model and write outputs

%%% Baseline Defaults: these reproduce level landing simulation of (DeMers et al 2017)
% % % pdi =     0.0    ; %degrees, platform incline
% % % hi  =    0.300   ; %m, drop height
% % % ei  =    50000000; %Pa/m, contact stiffness, (rubber/sole thickness) = (1MPa / 0.02m), 50000000 in DeMers
% % % di  =    5.0     ; %sec/m, contact dissipation
% % % fi  =    0.000   ; %stiffness, passive anatomy, scale factor 0=100%, +0.5=150% baseline, f > 1 = more stiff
% % % ci  =    0.000   ; %co-activation 1 = 100%
% % % rgi =    0.000   ; %reflex gain
% % % j1i =  -34.000   ; %joint, ankle, dorsi/plantar (-34 in DeMers)
% % % j2i =   -5.720   ; %joint, ankle, inv/ev (0 in DeMers)
% % % m1i =    0.000   ; %max_isometric_force (0 = 100% strength)
% % % bi  =    NaN     ; %stiffness, "brace", scale factor 0=100%, +0.5=150% baseline, f > 1 = more stiff; set NaN for no brace 

%%% Determinstic Input Parameters
pdi =     30.0   ; 
hi  =    0.300   ; 
ei  =    50000000; 
di  =    5.0     ; 
fi  =    0.000   ; 
ci  =    0.000   ; 
rgi =    5.000   ; 
j1i =  -34.000   ; 
j2i =   -5.720   ;
m1i =    0.000   ;  
bi  =    0.500   ;  

DISABLE_REFLEXES = false;
DISABLE_COACT = false;

trial = 'osim';
modelfile = 'DeMers_nominal_coact_reflex_brace_v2.osim';

%%%%%%%=========================================================================================
%%%%%%%============MODIFY BELOW=================================================================
%%%%%%% Copy/paste after, make sure "exit" at end
%%%%%%%=========================================================================================
%%%%%%%=========================================================================================

global dirN
dirN=cd; %current NESSUS sub-directory under \F\####
dirOutput = [dirN '\osim_output'];
mkdir(dirOutput)

%%%% (Option 1) Use if running in current directory
dirSetup = [dirN '\setup'];
dirCommon = [dirN '\common'];

%%%% (Option 2) Use if running in NESSUS sub-directories
% dirNsplit = strsplit(fileparts(dirN),'\');
% dirNroot = strjoin(dirNsplit(1:end-1),'\');
% dirSetup = [dirNroot '\setup'];
% dirCommon = [dirNroot '\common'];

%% Simulation settings (study fixed)
time_initial = 0; %sec 
time_end = 0.150; %sec

%% Initialize API
addpath(genpath('.\common\'))
addpath(genpath('.\setup\'))

import org.opensim.modeling.*
import org.opensim.utils.*

Model.LoadOpenSimLibrary("C:\OpenSim 3.3\plugins\ReflexControllersPlugin.dll")

%% Initialize directories and load default settings
%%%Model files, setup, states, etc...

setupGeneric=[dirSetup '\Setup_Forward.xml'];
% statesFileDefault=[dirSetup '\initialState_nominalLevelGround.sto'];
statesFileDefault=[dirSetup '\initialState_nominalLevelGround_WithMusc.sto']; %4/13/21: ADDED fiber activations and lengths
stateSto = Storage(statesFileDefault); %load as osim API object

%save input parameters to file for easy reference
T = array2table([pdi,round(hi,3),ei,di,fi,ci,rgi,j1i,j2i,m1i],'VariableNames',{'pdi','hi','ei','di','fi','ci','rgi','j1i','j2i','m1i'});
writetable(T,[dirOutput '\' trial '_params.txt'])

%% Modify initial states (posture, velocity)
vi  = -1 * sqrt( 2 * 9.80665 *  hi ); %m/s, contact velocity, sqrt(2*g*h)

%Convert to radians
j1i = j1i/180*pi;
j2i = j2i/180*pi;

%%% Changed 9/30:  removed ankle_angle_r_u=0; left at experimental 784deg/s dorsi
%%% Note 4/1/2021: DeMers subtalar_angle_r_u is non-zero even though initial state = 0
newStates={'pelvis_ty_u',vi; 'ankle_angle_r',j1i; 'subtalar_angle_r',j2i; 'subtalar_angle_r_u',0 }; 

for si=1:size(newStates,1)
istate=stateSto.getStateIndex(newStates{si,1}); 
A = ArrayDouble(newStates{si,2}, stateSto.getSize); % constant value over all file timesteps
stateSto.setDataColumn(istate,A);
end
statesFileMod=[dirOutput '\' trial '_initialState.sto'];
stateSto.print(statesFileMod);

%% Configure model and enable visualizer to show simulation
model = Model([dirSetup '\' modelfile]);
% model.setUseVisualizer(true);
model.initSystem();

%% Adjust platform incline (can't change through initial states, because model coordinate must be locked)
pli = pdi /180*pi; %radians, platform incline
model.getCoordinateSet.get('platform_rx').set_default_value(pli); %radians

%% Adjust contact model
eff=ElasticFoundationForce.safeDownCast( model.getForceSet.get('foot_floor_r') );
eff.setStiffness(ei);
eff.setDissipation(di);

%% Adjust ankle stiffness
%%%fi = [-1,1] stiffness scale factor f>1 = more stiff
c = cubicBushingExp(fi); %(3x1) (x,y,z)
bushing1 = model.getForceSet.get('cubic_ankle_bushing_r'); %passive anatomy
MX=c{1}; MY=c{2}; MZ=c{3}; 
pass = ExpressionBasedBushingForce.safeDownCast(bushing1);
pass.setMxExpression(MX);
pass.setMyExpression(MY);
pass.setMzExpression(MZ);

%% Adjust additional brace stiffness
c2 = cubicBushingExp(bi); %(3x1) (x,y,z)
bushing2 = model.getForceSet.get('brace_exp_bushing_r');
MX2 = c2{1}; MY2 = c2{2}; MZ2 = c2{3}; 
brace = ExpressionBasedBushingForce.safeDownCast(bushing2);
brace.setMxExpression(MX2);
brace.setMyExpression(MY2);
brace.setMzExpression(MZ2);

%% Adjust Reflexes
%%%% Note if these are activate at the same time as co-activation
%%%% controllers, they may conflict

%%%%  Delay: literature suggests 60-120ms, DeMers used 60 for fastest possible)
%%%%  Gain: Demers explored (0-10) (none-strong)
%%%%  Deprecated PropertyHelper doc:
%%%%  https://simtk.org/api_docs/opensim/api_docs30/classOpenSim_1_1PropertyHelper.html

reEv=Controller.safeDownCast( model.updControllerSet.get('AnkleEverterDelayedReflexes') );
reIv=Controller.safeDownCast( model.updControllerSet.get('AnkleInverterDelayedReflexes') );

PropertyHelper.setValueBool(DISABLE_REFLEXES, reEv.updPropertyByName('isDisabled'), 0)
PropertyHelper.setValueDouble(rgi, reEv.updPropertyByName('gain'), 0)
PropertyHelper.setValueDouble(0.06, reEv.updPropertyByName('delay'), 0)

PropertyHelper.setValueBool(DISABLE_REFLEXES, reIv.updPropertyByName('isDisabled'), 0)
PropertyHelper.setValueDouble(rgi, reIv.updPropertyByName('gain'), 0)
PropertyHelper.setValueDouble(0.06, reIv.updPropertyByName('delay'), 0)

%% Adjust co-activation
coEv=PrescribedController.safeDownCast( model.updControllerSet.get('everter_controls_r') );
coInv=PrescribedController.safeDownCast( model.updControllerSet.get('inverter_controls_r') );
 
coEv.set_isDisabled(DISABLE_COACT);
coInv.set_isDisabled(DISABLE_COACT);

% %%% EVERTORS: (ext_dig_r, per_brev_r, per_long_r, per_tert_r)
fset1 = FunctionSet();
fset1.adoptAndAppend(Constant(ci));
fset1.adoptAndAppend(Constant(ci));
fset1.adoptAndAppend(Constant(ci));
fset1.adoptAndAppend(Constant(ci));
coEv.set_ControlFunctions(fset1)

%%% INVERTORS: (ext_hal_r, flex_dig_r, flex_hal_r, tib_post_r)
fset2 = FunctionSet();
fset2.adoptAndAppend(Constant(ci*0.9322));
fset2.adoptAndAppend(Constant(ci*0.9322));
fset2.adoptAndAppend(Constant(ci*0.9322));
fset2.adoptAndAppend(Constant(ci*0.9322));
coInv.set_ControlFunctions(fset2);

%%%%% Adjust initial activation, else muscle will start at zero, and be
%%%%% subject to activation time delay dynamics
muscles=model.updMuscles;

%%% EVERTORS
muslist1={'per_long_r','per_brev_r','per_tert_r','ext_dig_r'};
for mus=muslist1
mus1=Thelen2003Muscle.safeDownCast( muscles.get(mus{:}) );
mus1.setDefaultActivation(ci);
stateSto.setDataColumn(stateSto.getStateIndex([mus{:} '.activation']), ArrayDouble(ci, stateSto.getSize)); 
end

%%% INVERTORS
muslist2={'ext_hal_r', 'flex_dig_r', 'flex_hal_r', 'tib_post_r'};
for mus=muslist2
mus2=Thelen2003Muscle.safeDownCast( muscles.get(mus{:}) );
mus2.setDefaultActivation(ci*0.9322);
stateSto.setDataColumn(stateSto.getStateIndex([mus{:} '.activation']), ArrayDouble(ci*0.9322, stateSto.getSize));
end

clear mus1 mus2

% Re-print the states (already adjused kinematics above)
stateSto.print(statesFileMod);


%% Adjust muscle parameters
muscles=model.updMuscles;

for mus=[muslist1 muslist2]
mus1=Thelen2003Muscle.safeDownCast( muscles.get(mus{:}) );
Fmax=mus1.get_max_isometric_force;
mus1.set_max_isometric_force( Fmax + Fmax*m1i )
end

%%%% Comment the above out and use this to effectively turn off all force
%%%% contributions from muscles (for verification)
% for mi = 1:muscles.getSize
% mus1=Thelen2003Muscle.safeDownCast( muscles.get(mi-1) );
% mus1.set_max_isometric_force( 0.0 )
% end

%% Execute forward simulation, print results, save settings
model.initSystem;

% print modified model for records
model.print([dirOutput '\' trial '_model.osim']);

fwd = ForwardTool(setupGeneric, true, false); 
fwd.setModel(model);

fwd.setName(trial);
fwd.setStatesFileName(statesFileMod)
fwd.setResultsDir(dirOutput);
fwd.setStartTime(time_initial);
fwd.setFinalTime(time_end);
fwd.setSolveForEquilibrium(true); %JUNE 21: switched from FALSE to TRUE

fwd.print([dirOutput '\' trial '_Setup_Foward.xml']);

tic
fwd.run();
toc
fprintf(1,['Foward Tool: success!\n']);

% rename the out.log so that it doesn't get overwritten
% % dirScript = matlab.desktop.editor.getActiveFilename;
% % dirScript=fileparts(dirScript);
copyfile([cd '\out.log'],[dirOutput '\' trial '_out.log'])

%% Load opensim results from text files
s = ReadOSIMtxt([dirOutput '\' trial '_states.sto']); s=s.data;
c = ReadOSIMtxt([dirOutput '\' trial '_controls.sto']); c=c.data;
kq = ReadOSIMtxt([dirOutput '\' trial '_Kinematics_q.sto']); kq=kq.data; 
kv = ReadOSIMtxt([dirOutput '\' trial '_Kinematics_u.sto']); kv=kv.data;
ka = ReadOSIMtxt([dirOutput '\' trial '_Kinematics_dudt.sto']); ka=ka.data;
f = ReadOSIMtxt([dirOutput '\' trial '_ForceReporter_forces.sto']); f=f.data; 
r = ReadOSIMtxt([dirOutput '\' trial '_JointReaction_ReactionLoads.sto']); r=r.data;
% a = ReadOSIMtxt([dirOutput '\' trial '_Actuation_force.sto']); a=a.data; 
% ap = ReadOSIMtxt([dirOutput '\' trial '_Actuation_power.sto']); ap=ap.data; 
% as = ReadOSIMtxt([dirOutput '\' trial '_Actuation_speed.sto']); as=as.data; 
bq = ReadOSIMtxt([dirOutput '\' trial '_BodyKinematics_pos_global.sto']); bq=bq.data; 
% bv = ReadOSIMtxt([dirOutput '\' trial '_BodyKinematics_vel_global.sto']); bv=bv.data;
% ba = ReadOSIMtxt([dirOutput '\' trial '_BodyKinematics_acc_global.sto']); ba=ba.data;

%% Compute discrete output metrics
Y1=max(kq.subtalar_angle_r)
Y2=max(kv.subtalar_angle_r);
Y3=max(kq.ankle_angle_r);
Y4=max(kv.ankle_angle_r);

%% Write outcome metrics to delimited text that NESSUS can read
if ~exist('dirN','var');  dirN=dirOutput; end
fid=fopen([dirN '\nessusOut.txt'],'w+'); %MUST be in cd, so is next to perturbed input file
fprintf(fid,'%.1f\n',Y1);
fprintf(fid,'%.1f\n',Y2);
fprintf(fid,'%.1f\n',Y3);
fprintf(fid,'%.1f\n',Y4);
fclose(fid);


%% Log a small subset of time trajectories for post-hoc processing, delete the rest

T1 = f(:,{'time','foot_floor_r_calcn_r_force_X','foot_floor_r_calcn_r_force_Y','foot_floor_r_calcn_r_force_Z','foot_floor_r_calcn_r_torque_X','foot_floor_r_calcn_r_torque_Y','foot_floor_r_calcn_r_torque_Z',...
          'cubic_ankle_bushing_r_tibia_r_torque_X','cubic_ankle_bushing_r_tibia_r_torque_Y','cubic_ankle_bushing_r_tibia_r_torque_Z',...
          'brace_exp_bushing_r_tibia_r_torque_X','brace_exp_bushing_r_tibia_r_torque_Y','brace_exp_bushing_r_tibia_r_torque_Z'});
T1.Properties.VariableNames = {'time','Fx','Fy','Fz','Mx','My','Mz',...
                                      'ankle_bushing_r_MX','ankle_bushing_r_MY','ankle_bushing_r_MZ',...
                                      'brace_bushing_r_MX','brace_bushing_r_MY','brace_bushing_r_MZ'};
T1.time = seconds(T1.time);
% T2 = kq(:,2:end);
T2 = kq(:,{'ankle_angle_r','subtalar_angle_r'});
T2.Properties.VariableNames = strcat(T2.Properties.VariableNames,'_q');
% T3 = kv(:,2:end);
T3 = kv(:,{'ankle_angle_r','subtalar_angle_r'});
T3.Properties.VariableNames = strcat(T3.Properties.VariableNames,'_u');
T4 = bq(:,{'tibia_r_X','tibia_r_Y','tibia_r_Z','tibia_r_Ox','tibia_r_Oy','tibia_r_Oz',...
           'calcn_r_X','calcn_r_Y','calcn_r_Z','calcn_r_Ox','calcn_r_Oy','calcn_r_Oz',...
           'talus_r_X','talus_r_Y','talus_r_Z','talus_r_Ox','talus_r_Oy','talus_r_Oz'});
TO = [T1 T2 T3 T4];
TO = table2timetable(TO);
TO = retime(TO,seconds(0:0.001:0.150),'mean');
TO{:,:} = round(TO{:,:},3);
writetimetable(TO,[dirN '\nessusOut2.csv']);

%%%%%%%%%===========
% rmdir(dirOutput,'s'); %COMMENT TO KEEP TEMP RESULTS
% delete('err.log'); 
% delete('out.log'); 
% exit
%%%%%%%%%===========




%% PLOT RESULTS
if PLOT_RESULTS
    
%%%% VALIDATE: inclined simulations published by DeMers
% dirDeMers=[dirN '\demers_2017_baseline\simulations of ankle co-activation and ankle stretch reflexes\'];
% trialDeMers = 'incline_30.0_activation_0.0';
% trialDeMers = 'incline_30.0_gain_5.0_delay_0.06';
% fD = ReadOSIMtxt([dirDeMers trialDeMers '_forces.sto']);
% fD=fD.data; forcesD=fD.Properties.VariableNames';
% cD = ReadOSIMtxt([dirDeMers trialDeMers '_controls.sto']);
% cD=cD.data; controlsD=cD.Properties.VariableNames';
% sD = ReadOSIMtxt([dirDeMers trialDeMers '_states.sto']);
% sD=sD.data; statesD=sD.Properties.VariableNames';

%%%% VALIDATE: level landing simulation published by DeMers
dirDeMers=[dirN '\demers_2017_baseline\simulated landing on level ground\results\'];
fD = ReadOSIMtxt([dirDeMers '\fwd_ForceReporter_forces.sto']);
fD=fD.data; forcesD=fD.Properties.VariableNames';
cD = ReadOSIMtxt([dirDeMers '\fwd_controls.sto']);
cD=cD.data; controlsD=cD.Properties.VariableNames';
sD = ReadOSIMtxt([dirDeMers '\fwd_states.sto']);
sD=sD.data; statesD=sD.Properties.VariableNames';

%PLOTS
t=s.time;  
tD=sD.time;
mass = model.getTotalMass(model.initSystem);
weight = mass*9.81;

figure; 
Nr = 4;
Nc = 1;

hax1=subplot(Nr,Nc,1); hold on; hold on; title([trial ': Joint Kinematics (dotted = baseline model)'],'interpreter','none')
p1=plot(t,kq.subtalar_angle_r,'g-');
pb=plot(t,kq.ankle_angle_r,'b-');
p1D=plot(tD,rad2deg(sD.subtalar_angle_r),'g:'); %DeMers verification 
p2D=plot(tD,rad2deg(sD.ankle_angle_r),'b:'); %DeMers verification 
XLM=get(hax1,'xlim');
plot([XLM(1) XLM(2)],[0 0],'k-')
legend([p1,pb],{'subtalar','ankle'})

hax2=subplot(Nr,Nc,2); hold on; title([trial ': Vertical Ground Reaction (dotted = baseline model)'],'interpreter','none')
pf=plot(t,f.foot_floor_r_platform_force_Y/weight*-1,'b-');
pfD=plot(tD,fD.foot_floor_r_platform_force_Y/weight*-1,'b:');
XLM=get(hax2,'xlim');
plot([XLM(1) XLM(2)],[0 0],'k-')
legend([pf,pfD],{'fy','fyD'})

passT=f.cubic_ankle_bushing_r_tibia_r_torque_X;
passTD=fD.cubic_ankle_bushing_r_tibia_r_torque_X;
passBC=f.brace_exp_bushing_r_tibia_r_torque_X;

hax3=subplot(Nr,Nc,3); hold on; title([trial ': Passive Bushing Moment (frontal): Brace vs. Anatomy (Nm) (dotted = baseline model)'],'interpreter','none')
hpt=plot(t,passT,'k-');
hptD=plot(tD,passTD,'k:');
% hbl=plot(t,passBL,'b-');
hbc=plot(t,passBC,'g-');
XLM=get(hax3,'xlim');
plot([XLM(1) XLM(2)],[0 0],'k-')
legend([hpt, hptD, hbc],{'anatomy','anatomyD','brace'},'location','best')

%% STATES
% figure; hold on;

statenames = s.Properties.VariableNames(:,2:end)';
statenamesD = sD.Properties.VariableNames(:,2:end)';

hax5=subplot(Nr,Nc,4); hold on
title('states - muscle activation (invertors & evertors)')
plotlist1 = statenames(contains(statenames,'activation'));
hs1 = plot(s.time, s{:,plotlist1});
set(hs1,{'displayname'},plotlist1);
set(hs1,{'tag'},plotlist1);
plotlist1D = statenamesD(contains(statenamesD,'activation'));
hs1D = plot(sD.time, sD{:,plotlist1D}, 'linestyle',':');
set(hs1D,{'displayname'},plotlist1D);
set(hs1D,{'tag'},plotlist1D);

[ h_show,h_hide ] = FindObjs( hax5, [musEv musInv],'line');

end
