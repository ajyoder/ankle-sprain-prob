%%%%  Purpose, resolve OpenSim anatomic joint kinematics (helical axes) to spatial
%%%%  kinematics of reference frames between the tibia and calcaneus
%%%%  bodies. This will facilitate comparison with ISB conventions as used
%%%%  more commonly in clinical biomechanics from motion capture (i.e. Visual3D) 
%%%%
%%%%  Steps:
%%%% (1) per joint, convert helical to 3 cardan angles (sequence = 3-1-2 ZXY Sagittal-Frontal-Transverse)
%%%% (2) compute combined rotation of tibia-calcaneues ankle complex (talocrural followed by subtalar) 
%%%% (3) compute cardan angular velocities of ankle complex, given known joint anuglar velocities
%%%%
%%%% Notes:
%%%% Joint definitions for Gait2392 ankle-foot:
%%%% https://simtk-confluence.stanford.edu:8443/display/OpenSim33/OpenSim+Models#OpenSimModels-Joints
%%%% 
%%%% Each is <CustomJoint> which can accept 3 arbitrary (potentially non-orthogonal)
%%%% rotation axes, followed by 3-translations. The talocrural and subtalar
%%%% joints each are 1 oblique axis in the parent body 

clear all;
close all;
clc;

%% VARIABLE INPUTS

%%%% Logged OpenSim file from a Nessus trial execution
%%%% Note: moments are between the calc and tibia, borne by the passive anatomy bushing ONLY, 
%%%% in the tibia base reference frame. Torque borne by the brace bushing
%%%% in parallel (when present) is also available, but wasnt logged in last run of MC's
TT=readtable('nessusOut2.csv');
TT.time = seconds(str2double(strrep(TT.time,' sec','')));
TT = table2timetable(TT);
q1 = TT{:,'ankle_angle_r'}; %ankle
q2 = TT{:,'subtalar_angle_r'}; %subtalar

%%%% Analyze instant of max inversion, for now
[submax,isubmax]=max(q2);
tib_angle = q1(isubmax); %ankle
sub_angle = q2(isubmax); %subtalar

%%%% For verification:
% tib_angle = -34; %ankle (DF+)
% sub_angle = 40; %subtalar (INV+)

%% FIXED INPUTS
tibax = [-0.10501355 -0.17402245 0.97912632]; %'ankle_r' OpenSim coordinate
subax = [0.78717961 0.60474746 -0.12094949];

%%%% For verification: 
%tibax = [0 0 1]; % PURE SAGITTAL +Z
%subax = [1 0 0]; % PURE FRONTAL +X

%%% ORIGIN: <location_in_parent> of each axis, as needed
%%% tibO = [0 -0.4647 0]; 
%%% subO = [-0.0510 -0.0439 0.0083];

%% (1) Helical 2 Cardan, each joint

alpha1 = deg2rad(tib_angle)*tibax;
[flx1,lat1,axl1] = helical2Cardan(alpha1(1),alpha1(2),alpha1(3));
dfx1 = rad2deg(flx1); inv1 = rad2deg(lat1); int1 = rad2deg(axl1);
fprintf(1,'<** TALOCRURAL> tib_angle = %6.1f, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',tib_angle,dfx1,inv1,int1)
Rz1 = [cos(flx1) sin(flx1) 0;-sin(flx1) cos(flx1) 0;0 0 1];
Rx1 = [1 0 0;0 cos(lat1) sin(lat1);0 -sin(lat1) cos(lat1)];
Ry1 = [cos(axl1) 0 -sin(axl1);0 1 0;sin(axl1) 0 cos(axl1)];
R1 = Ry1*Rx1*Rz1;

alpha2 = deg2rad(sub_angle)*subax;
[flx2,lat2,axl2] = helical2Cardan(alpha2(1),alpha2(2),alpha2(3));
dfx2 = rad2deg(flx2); inv2 = rad2deg(lat2); int2 = rad2deg(axl2);
fprintf(1,'<**** SUBTALAR> sub_angle = %6.1f, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',sub_angle,dfx2,inv2,int2)
Rz2 = [cos(flx2) sin(flx2) 0;-sin(flx2) cos(flx2) 0;0 0 1];
Rx2 = [1 0 0;0 cos(lat2) sin(lat2);0 -sin(lat2) cos(lat2)];
Ry2 = [cos(axl2) 0 -sin(axl2);0 1 0;sin(axl2) 0 cos(axl2)];
R2 = Ry2*Rx2*Rz2;

%% (2) Combined Ankle Complex

Rtot = R2*R1;
[alphax,alphay,alphaz] = CardanR2helical(Rtot);
[flx3,lat3,axl3] = helical2Cardan(alphax,alphay,alphaz);
dfx3 = rad2deg(flx3); inv3 = rad2deg(lat3); int3 = rad2deg(axl3);
fprintf(1,'<TIB-CALCANEUS> ******************, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',dfx3,inv3,int3)

%% (3) Angular Velocity 
%%%% Could be most efficient to use Corke's rtb toolbox

% Joint velocities (about oblique helical axis)
q1d=[NaN; diff(q1)];
q2d=[NaN; diff(q2)];

% Map joint velocities (q1d,q2d) to spatial omega_ZXY (calcaneus rel. tibia ground)
%%%% https://robotacademy.net.au/lesson/the-analytic-jacobian/
%%%% https://robotacademy.net.au/lesson/mapping-3d-spatial-velocity-between-coordinate-frames/

%# TO DO
%# TO DO
%# TO DO


%% (4) Moments: Read in, extract peaks, from passive anatomy and brace bushing

%%%% OSim Analysis output file "ForceReporter_forces.sto", contains all
%%%% generalized system forces named with the ref. frame in which they
%%%% are expressed.  Example: "cubic_ankle_bushing_r.tibia_r.torque.X"
%%%% Our bushing definitions <ExpressionBasedBushingForce>, have
%%%% M(theta) expressions specifed about BODY 2 axes (calcn_r), as a function of XYZ body-fixed
%%%% Euler rotations.

%# TO DO
%# TO DO
%# TO DO




%% Support functions





% ********* Tony Petrella:

function [flx,lat,axl] = helical2Cardan(alphax,alphay,alphaz)
% input angles in radians, output angles in radians
% v1 completed 18 Nov 2016

% this function converts rotation angles used in Abaqus (which perform a
% helical axis rotation) into a Cardan angle sequence corresponding to lumbar joint
% angles computed by OpenSim... that is, a 3-1-2 body-fixed rotation
% sequence, which corresponds to flexion, lateral bending, axial rotation
% Note: inputs are supplied in units of radians

% overall rotation magnitude is computed from the components
alpha = norm([alphax alphay alphaz]);

% rotation vector components are used to define the rotation axis
p = [alphax alphay alphaz]/alpha;

% make arbitrary local frame with x-axis lying along helical rotation axis
hx = p;
% create a random vector and get the part perpendicular to p
ry = [mean(mean(rand(5))) mean(mean(rand(5))) mean(mean(rand(5)))];
hy = unit(ry - dot(ry,p)*p); % unit vector for part perpendicular to p
hz = cross(hx,hy); % simple cross product for z-axis definition

% find local representation of vector we wish to rotate by projecting it
% into the arbitrary local frame constructed above
xlocal = [hx;hy;hz]*[1 0 0]';
% rotate xlocal around the rotation axis, which is local x-axis; note Rx'
% is used here because we are rotating the vector rather than the ref frame
Rx = [1 0 0;0 cos(alpha) sin(alpha);0 -sin(alpha) cos(alpha)];
xrot = Rx'*xlocal;
% put xrot back in global components by reversing the local transformation
bx = [hx;hy;hz]'*xrot;

% find local representation of vector we wish to rotate by projecting it
% into the arbitrary local frame constructed above
ylocal = [hx;hy;hz]*[0 1 0]';
% rotate ylocal around the rotation axis, which is local x-axis; note Rx'
% is used here because we are rotating the vector rather than the ref frame
Rx = [1 0 0;0 cos(alpha) sin(alpha);0 -sin(alpha) cos(alpha)];
yrot = Rx'*ylocal;
% put yrot back in global components by reversing the local transformation
by = [hx;hy;hz]'*yrot;

% find local representation of vector we wish to rotate by projecting it
% into the arbitrary local frame constructed above
zlocal = [hx;hy;hz]*[0 0 1]';
% rotate zlocal around the rotation axis, which is local x-axis; note Rx'
% is used here because we are rotating the vector rather than the ref frame
Rx = [1 0 0;0 cos(alpha) sin(alpha);0 -sin(alpha) cos(alpha)];
zrot = Rx'*zlocal;
% put zrot back in global components by reversing the local transformation
bz = [hx;hy;hz]'*zrot;

% Cardan angles
L = -unit(cross(by,[0 0 1]));
flx = +asin(-dot(L,[0 1 0]));
lat = +asin(dot([0 0 1],by));
axl = -asin(dot(L,bz));
end

function [alphax,alphay,alphaz] = CardanR2helical(Rtot)
% input angles in radians, output angles in radians
% v1 completed 18 Nov 2016

% this function converts a Cardan angle sequence corresponding to lumbar joint
% angles (rad) computed by OpenSim... that is, a 3-1-2 body-fixed rotation
% sequence, which corresponds to flexion, lateral bending, axial rotation
% into rotation angles used in Abaqus (which perform a helical axis rotation) 

% this is the total rotation matrix represented by the given Cardan sequence
% Note: these angles must be given in radians
%Rz = [cos(flx) sin(flx) 0;-sin(flx) cos(flx) 0;0 0 1];
%Rx = [1 0 0;0 cos(lat) sin(lat);0 -sin(lat) cos(lat)];
%Ry = [cos(axl) 0 -sin(axl);0 1 0;sin(axl) 0 cos(axl)];
%Rtot = Rz*Ry*Rx; % NOTE: changed currently to match BodyKinematics sequence
%Rtot = Ry*Rx*Rz; % Previously Ry*Rx*Rz for the 3-1-2 sequence

% the projection of any vector onto the rotation axis before the rotation
% must be identcal after the rotation... we apply this identity to the
% three unit vectors of the body-fixed frame, which start as eye(3) and end
% up as Rtot after the rotation; the vector p is the resulting rotation axis
A = eye(3)-Rtot;
p = unit(fsolve(@(p) A*p,[1;1;1],optimoptions('fsolve','Display','off')));

% now find the components of a vector perpendicular to the rotation axis...
% here we use the x-axis with starts as [1 0 0] and becomes the first row
% of Rtot after the rotation... same should work if we use y-axis or z-axis
v1 = [1 0 0] - dot([1 0 0],p)*p';m1 = norm(v1);
v2 = Rtot(1,:) - dot(Rtot(1,:),p)*p';m2 = norm(v2);
alpha = asin(norm(cross(v1,v2))/m1/m2);
% now confirm the rotation axis is pointing in the right direction...
% if we cross v1 with v2 the result should point in the same direction as p
if(dot(cross(v1,v2),p)<0),p=-p;end

% % using y-axis or z-axis should produce identical results (they do)
% v1 = [0 1 0] - dot([0 1 0],p)*p';m1 = norm(v1);
% v2 = Rtot(2,:) - dot(Rtot(2,:),p)*p';m2 = norm(v2);
% alpha = asin(norm(cross(v1,v2))/m1/m2)
% 
% v1 = [0 0 1] - dot([0 0 1],p)*p';m1 = norm(v1);
% v2 = Rtot(3,:) - dot(Rtot(3,:),p)*p';m2 = norm(v2);
% alpha = asin(norm(cross(v1,v2))/m1/m2)

alphax = p(1)*alpha;
alphay = p(2)*alpha;
alphaz = p(3)*alpha;
end

function n = unit(x)
n = x/norm(x);
end


%% https://github.com/petercorke/spatial-math
function S = skew(v)
    if isvec(v,3)
        % SO(3) case
        S = [  0   -v(3)  v(2)
              v(3)  0    -v(1)
             -v(2) v(1)   0];
    elseif isvec(v,1)
        % SO(2) case
        S = [0 -v; v 0];
    else
        error('SMTB:skew:badarg', 'argument must be a 1- or 3-vector');
    end
    
end