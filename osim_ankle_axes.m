%%%% Joint definitions for Gait2392 ankle-foot:
%%%% https://simtk-confluence.stanford.edu:8443/display/OpenSim33/OpenSim+Models#OpenSimModels-Joints
%%%% 
%%%% Each appear to be classs <CustomJoint> which can accept 3 arbitrary (potentially non-orthogonal)
%%%% rotation axes, followed by 3-translations
%%%%
%%%% In this case each of the talocrural and subtalar
%%%% joints has 1 oblique, body-fixed unit vector at a specified origin so,
%%%% given 1 input joint angles:
%%%% (1) convert angle-axis to DCMs, and extract 3DOF XYZ component rotations
%%%% (2) compute DCM of total ankle complex (talocrural followed by subtalar) 
%%%% (3) extract component XYZ rotations to facilate comparison with common ISB convention

%%%% OpenSim forum posts on extracting moments from a FORWARD dynamic simulation:
%%%% Not straightfoward, may be simpler to just run Inv Dynamic tool on
%%%% forward sim outputs, but won't give muscle vs. passive moment
%%%% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=91&t=10365&p=28702&start=0&view=
%%%% https://simtk.org/plugins/phpBB/viewtopicPhpbb.php?f=91&t=8714&p=24061&start=0&view=

clear all;close all;clc;
tib_angle = 34; %ankle
sub_angle = 48; %subtalar

%% OpenSim Talocrural 1DOF joint
%%%% PARENT = TIBIA
%%%% CHILD = TALUS

%%%<location_in_parent> = TIBIA
tibO = [0 -0.464696391450486 0]; %mid-ankle, [0,0,0] is knee joint center 

%%%% TransformAxis for <SpatialTransform>
tibax = [-0.10501355 -0.17402245 0.97912632]; %assigned to 'ankle_r' coordinate
%tibax = [0 0 1];
%%%% In addtion to being primarily sagittal motion... with plantarflexion,
%%%% there is notable frontal EVERSION + transverse EXTERNAL rotation at

[tib_rx, tib_ry, tib_rz] = axang2euler(tibax, deg2rad(tib_angle)); %radians
tib_dx = rad2deg(tib_rx); %inversion (+), eversion (-)
tib_dy = rad2deg(tib_ry); %internal (+), external (-)
tib_dz = rad2deg(tib_rz); %dorsiflexion (+), plantarflexion (-)

R_talus_tibia = axang2rotmat(tibax, deg2rad(tib_angle)); %DCM of TALUS rel. TIBIA 

fprintf(1,'<TALOCRURAL> rotation = %.1f , INV(+)/EV=%.1f  INT(+)/EXT=%.1f  DF(+)/PF=%.1f\n',tib_angle, tib_dx, tib_dy, tib_dz)

%% OpenSim Subtalar 1DOF joint
%%%% PARENT = TALUS
%%%% CHILD = CALC

%%%<location_in_parent>
subO = [-0.051042192975949 -0.0439044493611044 0.00828899258498085];

%%%% TransformAxis for <SpatialTransform>
subax = [0.78717961 0.60474746 -0.12094949]; % assigned to COORD subtalar_r
%subax = [1 0 0];
%%%% This means that in addtion to being primarily frontal... with
%%%% (+)inversion, there is large transverse INTERNAL + slight sagittal PLANTAR rotation

fprintf(1,'Subtalar axis fixed incline, sagittal = %.1f\n',atand(subax(2)/subax(1))); %subtalar axis incline in foot sagittal YX plane
fprintf(1,'Subtalar axis fixed offset, transverse = %.1f\n',atand(subax(3)/subax(1)));

[sub_rx, sub_ry, sub_rz] = axang2euler(subax, deg2rad(sub_angle)); %radians
sub_dx = rad2deg(sub_rx); %inversion (+), eversion (-)
sub_dy = rad2deg(sub_ry); %internal (+), external (-)
sub_dz = rad2deg(sub_rz); %dorsiflexion (+), plantarflexion (-)

R_calc_talus = axang2rotmat(subax, deg2rad(sub_angle)); %DCM of CALC rel TALUS 

fprintf(1,'<SUBTALAR> rotation = %.1f , INV(+)/EV=%.1f  INT(+)/EXT=%.1f  DF(+)/PF=%.1f\n',sub_angle, sub_dx, sub_dy, sub_dz)


%% 

R_calc_tibia = R_calc_talus * R_talus_tibia ; %calc orientation relative to tibia

[calc_tibia_rx, calc_tibia_ry, calc_tibia_rz] = rotmat2euler( R_calc_tibia );
calc_tibia_dx = rad2deg(calc_tibia_rx);
calc_tibia_dy = rad2deg(calc_tibia_ry);
calc_tibia_dz = rad2deg(calc_tibia_rz);

fprintf(1,'<TIB-CALC> rotation, INV(+)/EV=%.1f  INT(+)/EXT=%.1f  DF(+)/PF=%.1f\n',calc_tibia_dx, calc_tibia_dy, calc_tibia_dz)

%% Tony's Check
%tib_angle = 34; %ankle
%sub_angle = 30; %subtalar
%tibax = [0 0 1]; %[-0.10501355 -0.17402245 0.97912632];
%subax = [1 0 0]; %[0.78717961 0.60474746 -0.12094949];

alpha = deg2rad(tib_angle)*tibax;
[flx,lat,axl] = helical2Cardan(alpha(1),alpha(2),alpha(3));
dfx = rad2deg(flx); inv = rad2deg(lat); int = rad2deg(axl);
fprintf(1,'\n<** TALOCRURAL> tib_angle = %6.1f, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',tib_angle,dfx,inv,int)
Rz = [cos(flx) sin(flx) 0;-sin(flx) cos(flx) 0;0 0 1];
Rx = [1 0 0;0 cos(lat) sin(lat);0 -sin(lat) cos(lat)];
Ry = [cos(axl) 0 -sin(axl);0 1 0;sin(axl) 0 cos(axl)];
R1 = Ry*Rx*Rz;

alpha = deg2rad(sub_angle)*subax;
[flx,lat,axl] = helical2Cardan(alpha(1),alpha(2),alpha(3));
dfx = rad2deg(flx); inv = rad2deg(lat); int = rad2deg(axl);
fprintf(1,'<**** SUBTALAR> sub_angle = %6.1f, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',sub_angle,dfx,inv,int)
Rz = [cos(flx) sin(flx) 0;-sin(flx) cos(flx) 0;0 0 1];
Rx = [1 0 0;0 cos(lat) sin(lat);0 -sin(lat) cos(lat)];
Ry = [cos(axl) 0 -sin(axl);0 1 0;sin(axl) 0 cos(axl)];
R2 = Ry*Rx*Rz;

Rtot = R2*R1;
[alphax,alphay,alphaz] = CardanR2helical(Rtot);
[flx,lat,axl] = helical2Cardan(alphax,alphay,alphaz);
dfx = rad2deg(flx); inv = rad2deg(lat); int = rad2deg(axl);
fprintf(1,'<TIB-CALCANEUS> ******************, DF(+)/PF = %6.1f, INV(+)/EV = %6.1f, INT(+)/EXT = %6.1f\n',dfx,inv,int)

% disp ' ';
% disp '********************************************************************';
% disp 'Final tibia-calcaneus rotation angles checked by alternate code... ';
% dorsi = rad2deg(flx)
% inver = rad2deg(lat)
% inter = rad2deg(axl)
% 
% % R_calc_tibia = R_calc_talus' * R_talus_tibia';
% [alphax,alphay,alphaz] = CardanR2helical(R_calc_tibia);
% [flx,lat,axl] = helical2Cardan(alphax,alphay,alphaz);
% disp ' ';
% disp '********************************************************************';
% disp 'Final tibia-calcaneus rotation angles with R_calc_tibia as global... ';
% dorsi = rad2deg(flx)
% inver = rad2deg(lat)
% inter = rad2deg(axl)

% lat = deg2rad(17) % angle for Rx (radians)
% axl = deg2rad(22) % angle for Ry (radians)
% flx = deg2rad(54) % angle for Rz (radians)
% Rx = [1 0 0;0 cos(lat) sin(lat);0 -sin(lat) cos(lat)]; % body-fixed x rotation
% Ry = [cos(axl) 0 -sin(axl);0 1 0;sin(axl) 0 cos(axl)]; % body-fixed y rotation
% Rz = [cos(flx) sin(flx) 0;-sin(flx) cos(flx) 0;0 0 1]; % body-fixed z rotation
% % Rtot = Ry*Rx*Rz
% % [rx,ry,rz] = rotmat2euler(Rtot)
% Rtot = Rz*Ry*Rx; % body-fixed x-y-z rotation sequence
% [rx,ry,rz] = rotmat2euler(Rtot) % this does not produce the original angles
% % Rtot = Ry'*Rx'*Rz'
% % [rx,ry,rz] = rotmat2euler(Rtot)
% Rtot = Rz'*Ry'*Rx'; % transpose makes each matrix global-fixed, still in body-fixed x-y-z order
% [rx,ry,rz] = rotmat2euler(Rtot) % this DOES produce the original angles
% % Rtot = Rx*Ry*Rz
% % [rx,ry,rz] = rotmat2euler(Rtot')

%% Unit Tests
    
%%%%% ##### should give dz =  90; R = [0 -1 0; 1 0 0; 0 0 1]
% axis1 = [0 1 0];  angle1 = 45; 
% 
% [rX, rY, rZ] = axang2euler(axis1, deg2rad(angle1));
% dX = rad2deg(rX)
% dY = rad2deg(rY)
% dZ = rad2deg(rZ)
% 
% R1 = axang2rotmat(axis1,deg2rad(angle1))
% 
% [r2X,r2Y,r2Z] = rotmat2euler(R1)
% d2X = rad2deg(r2X)
% d2Y = rad2deg(r2Y)
% d2Z = rad2deg(r2Z)

%% Support functions

function [R] = axang2rotmat(v,theta)
%%% https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

kx = v(1); ky = v(2); kz = v(3);

K = [0 -1*kz ky; kz 0 -1*kx; -1*ky kx 0];

R = eye(3) + sin(theta)*K + (1-cos(theta))*K^2;

end

function [rotX, rotY, rotZ] = axang2euler(v,theta) 
%%%% https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
%%%% https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
%%%% inputs/outputs in RADIANS

s=sin(theta);
c=cos(theta);
t=1-c;

vn = vecnorm(v,2); %normalize if not unit vector
if (abs(vn) < 0.998) && (abs(vn) > 1.002)
v = v ./ vn;
warning('Passed angle was not unit vector!')
end
x = v(1); y = v(2); z = v(3);

if ((x*y*t + z*s) > 0.998)  %north pole singularity 
		rotY = 2*atan2(x*sin(theta/2), cos(theta/2));
		rotZ = pi/2;
		rotX = 0;
		return;
end
if ((x*y*t + z*s) < -0.998) %south pole singularity 
		rotY = -2*atan2(x*sin(theta/2), cos(theta/2));
		rotZ = -pi/2;
		rotX = 0;
		return;
end

rotX = atan2(x * s-y * z * t , 1 - (x^2 + z^2) * t); %bank
rotY = atan2(y * s - x * z * t , 1 - (y^2 + z^2 ) * t); %heading
rotZ = asin(x * y * t + z * s); %attitude

end

function [rx,ry,rz] = rotmat2euler(R)
%ROTMAT2EULER Summary of this function goes here
%   Detailed explanation goes here

%%%% XYZ fixed sequence

ry = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2)); %2

if round(ry,4) == round(pi/2,4);    rz = 0; rx = atan2(R(1,2),R(2,2)); return; end
if round(ry,4) == round(-1*pi/2,4); rz = 0; rx = -1*atan2(R(1,2),R(2,2)); return; end
    
rz = atan2( R(2,1)/cos(ry), R(1,1)/cos(ry)); %3

rx = atan2( R(3,2)/cos(ry), R(3,3)/cos(ry)); %1

end

% ********************************************************************************
% Tony's functions...

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