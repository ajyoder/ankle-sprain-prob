function [c,p,rf2,mf2] = cubicBushingExp(f)
% Returns a cubic spline of ankle torque-angle stiffness, scaled from
% baseline anatomy (f = 0)
%
% Inputs: 
% f =  [-1,1] scale factor on raw torque-angle data
%      **user must comment out code below, to choose to apply the scale factor to moment data 
%      **(scale compliance at fixed angles) or to angle data (scale flexibility at fixed torques)
%      **supply f=NaN input to return zero-stiffness no brace condition
%      **note (f=0) returns the baseline unscaled expressions
% Outputs:
%    c = (3x1) cell array of expressions, strings
%    p = (3x4) array of cubic polynomial coeffs, for use with polyval()
%    [rf2,mf2] = (N x 3) matrix of angle & torque data 
%

fi=f;

d=(-90:0.1:90)';
r=d/180*pi;

%% Cadaveric torque-angle data
%%% ** NOTE: signage a bit confusing
%%%% From (Chen 1988)  +X = SAG superior, +Y = FRO anterior, +Z = TRA superior
%%%%                   And data are plotted flipped, X-Y = moment-angle
%%%% From (DeMers 2017)  +X = FRO anterior, +Y = TRA superior, +Z = SAG right 
%%%%                     these below data are pulled out of the Osim XML,
%%%%                     so should conform to DeMers signage
mx=9.52095038.*r-29.57608094.*r.^2+169.01588105.*r.^3;
my=3.61796125.*r-10.1612135.*r.^2+64.5279144.*r.^3;
mz=5.98187224.*r+15.74951827.*r.^2+33.6780792.*r.^3;
m=[mx my mz];

%% transform factor so that negative is a reduction
if f<0
  f = 1/(1 + -1*f); 
else
  f = 1 + f;
end

%% USER: choose between scaling compliance or flexibility, see (Wright et al 2000)
% (1) Scale compliance (moment)
% rf2 = r; % f>1 = more stiff (less flexibile)
% mf2 = m * f; % f>1 = more stiff (less compliance)

% (2) Scale flexibility (angle)
rf2 = r / f; % f>1 = more stiff (less flexibile)
mf2 = m ; % f>1 = more stiff (less compliance)

% (3) Scale flexibility & compliance
% rf2 = r / f; % f>1 = more stiff (less flexibile)
% mf2 = m * f; % f>1 = more stiff (less compliance)

%% Compute polynomials, numeric and string-based for input to OSIM model
c=cell(3,1); 
comp={'x','y','z'};
for mi=1:3
  if isnan(f) 
    p(mi,:) = zeros(1,4);
  else
    p(mi,:) = polyfit(rf2,mf2(:,mi),3);
  end
  
str1=sprintf('%.5f*S%+.5f*S^2%+.5f*S^3',p(mi,3),p(mi,2),p(mi,1));
str2=strrep(str1,'S',['theta_' comp{mi}]);
c{mi,:}=str2;
end


%% Quick verification plot vs. baseline trajectories
 
% hfig=figure('name',['rf = ' num2str(fi) sprintf(' (%3.0f%%)',f*100)]);
% for ii=1:3
% subplot(3,1,ii); hold on;
% switch ii
%     case 1; title('X: Frontal (+ Anterior)');
%     case 2; title('Y: Transverse (+ Superior)');
%     case 3; title('Z: Sagittal (+ Right)');
% end
% plot(d,m(:,ii),'k:'); %%%baseline
% plot(d,polyval(p(ii,:),r),'b-');
% xlim([-40 40]); xlabel('angle (deg)')
% ylim([-30 30]); ylabel('torque (N-m)')
% end



