clearvars;close all;clc;

% the folder names for all STUDYs in the prob project
STUDY = {'MCv26c','MCv27c','MCv28c','MCv29c','MCv30c','MCv31c'};

row = 1; % where do the data start in the CSV output file... indexed from 0, row 1 is 2nd row
col = 7; % where do the data start in the CSV output file... indexed from 0, col 7 is 8th col

jmax = 1001; % this is the total number of trials in each STUDY... nominally, jmax = 1001

% preallocate memory for the JCS angles array for all STUDYs
% store JCS angles in a 4D array jcs(r,c,p,s) where...
% r = 1 to 151 for time incs 0-150 corresponding to 0.15s with time incs of 0.001
% c = 1 to 11 corresponding to...
%     (1-3) dorsiflexion, inversion, and internal rotation (all positive rotations)
%     (4-6) angular velocities of dorsiflexion, inversion, and internal rotation
%     (7-11) moment magnitude around talocrural, subtalar, JCS_dfx, JCS_inv, JCS_int 
% p = trial number 1-1001 corresponding to trials 000000 to 001000
% s = 1 to 6 for STUDY number
jcs = zeros(151,11,jmax,6);
atm = zeros(151,4,jmax,6); % similar array for anatomical q1, q2, q1_dot, q2_dot

% preallocate similar array contains only max value of each curve over the "r" time incs listed above
jcs_mx = zeros(11,jmax,6);  % this array stores the max value of each curve
atm_mx = zeros(4,jmax,6);   % similar array for the 4 anatomical variables
jcs_mx_i = jcs_mx;          % this identical array stores the vector index of each max value for plotting
atm_mx_i = atm_mx;          % similar array for the 4 anatomical variables

% preallocate memory for the scalar outcomes, which are...
% first, extract max value of each curve, then compute mean and stdev
% across 1001 trials... mean and stdev are then the only two scalar outcomes retained
% store outcomes in a 3D array out(r,c,p) where...
% r = 1 to 8 rows: dfx, inv, int, dfx_dot, inv_dot, int_dot, M_talocrural, M_subtalar
%     9 to 15 rows: M_JCS_dfx, M_JCS_inv, M_JCS_int, q1, q2, q1_dot, q2_dot
% c = 1 to 2 for mean and stdev
% p = 1 to 6 for STUDY number
out = zeros(15,2,6);

tic;

for i = [1 2 3 4 5 6]    % iterate on studies listed above in STUDY
    for j = 1:jmax       % iterate on F/###### trial folder
        
        clc; disp(['Working on STUDY ',STUDY{i},' trial... ',sprintf('%06d',j-1)]);
        
        % write the filename of the output file of interest
        path = ['C:/nessus/ankle-sprain/' STUDY{i},'/F/',sprintf('%06d',j-1),'/nessusOut2.csv'];
        
        % read the columns of output data for kinematics/kinetics
        % cols in input file, indexed from 0 (NOT from 1)...
        % 7=ankle_MX, 8=ankle_MY, 9=ankle_MZ, 10=brace_MX, 11=brace_MY, 12=brace_MZ (all global)
        % 13=q1(talocrural rot), 14=q2 (subtalar rot), 15=q1_dot, 16=q2_dot
        % 17,18,19=tib X,Y,Z global coordinates of COM
        % 20,21,22=tib x,y,z body-fixed rotations for x-y-z sequence
        d = [dlmread(path,',',[row,col,row+150,col+15])...   % these are cols 7-22
             dlmread(path,',',[row,col+19,row+150,col+24])]; % these are cols 26-31
        
        ankle_MX  = d(:,1);  % ankle moment // global X component
        ankle_MY  = d(:,2);  % ankle moment // global Y component
        ankle_MZ  = d(:,3);  % ankle moment // global Z component
        brace_MX  = d(:,4);  % brace moment // global X component
        brace_MY  = d(:,5);  % brace moment // global Y component
        brace_MZ  = d(:,6);  % brace moment // global Z component
        tc_angle  = d(:,7);  % talocrural angle // OSim ankle joint axis
        st_angle  = d(:,8);  % subtalar angle // OSim subtalar joint axis
        tc_veloc  = d(:,9);  % talocrural joint angular velocity // q1_dot
        st_veloc  = d(:,10); % subtalar joint angular velocity // q2_dot
        tibia_r_X = d(:,11); % tibia COM global X coordinate
        tibia_r_Y = d(:,12); % tibia COM global Y coordinate
        tibia_r_Z = d(:,13); % tibia COM global Z coordinate
        tibia_x_O = d(:,14); % tibia frame body-fixed x rotation, x-y-z sequence
        tibia_y_O = d(:,15); % tibia frame body-fixed y rotation, x-y-z sequence
        tibia_z_O = d(:,16); % tibia frame body-fixed z rotation, x-y-z sequence     
        calcn_x_O = d(:,17); % calcaneus frame body-fixed x rotation, x-y-z sequence
        calcn_y_O = d(:,18); % calcaneus frame body-fixed y rotation, x-y-z sequence
        calcn_z_O = d(:,19); % calcaneus frame body-fixed z rotation, x-y-z sequence
        talus_r_X = d(:,20); % talus COM global X coordinate
        talus_r_Y = d(:,21); % talus COM global Y coordinate
        talus_r_Z = d(:,22); % talus COM global Z coordinate
        
        % linkage is TIBIA.{ankle-joint}.TALUS.{subtalar-joint}.CALCANEUS
        % the ref frame for each body is initially parallel to Global
        tc_ax = [-0.10501355 -0.17402245 +0.97912632]'; % talocrural axis defined in tibia body frame
        tibax = [+0.00000000 +0.00000000 +1.00000000]'; % considering JCS motion, talocrural is exactly Z-axis
        st_ax = [+0.78717961 +0.60474746 -0.12094949]'; % subtalar axis defined in talus body frame
        
        % for each time increment in a single simulation
        % 1 to 151 for time points 0-150 corresponding to 0.15s with time incs of 0.001
        for k = 1:length(d)
            
            % rotation matrices associated with tibia body-fixed Euler angle sequence x-y-z
            Rx = [1 0 0;0 cosd(tibia_x_O(k)) sind(tibia_x_O(k));0 -sind(tibia_x_O(k)) cosd(tibia_x_O(k))];
            Ry = [cosd(tibia_y_O(k)) 0 -sind(tibia_y_O(k));0 1 0;sind(tibia_y_O(k)) 0 cosd(tibia_y_O(k))];
            Rz = [cosd(tibia_z_O(k)) sind(tibia_z_O(k)) 0;-sind(tibia_z_O(k)) cosd(tibia_z_O(k)) 0;0 0 1];
            Rt = Rz*Ry*Rx; % transforms from Global to tibia
            
            % transform talocrural axis from tibia frame to Global
            TIB_Z = Rt'*tc_ax; % this is the JCS Z-axis for tibia defined in Global
            % tibia long axis defined in Global frame
            TL_AX = unit([tibia_r_X(k) tibia_r_Y(k) tibia_r_Z(k)] - [talus_r_X(k) talus_r_Y(k) talus_r_Z(k)])';
            % cross product of TL_AX and TC_AX gives JCS X-axis for tibia
            TIB_X = cross(TL_AX,TIB_Z);
            % cross product of TIB_Z and TIB_X gives JCS Y-axis for tibia
            TIB_Y = cross(TIB_Z,TIB_X);
            
            % rotation matrices associated with calcaneus body-fixed Euler angle sequence x-y-z
            Rx = [1 0 0;0 cosd(calcn_x_O(k)) sind(calcn_x_O(k));0 -sind(calcn_x_O(k)) cosd(calcn_x_O(k))];
            Ry = [cosd(calcn_y_O(k)) 0 -sind(calcn_y_O(k));0 1 0;sind(calcn_y_O(k)) 0 cosd(calcn_y_O(k))];
            Rz = [cosd(calcn_z_O(k)) sind(calcn_z_O(k)) 0;-sind(calcn_z_O(k)) cosd(calcn_z_O(k)) 0;0 0 1];
            Rc = Rz*Ry*Rx; % transforms from Global to calcaneus
            % the rows of this rotation matrix are the Global components of the calcaneus frame
            % these are the axes of the JCS frame for the calcaneus expressed in Global components
            CAL_X = Rc(1,:)';
            CAL_Y = Rc(2,:)';
            CAL_Z = Rc(3,:)';
            
            % JCS angles
            L   = -unit(cross(CAL_Y,TIB_Z)); % line of nodes
            dfx = +asind(-dot(L,TIB_Y));     % dorsiflexion about common Z
            inv = +asind(dot(TIB_Z,CAL_Y));  % inversion about line of nodes
            int = -asind(dot(L,CAL_Z));      % internal rotation about calcaneus y
            
            % save the JCS angles in the results array
            jcs(k,1,j,i) = dfx; jcs(k,2,j,i) = inv; jcs(k,3,j,i) = int;
            
            % helical2Cardan() needs something non-zero to work
            if(tc_angle(k)==0),tc_angle(k)=0.001;end
            % subtalar axis fixed in talus rotates around talocrural axis (tc_ax) defined in tibia frame
            alpha = deg2rad(tc_angle(k))*tc_ax; % helical axis rotation of talus relative to tibia
            [flx,lat,axl] = helical2Cardan(alpha(1),alpha(2),alpha(3)); % this function uses z-x-y sequence
            Rz = [cos(flx) sin(flx) 0;-sin(flx) cos(flx) 0;0 0 1]; % these angles are all in radians
            Rx = [1 0 0;0 cos(lat) sin(lat);0 -sin(lat) cos(lat)]; % these angles are all in radians
            Ry = [cos(axl) 0 -sin(axl);0 1 0;sin(axl) 0 cos(axl)]; % these angles are all in radians
            Rs = Ry*Rx*Rz; % transforms from tibia to talus... rotation sequence doesn't matter
            ST_AX = Rt'*Rs'*st_ax; % transform subtalar axis from talus to tibia to Global
            
            % compute JCS angular velocities from q1_dot and q2_dot
            % dfx_dot = q1_dot ... put this in column 4 of jcs array
            jcs(k,4,j,i) = tc_veloc(k);
            % inv_dot = q2_dot*subax(x)/(cosd(int) + sind(int)) ... put this in column 5 of jcs array
            jcs(k,5,j,i) = st_veloc(k)*st_ax(1)/(cosd(int) + sind(int));
            % int_dot = q2_dot*subax(y) ... put this in column 6 of jcs array
            jcs(k,6,j,i) = st_veloc(k)*st_ax(2);
            
            % dot product is orthogonal projection of calcaneus moment onto talocrural axis = M_talocrural
            jcs(k,7,j,i) = dot([ankle_MX(k),ankle_MY(k),ankle_MZ(k)],TIB_Z);
            % dot product is orthogonal projection of calcaneus moment onto subtalar axis = M_subtalar
            jcs(k,8,j,i) = dot([ankle_MX(k),ankle_MY(k),ankle_MZ(k)],ST_AX);
            % M_JCS_dfx is identical to M_talocrural since talocrural axis is JCS Z-axis
            jcs(k,9,j,i) = jcs(k,7,j,i);
            % dot product is orthogonal projection of calcaneus moment onto JCS floating axis = M_JCS_inv
            jcs(k,10,j,i) = dot([ankle_MX(k),ankle_MY(k),ankle_MZ(k)],L);
            % dot product is orthogonal projection of calcaneus moment onto JCS y-axis = M_JCS_int
            jcs(k,11,j,i) = dot([ankle_MX(k),ankle_MY(k),ankle_MZ(k)],CAL_Y);
            
        end
        
        % extract the max values and their indices... 1=dfx, 2=inv, 3=int
        [jcs_mx(1,j,i),jcs_mx_i(1,j,i)] = minmax(jcs(:,1,j,i));
        [jcs_mx(2,j,i),jcs_mx_i(2,j,i)] = minmax(jcs(:,2,j,i));
        [jcs_mx(3,j,i),jcs_mx_i(3,j,i)] = minmax(jcs(:,3,j,i));
        % extract the max values and their indices... 4=dfx_dot, 5=inv_dot, 6=int_dot
        [jcs_mx(4,j,i),jcs_mx_i(4,j,i)] = minmax(jcs(:,4,j,i));
        [jcs_mx(5,j,i),jcs_mx_i(5,j,i)] = minmax(jcs(:,5,j,i));
        [jcs_mx(6,j,i),jcs_mx_i(6,j,i)] = minmax(jcs(:,6,j,i));
        % extract the max values and their indices... 7=M_talocrural, 8=M_subtalar
        [jcs_mx(7,j,i),jcs_mx_i(7,j,i)] = minmax(jcs(:,7,j,i));
        [jcs_mx(8,j,i),jcs_mx_i(8,j,i)] = minmax(jcs(:,8,j,i));
        % extract the max values and their indices... 9=M_JCS_dfx, 10=M_JCS_inv, 11=M_JCS_int
        [jcs_mx(9,j,i),jcs_mx_i(9,j,i)] = minmax(jcs(:,9,j,i));
        [jcs_mx(10,j,i),jcs_mx_i(10,j,i)] = minmax(jcs(:,10,j,i));
        [jcs_mx(11,j,i),jcs_mx_i(11,j,i)] = minmax(jcs(:,11,j,i));

        % keep the anatomical variables... already have whole vectors from the output file (j=trial, i=STUDY)
        % 1=q1, 2=q2, 3=q1_dot, 4=q2_dot
        atm(:,1,j,i) = tc_angle; % talocrural angle // OSim ankle joint axis
        atm(:,2,j,i) = st_angle; % subtalar angle // OSim subtalar joint axis
        atm(:,3,j,i) = tc_veloc; % talocrural joint angular velocity // q1_dot
        atm(:,4,j,i) = st_veloc; % subtalar joint angular velocity // q2_dot
        % extract the max values and their indices... 1=q1, 2=q2, 3=q1_dot, 4=q2_dot
        [atm_mx(1,j,i),atm_mx_i(1,j,i)] = minmax(atm(:,1,j,i));
        [atm_mx(2,j,i),atm_mx_i(2,j,i)] = minmax(atm(:,2,j,i));
        [atm_mx(3,j,i),atm_mx_i(3,j,i)] = minmax(atm(:,3,j,i));
        [atm_mx(4,j,i),atm_mx_i(4,j,i)] = minmax(atm(:,4,j,i));

    end
    
    % process joint angles and velocities down to scalar values of interest
    % rows: 1=dfx, 2=inv, 3=int, 4=dfx_dot, 5=inv_dot, 6=int_dot
    out(1,1,i) = mean(jcs_mx(1,:,i)); % outcome(dfx,mean,STUDY=i)
    out(1,2,i) =  std(jcs_mx(1,:,i)); % outcome(dfx,stdev,STUDY=i)
    out(2,1,i) = mean(jcs_mx(2,:,i)); % outcome(inv,mean,STUDY=i)
    out(2,2,i) =  std(jcs_mx(2,:,i)); % outcome(inv,stdev,STUDY=i)
    out(3,1,i) = mean(jcs_mx(3,:,i)); % outcome(int,mean,STUDY=i)
    out(3,2,i) =  std(jcs_mx(3,:,i)); % outcome(int,stdev,STUDY=i)
    out(4,1,i) = mean(jcs_mx(4,:,i)); % outcome(dfx_dot,mean,STUDY=i)
    out(4,2,i) =  std(jcs_mx(4,:,i)); % outcome(dfx_dot,stdev,STUDY=i)
    out(5,1,i) = mean(jcs_mx(5,:,i)); % outcome(inv_dot,mean,STUDY=i)
    out(5,2,i) =  std(jcs_mx(5,:,i)); % outcome(inv_dot,stdev,STUDY=i)
    out(6,1,i) = mean(jcs_mx(6,:,i)); % outcome(int_dot,mean,STUDY=i)
    out(6,2,i) =  std(jcs_mx(6,:,i)); % outcome(int_dot,stdev,STUDY=i)
    % 7=M_talocrural, 8=M_subtalar, 9=M_JCS_dfx, 10=M_JCS_inv, 11=M_JCS_int
    out(7,1,i) = mean(jcs_mx(7,:,i)); % outcome(M_talocrural,mean,STUDY=i)
    out(7,2,i) =  std(jcs_mx(7,:,i)); % outcome(M_talocrural,stdev,STUDY=i)
    out(8,1,i) = mean(jcs_mx(8,:,i)); % outcome(M_subtalar,mean,STUDY=i)
    out(8,2,i) =  std(jcs_mx(8,:,i)); % outcome(M_subtalar,stdev,STUDY=i)
    out(9,1,i) = mean(jcs_mx(9,:,i)); % outcome(M_JCS_dfx,mean,STUDY=i)
    out(9,2,i) =  std(jcs_mx(9,:,i)); % outcome(M_JCS_dfx,stdev,STUDY=i)
    out(10,1,i) = mean(jcs_mx(10,:,i)); % outcome(M_JCS_inv,mean,STUDY=i)
    out(10,2,i) =  std(jcs_mx(10,:,i)); % outcome(M_JCS_inv,stdev,STUDY=i)
    out(11,1,i) = mean(jcs_mx(11,:,i)); % outcome(M_JCS_int,mean,STUDY=i)
    out(11,2,i) =  std(jcs_mx(11,:,i)); % outcome(M_JCS_int,stdev,STUDY=i)
    % 12=q1, 13=q2, 14=q1_dot, 15=q2_dot
    out(12,1,i) = mean(atm_mx(1,:,i)); % outcome(q1,mean,STUDY=i)
    out(12,2,i) =  std(atm_mx(1,:,i)); % outcome(q1,stdev,STUDY=i)
    out(13,1,i) = mean(atm_mx(2,:,i)); % outcome(q2,mean,STUDY=i)
    out(13,2,i) =  std(atm_mx(2,:,i)); % outcome(q2,stdev,STUDY=i)
    out(14,1,i) = mean(atm_mx(3,:,i)); % outcome(q1_dot,mean,STUDY=i)
    out(14,2,i) =  std(atm_mx(3,:,i)); % outcome(q1_dot,stdev,STUDY=i)
    out(15,1,i) = mean(atm_mx(4,:,i)); % outcome(q1_dot,mean,STUDY=i)
    out(15,2,i) =  std(atm_mx(4,:,i)); % outcome(q1_dot,stdev,STUDY=i)
    
    %
    % plot the joint angles and velocities for visualization
    %
    figure;subplot(3,4,1);
    for j = 1:jmax
        % plot q1 angle
        plot(atm(:,1,j,i),'LineWidth',0.4);hold on;
        plot(atm_mx_i(1,j,i),atm_mx(1,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,5);
    for j = 1:jmax
        % plot q2 angle
        plot(atm(:,2,j,i),'LineWidth',0.4);hold on;
        plot(atm_mx_i(2,j,i),atm_mx(2,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,2);
    for j = 1:jmax
        % plot dorsiflexion angle
        plot(jcs(:,1,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(1,j,i),jcs_mx(1,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,6);
    for j = 1:jmax
        % plot inversion angle
        plot(jcs(:,2,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(2,j,i),jcs_mx(2,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,10);
    for j = 1:jmax
        % plot internal rotation angle
        plot(jcs(:,3,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(3,j,i),jcs_mx(3,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    %
    % plot the joint angular velocities for visualization
    %
    subplot(3,4,3);
    for j = 1:jmax
        % plot q1 angle
        plot(atm(:,3,j,i),'LineWidth',0.4);hold on;
        plot(atm_mx_i(3,j,i),atm_mx(3,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,7);
    for j = 1:jmax
        % plot q2 angle
        plot(atm(:,4,j,i),'LineWidth',0.4);hold on;
        plot(atm_mx_i(4,j,i),atm_mx(4,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,4);
    for j = 1:jmax
        % plot dorsiflexion angular velocity
        plot(jcs(:,4,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(4,j,i),jcs_mx(4,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,8);
    for j = 1:jmax
        % plot inversion angular velocity
        plot(jcs(:,5,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(5,j,i),jcs_mx(5,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,12);
    for j = 1:jmax
        % plot internal rotation angular velocity
        plot(jcs(:,6,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(6,j,i),jcs_mx(6,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,4,1);title({['STUDY: ',STUDY{i}];'ATM angles (top -> bot: q1, q2)'});ylabel('degrees');
    subplot(3,4,5);xlabel('Solution increment (0-150)');ylabel('degrees');
    subplot(3,4,2);title({['STUDY: ',STUDY{i}];'JCS angles (top -> bot: dfx, inv, int)'});ylabel('degrees');
    subplot(3,4,6);ylabel('degrees');
    subplot(3,4,10);xlabel('Solution increment (0-150)');ylabel('degrees');
    subplot(3,4,3);title({['STUDY: ',STUDY{i}];'ATM velocities (top -> bot: q1\_dot, q2\_dot)'});ylabel('degrees/sec');
    subplot(3,4,7);xlabel('Solution increment (0-150)');ylabel('degrees/sec');
    subplot(3,4,4);title({['STUDY: ',STUDY{i}];'JCS ang vel (top -> bot: dfx\_dot, inv\_dot, int\_dot)'});ylabel('degrees/sec');
    subplot(3,4,8);ylabel('degrees/sec');
    subplot(3,4,12);xlabel('Solution increment (0-150)');ylabel('degrees/sec');
    
    %
    % plot the anatomical moments for visualization
    %
    figure;subplot(3,2,1);
    for j = 1:jmax
        % plot talocrural anatomical moment
        plot(jcs(:,7,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(7,j,i),jcs_mx(7,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,2,3);
    for j = 1:jmax
        % plot subtalar anatomical moment
        plot(jcs(:,8,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(8,j,i),jcs_mx(8,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    %
    % plot the JCS moments for visualization
    %
    subplot(3,2,2);
    for j = 1:jmax
        % plot JCS dorsiflexion moment
        plot(jcs(:,9,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(9,j,i),jcs_mx(9,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,2,4);
    for j = 1:jmax
        % plot JCS inversion moment
        plot(jcs(:,10,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(10,j,i),jcs_mx(10,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,2,6);
    for j = 1:jmax
        % plot JCS internal moment
        plot(jcs(:,11,j,i),'LineWidth',0.4);hold on;
        plot(jcs_mx_i(11,j,i),jcs_mx(11,j,i),'.','MarkerSize',12,'MarkerFaceColor',[184/255 0 0]);
    end
    subplot(3,2,1);title({['STUDY: ',STUDY{i}];'Anatomical Moments (top -> bot: TC, ST)'});ylabel('N.m');
    subplot(3,2,3);xlabel('Solution increment (0-150)');ylabel('N.m');
    subplot(3,2,2);title({['STUDY: ',STUDY{i}];'JCS Moments (top -> bot: M\_dfx, M\_inv, M\_int)'});ylabel('N.m');
    subplot(3,2,4);ylabel('N.m');
    subplot(3,2,6);xlabel('Solution increment (0-150)');ylabel('N.m');

end
toc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% supporting functions
%
function [val,index] = minmax(v)
% the purpose of this function is to return the upper or lower bound of vector
% v subject to the constraint that a lower bound must not be the first data point
[hi,ihi] = max(v); [lo,ilo] = min(v);
if (abs(lo)>abs(hi) && ilo>1), val = lo; index = ilo;
else, val = hi; index = ihi; end
end

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
% input rotation matrix, output angles in radians
% v2 completed 13 July 2021

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