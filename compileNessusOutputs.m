

dir1 = 'C:\nessus\ankle-sprain\MCv31c';  % 31 = LEVEL GROUND

filelist = struct2table(dir([dir1 '\F\**\nessusOut2.csv']));

if ~exist('GRF','var')
for fi=1:size(filelist,1)
    TT = readtable([filelist.folder{fi} '\' filelist.name{fi}]);
    TT.time = seconds(str2double(strrep(TT.time,' sec','')));
    TT = table2timetable(TT);
    if fi==1
        GRF = TT.Fy;
    end
    GRF = [GRF, TT.Fy];
end
end
t = seconds(TT.time);

%% normalize
mass = 68.4400; %kg
weight = mass*9.81; %N
BW = GRF / weight ;

%% Plot
BWm=mean(BW,2);
BWs=std(BW,0,2);

BWupper=BWm+BWs;
BWlower=BWm-BWs;
BWlower(BWlower<0) = 0;

figure; hold on;
% hg1 = plot(t,BW,'-','color',[1 1 1]*0.8,'linewidth',2);
hsd = sdev_fill(t', BWupper', BWlower');
set(hsd,'facealpha',0.5,'facecolor',[1 1 1]*0.8,'linestyle','none')
hg2 = plot(t,BWm,'k-','linewidth',2,'displayname','Level: Monte Carlo, Mean');
% hg3 = plot(t,BWupper,'k:','linewidth',2);
% hg4 = plot(t,BWlower,'k:','linewidth',2);

xlim([.01 .1])
ylim([0 20])
 
simGRF=readmatrix('C:\Users\EACE-Precision-1\Google Drive\WORK\PROJECTS\AnkleSprains\opensim\demers\simulated landing on level ground\results\demers_simulatedGRF_digitized.csv');
simGRF(:,1) = simGRF(:,1)/1000;
hGRF2 = plot(simGRF(:,1), simGRF(:,2),'b-.','linewidth',2,'displayname','Level: (DeMers 2017) Simulated');

expGRF=readmatrix('C:\Users\EACE-Precision-1\Google Drive\WORK\PROJECTS\AnkleSprains\opensim\demers\singl-leg drop landing trials\demers_experimentalGRF_digitized.csv');
expGRF(:,1) = expGRF(:,1)/1000;
hGRF3 = plot(expGRF(:,1), expGRF(:,2),'r:','linewidth',2,'displayname','Level: (DeMers 2017) Measured');

legend([hg2,hGRF2,hGRF3])
set(gca,'fontsize',14,'fontweight','bold')
ylabel('Force (Bodyweights)')
xlabel('Time (sec)')


