%%%%%
%%%%%  This script will read in text files compiled from a NESSUS probabilistic analysis
%%%%%  (*.smx) = contains the input-response values, one row per trial
%%%%%  (*.zal) = sensitivity results (Z, U, CDF, columns of Sr, Su)
%%%%%
%%%%%  Outputs: 
%%%%%      Tcorr = table of input-response simple correlations, linear fits, p-values
%%%%%      Figure = CDF with requested prob levels and/or response levels annotated
%%%%%      Figure = trajectories matching those reported in NESSUS --> Visualize Results --> Probabilistic
%%%%                Sensitivity Factors, for input mean & variance, with a
%%%%                corresponding bar chart of factor mean(std) over a specified range

% close all
% clear all

addpath(genpath('.\common\'))

%%  Script inputs & options  
o1=[]; % response levels to extract (leave empty for none)
p1=[.05 .5 .95]; % prob levels to extract (leave empty for none)
trial=30; % study title for display
XLABEL = 'Subtalar Supination (Inv, Deg)'; %name of reponse being analyzed
PLOT_CORRELATION_SCATTER = 0;  % XY scatter plots corresponding to Tcorr results table
COMPUTE_SENSITIVITIES = 1;

%%%%%% Note: Manual variable definition required for (name, vu=mean, vs=sdev) 
%%%%%% No easy way to parse these from NESSUS .dat..... but they are needed
%%%%%% normalize prob sensitivity factors.  Comment out only 1 block

switch trial
    
    case 26    
%%%%%%% MCV26: (UN-BRACED) (incline, 0% co-activation)
fileZAL='.\results\mc26_incline_c0_b0_r0\ankleSprain.zal';
fileSMX='.\results\mc26_incline_c0_b0_r0\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','j1i','j2i','m1i','response'};
namesSMX={'DH','GC','GD','SA','AP','AI','MS'};
vu=[.30, 50002079, 5, 0.0000, -34, 0, 0.00]; %param, mean
vs=[.05, 5000205,  1, 0.0625,   5, 5, 0.05]; %param, std 

    case 27
%%%%%%% MCV27: (UN-BRACED) (incline, 10 REFLEX)
fileZAL='.\results\mc27_incline_c0_b0_r10\ankleSprain.zal';
fileSMX='.\results\mc27_incline_c0_b0_r10\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','rgi','j1i','j2i','m1i','response'};
namesSMX={'DH','GC','GD','SA','MR','AP','AI','MS'};
vu=[.30, 50002079, 5, 0.0000, 10, -34, 0, 0.00]; %param, mean
vs=[.05, 5000205,  1, 0.0625, 1,  5, 5, 0.05]; %param, std 

    case 28
%%%%%%% MCV28: (UN-BRACED) (incline, 60% co-activation)
fileZAL='.\results\mc28_incline_c60_b0_r0\ankleSprain.zal';
fileSMX='.\results\mc28_incline_c60_b0_r0\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','ci','j1i','j2i','m1i','response'};
namesSMX={'DH','GC','GD','SA','MC','AP','AI','MS'};
vu=[.30, 50002079, 5, 0.0000,  0.6, -34, 0, 0.00]; %param, mean
vs=[.05, 5000205,  1, 0.0625, 0.05,   5, 5, 0.05]; %param, std 

    case 29
%%%%%%% MCV29: (BRACED) (incline, 240% stiffness)
fileZAL='.\results\mc29_incline_c0_b240_r0\ankleSprain.zal';
fileSMX='.\results\mc29_incline_c0_b240_r0\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','j1i','j2i','m1i','bi','response'};
namesSMX={'DH','GC','GD','SA','AP','AI','MS','SB'};
vu=[.30, 50002079, 5, 0.0000, -34, 0, 0.00, 1.40]; %param, mean
vs=[.05, 5000205,  1, 0.0625,   5, 5, 0.05, 0.0625]; %param, std 

    case 30
%%%%%%% MCV30: (BRACED) (incline, brace + reflex + co-activation)
fileZAL='.\results\mc30_incline_c20_b150_r5\ankleSprain.zal';
fileSMX='.\results\mc30_incline_c20_b150_r5\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','ci','rgi','j1i','j2i','m1i','bi','response'};
namesSMX={'DH','GC','GD','SA','MC','MR','AP','AI','MS','SB'};
vu=[.30, 50002079, 5, 0.0000, 0.20, 5, -34, 0, 0.00, 0.50]; %param, mean
vs=[.05, 5000205,  1, 0.0625, 0.05, 1,   5, 5, 0.05, 0.0625]; %param, std 

    case 31
%%%%%%% MCV31: (UN-BRACED) (level, 0% co-activation)
fileZAL='.\results\mc31_level_c0_b0_r0\ankleSprain.zal';
fileSMX='.\results\mc31_level_c0_b0_r0\ankleSprain.smx';
varsSMX={'caseid','hi','ei','di','fi','j1i','j2i','m1i','response'};
namesSMX={'DH','GC','GD','SA','AP','AI','MS'};
vu=[.30, 50002079, 5, 0.0000, -34, 0, 0.00]; %param, mean
vs=[.05, 5000205,  1, 0.0625,   5, 5, 0.05]; %param, std 

end %SWITCH

%% Read in text files
varsZAL=varsSMX(2:(end-1));
dataSMX=readmatrix(fileSMX,'FileType','text');
trialid=(1:size(dataSMX(2:end,:),1))';
Tsmx=array2table( [trialid, dataSMX(2:end,1:(end-1))] ,'variablenames',varsSMX);
Tsmx=sortrows(Tsmx,size(Tsmx,2));
R=Tsmx{:,end}; %response
name = ['mc' num2str(trial)];

%% Correlations
Tcorr = {};
for vi=2:(size(Tsmx,2)-1)
x=Tsmx{:,vi}; %input 1
VAR = Tsmx.Properties.VariableNames{vi};

[c,cp]=corrcoef(x,R);
[pfit,sfit] = polyfit(x,R,1) ;
Rsquared=1-(sfit.normr/norm(R-mean(R)))^2; %R^2 fit term

Tcorr = [Tcorr; {VAR,Rsquared,c(1,2),cp(1,2)}];

if PLOT_CORRELATION_SCATTER
[yf,delta] = polyval(pfit,x,sfit);
figure('name',[name ' ' varsSMX{vi}]); hold on;
plot(x,R,'bo')
XLM=get(gca,'xlim');
h1=plot(x,yf,'k-','linewidth',2);
h2=plot(x,yf+2*delta,'m--',x,yf-2*delta,'m--');
xlabel(varsSMX{vi})
ylabel('response')
legend([h1,h2(1)],{'Linear Fit','95%PI'})
end

end

%Summarize in table for display or write out
Tcorr = cell2table(Tcorr,'VariableNames',{'Parameter','R_Squared','Corr','P_Value'});
disp(Tcorr)

%% Generate CDF
R2=sort(R);
CDF=(1:numel(R2))/numel(R2);

%%%%% Limit plot to only a specified probability range of interest
PLOW = 0;  PHIGH = 1;
[~,iLow]=min(abs(CDF-PLOW)); 
[~,iHigh]=min(abs(CDF-PHIGH)); 

hfig2=figure('name',name,'position',[639    34   560   420]); hold on;
plot(R2(iLow:iHigh),CDF(iLow:iHigh),'ko','displayname',name);
xlim([-10 55]);xticks(-10:10:50)
XLM=get(gca,'xlim');
xlim([XLM(1) XLM(2)])
ylim([0 1])
xlabel(XLABEL)
ylabel('Cumulative Probability')
set(gca,'fontsize',20,'fontweight','bold')

%-------PLOT/STORE RESPONSE-PROB LEVELS OF INTEREST 
for O = o1 
[~,i1]=min(abs(R2-O)); %index of nearest requested response level
iP = CDF(i1); %corresponding prob level
plot([O O],[0 iP],'b:')
plot([XLM(1) R2(i1)],[iP iP],'b:')
text(XLM(1),iP,sprintf('%.0f%%',100*iP),'VerticalAlignment','bottom','color','b')
end

%-------PLOT/STORE RESPONSE-PROB LEVELS OF INTEREST 
for P = p1 
[~,i2]=min(abs(CDF-P)); %index of nearest requested response level
iR = R2(i2); %corresponding prob level
plot([iR iR],[0 P],'r:')
plot([XLM(1) iR],[P P],'r:')
text(iR,0,sprintf('%.0f',iR),'VerticalAlignment','top','HorizontalAlignment','center','color','r')
end


if COMPUTE_SENSITIVITIES
%% Sensitivity factors
dataZAL=readmatrix(fileZAL,'FileType','text','range',[7 1]); %6 header rows
%%% (Z,u,p,<vars>,<du>,<ds>) thus with N variables, the sensitivity columns
%%% start in column (3+N+1) of size (m x 2*N) (m = # p levels)
N=numel(varsZAL); c1 = 4+N;
dataZAL = [dataZAL(:,1:3) dataZAL(:,c1:(c1+2*N-1))];
Tzal=array2table(dataZAL,'variablenames',[{'z','u','cdf'} strcat(varsZAL,'_du') strcat(varsZAL,'_dr')]);
%%% This modified P vector was required to get exact match between my #'s
%%% and those in NESSUS results dialog
Tzal.p=[Tzal.cdf(Tzal.cdf<0.5); flipud(Tzal.cdf(Tzal.cdf<=0.5))]; 

%%% Normalize sensitivity factors by (std / p)
for vi=1:N
    du = Tzal{:,[varsZAL{vi} '_du']};
    dr = Tzal{:,[varsZAL{vi} '_dr']};
    Tzal.([varsZAL{vi} '_su']) = du ./ Tzal.p * vs(vi);
    Tzal.([varsZAL{vi} '_sr']) = dr ./ Tzal.p * vs(vi);
end

%% Plot sensitivity factors
hfig3=figure('name',name,'position',[135         490        1599         420]); 
hax1=subplot(1,3,1); hold on;
hax2=subplot(1,3,2); hold on;
hax3=subplot(1,3,3); hold on;

z = Tzal.z;

s1 = 1;
s2 = size(Tzal,1);

bM = []; bS = [];
varsPLOT={'hi','ei','di','ci','rgi','m1i','j1i','j2i','fi','bi'};
namesPLOT={'DROP HEIGHT','GROUND CONTACT','GROUND DISSIPATION','MUSCLE COACTIVATION','MUSCLE REFLEX','MUSCLE STRENGTH','TALOCRURAL ANGLE (PF.)','SUBTALAR ANGLE (SUP.)','STIFFNESS - ANATOMY','STIFFNESS - BRACE'};
STYLE={'-','-.','--','-','-.','--','-','-.','--','-'};
COLOR=linspecer(numel(varsPLOT));
for vi=1:numel(varsPLOT)  
    if any(contains(Tzal.Properties.VariableNames,[varsPLOT{vi} '_su']))
    su = Tzal{:,[varsPLOT{vi} '_su']};
    sr = Tzal{:,[varsPLOT{vi} '_sr']};
    else
    su = nan(size(z));
    sr = nan(size(z));    
    end
    hp1(vi)=plot(hax1,z,su,'linestyle',STYLE{vi},'color',COLOR(vi,:),'tag',namesPLOT{vi},'linewidth',1,'displayname',namesPLOT{vi});
    hp2(vi)=plot(hax2,z,sr,'linestyle',STYLE{vi},'color',COLOR(vi,:),'tag',namesPLOT{vi},'linewidth',1,'displayname',namesPLOT{vi});
    suM=mean(su(s1:s2));
    srM=mean(sr(s1:s2));
    suS=std(su(s1:s2));   
    srS=std(sr(s1:s2));
    fprintf(1,'(%5s) Prob Sens. Factor: mean(su)=%.2f, mean(sr)=%.2f\n',varsPLOT{vi},suM,srM)
    bM = [bM; [suM,srM]];
    bS = [bS; [suS,srS]];
end

set([hax1;hax2],'xlim',[min(z) max(z)])
plot(hax1,[min(z) max(z)],[0 0],'k-')
plot(hax2,[min(z) max(z)],[0 0],'k-')
legend(hax1,hp1,namesPLOT,'location','best')

% Specified response or prob levels
for O = o1
YLM1=get(hax1,'ylim');
plot(hax1,[O O],[YLM1(1) YLM1(2)],'k:','handlevisibility','off')
YLM2=get(hax2,'ylim');
plot(hax2,[O O],[YLM2(1) YLM2(2)],'k:','handlevisibility','off')
end

% Bar plot
hbar = bar(hax3,bM);

ngroups = size(bM, 1);
nbars = size(bM, 2);
bw = min(0.8, nbars/(nbars + 1.5));
for i = 1:nbars
    xb = (1:ngroups) - bw/2 + (2*i-1) * bw / (2*nbars); %bar group centers
    heb = errorbar(hax3, xb', bM(:,i), bS(:,i), 'k','linestyle','none','capsize',0,'linewidth',2);
end

set([hax1,hax2,hax3],'Fontsize',12,'fontweight','bold')
ylabel(hax1,name)
title(hax1,'Sensitivity to Input Mean')
title(hax2,'Sensitivity to Input Variance')
legend(hbar,{'S_{\mu}','S_{\sigma}'})
% set([hax1,hax2],'ylim',[-4.5 4.5])
xlabel(hax1,XLABEL)
xlabel(hax2,XLABEL)

set(hax3,'xlim',[0 numel(varsPLOT)+1],'xtick',1:numel(varsPLOT),'xticklabel',namesPLOT,'XTickLabelRotation',30)
title(hax3,'Probabilistic Sensitivity (Mean, 1SD)')
legend(hbar,{'S_{\mu}','S_{\sigma}'})

end

