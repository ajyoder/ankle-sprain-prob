function [ file_data ] = ReadOSIMtxt( filename )
%READOSIMTXT Summary of this function goes here
%   Detailed explanation goes here

if ~exist(filename,'file')
  file_data=[];
  warning(['File not found: ' filename]);
  return;
end

FID=fopen(filename);

LN_cnt=1;
LN='';
HEADER=struct();
while ~feof(FID) && ~strcmpi(LN,'endheader')
LN=strtrim( fgetl(FID) ); %*AY: added 9FEB18 to account for whitespace from Excel editing

if LN==-1
    file_data=[];
    return
end

% HEADER.(['LN' num2str(LN_cnt)])=LN;
HEADER(LN_cnt).str=LN;
LN_cnt=LN_cnt+1;

end

%* Reached last header line, next LN is variable names
LN=fgetl(FID); 
LN_cnt=LN_cnt+1;
LN_scan=textscan(LN,'%s','delimiter','\t');
ColNames=LN_scan{1};
ColNames=strrep(ColNames,'.','_');

%*Ensure no identical names in ColNames for transform to struct. If so, append _#
for st=ColNames'
    dupl=strcmpi(ColNames,st{:});
    idx=find(dupl);
    if numel(idx)>1
       for i=2:numel(idx)
         ColNames{idx(i)}=[ColNames{idx(i)} '_' num2str(i)];  
       end
    end     
end

DATA={};
while ~feof(FID)

LN=fgetl(FID); 
LN_cnt=LN_cnt+1;    
LN_scan=textscan(LN,'%f','delimiter','\t');
    
DATA=[DATA ;LN_scan{1}'];
end

DATA=cell2mat(DATA);
DATA=mat2cell(DATA,size(DATA,1),ones(1,size(DATA,2)));
DATA=cell2struct(DATA,ColNames',2);

DATA_table=struct2table(DATA);

%**(AY) 6-18-18: Bug found - duplicate row entries (for small timesteps?)
%  FIX: trim last row if equivalent to end-1 (can check all if needed?)
dec_place=5; %depends on variable... but for most part unless radians
if all( round(DATA_table{end-1,:},5)==round(DATA_table{end,:},dec_place) )
    DATA_table=DATA_table(1:end-1,:);
end

file_data.header=HEADER;
file_data.data=DATA_table;

fclose(FID);
end