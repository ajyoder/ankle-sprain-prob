function [h_fill] = sdev_fill(xpoints,upper,lower,varargin)
%USAGE: [h_fill]=sdev_fill(xpoints,upper,lower,varargin)
%This function will fill a region with a color between the two vectors provided
%using the Matlab fill command.
%Any valid combination of Parameter-Value pairs can be entered for patch()

%% Form parameter-value string for use as patch settings
ParamValString=[];
for i=1:length(varargin)
   if isnumeric(varargin{i}); 
     if length(varargin{i})>1
     ParamValString=[ParamValString '[' num2str(varargin{i}) ']' ',']; 
     else
     ParamValString=[ParamValString num2str(varargin{i}) ','];    
     end
   else
     ParamValString=[ParamValString '''' varargin{i} '''' ',']; 
   end 
end
ParamValString=ParamValString(1:end-1);

%% if any of the parameters are not valid for patch, error will be thrown
if length(upper)==length(lower) && length(lower)==length(xpoints)
    msg='';
    filled=[upper,fliplr(lower)];
    xpoints=[xpoints,fliplr(xpoints)];

    h_fill=fill(xpoints,filled,'blue');%plot the data
    if ~isempty(ParamValString)
    eval(sprintf('set(h_fill,%s)',ParamValString));
    end
else
    error('Error: Must use the same number of points in each vector');
end
