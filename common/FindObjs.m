function [ h_show,h_hide ] = FindObjs( h, st, types, varargin)
%FINDLINES  Searches across all children of object 'h' for Tags matching 
%   passed format string. Makes found objects visible and all others 
%   hidden. 
% 
%  inputs: 
%     h=handle of object to act on (figure, axes, panel)
%     st=string (or cell array of strings) to match using regular expression notation
%     (0,1) = 1 to hide the 
%
%  NOTE: pass st='\S*' to reset figure, showing all lines that don't have empty tags
%
%  EXAMPLE: [h1,h2]=FindLines(gcf,'ankle')

axis_reset = 1;

h_show=[]; h_hide=[]; h_found_all=[];

if isempty(types); types ={'line'}; end

if all(strcmpi(get(h,'type'),'axes'))
YLIM = get(h,'ylim');
end

if ~iscell(types); types={types};end
for ti=1:numel(types)
     h_found2=findall(h,'type',types{ti},{'-not','tag',''},'-regexp',{'Tag','\S*'});
     h_found_all=[h_found_all;h_found2];      
end

if ~iscell(st); st={st};end
for sti=1:numel(st)
    for ti=1:numel(types)
        h_found1=findall(h,'type',types{ti},'-regexp',{'Tag',['\S*' st{sti} '\S*']});
        h_show=[h_show;h_found1];
        
        % h_hide=[h_hide;h_found2];
    end
end

h_hide=setdiff(h_found_all,h_show);

set(h_show,'Visible','on');
if ~isempty(varargin) && strcmpi(varargin{1},'show')
else
set(h_hide,'Visible','off'); %DEFAULT hide anything not found
end
if ~axis_reset && all(strcmpi(get(h,'type'),'axes'))
    set(h,'ylim',YLIM) %keep original   
end



