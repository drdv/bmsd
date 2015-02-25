function setup_bMSd(case_flag)
%
% ------------------------------------------------------
% | Basic Multibody Simulator Derived (Matlab toolbox) |
% ------------------------------------------------------
%
% setup_brd
%
% Add (remove) bMSd Toolbox to (from) the Matlab path
%
% Syntax:
% -------
% setup_brd                 % case_flag = 1 is assumed
% setup_brd(case_flag)
%
% Input:
% ------
% case_flag   -  if case_flag == 1 add bMSd to the Matlab path
%                if case_flag ~= 1 remove bMSd from the Matlab path
% 

% Note:
% -------
% Under LINUX the savepath command issues an error (when the user permissions are not sufficient)
% chmod: changing permissions of `/usr/local/Matlab2007a/toolbox/local/pathdef.m': Operation not permitted
% 
% One can change the permissions of pathdef.m by doing: 
% cd path_to_Matlab/toolbox/local/
% sudo chmod 777 pathdef.m

% User input (add directories)
% ------------------------
d{1} = 'rotation';
d{2} = 'kinematics';
d{3} = 'dynamics';
d{4} = 'general_purpose';
d{5} = 'models';
d{6} = 'solvers';
%d{7} = 'examples';
% ------------------------

if nargin == 0
    case_flag = 1;
end

if isunix
  slash_str = '/';
elseif ispc
  slash_str = '\';
else
  disp('ERROR: Unkonwn computer type')
  return
end

cdir = pwd;

if case_flag == 1 % add to the path
  for i=1:length(d)
    path([cdir slash_str d{i}], path);
  end
else % remove from the path
  for i=1:length(d)
    rmpath([cdir slash_str d{i}])
  end
end

% save the path
s = savepath;
if s
  disp('ERROR: the path could not be saved')
end

%%%EOF