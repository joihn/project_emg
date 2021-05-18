%  Copyright (C) 2017 EPFL-LSRO (Laboratoire de Systemes Robotiques).
% 
%  Licensed under the Apache License, Version 2.0 (the "License");
%  you may not use this file except in compliance with the License.
%  You may obtain a copy of the License at
% 
%       http://www.apache.org/licenses/LICENSE-2.0
% 
%  Unless required by applicable law or agreed to in writing, software
%  distributed under the License is distributed on an "AS IS" BASIS,
%  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%  See the License for the specific language governing permissions and
%  limitations under the License.

function variables = hri_load_logfile(filename)
%HRI_LOAD_LOGFILE Loads a HRI logfile to a MATLAB structure.
%   Loads the filename with the given filename. A MATLAB structure is
%   generated, each named field is an array of the history of the variable
%   value.

%% Read the header to get the variables names.
fid = fopen(filename, 'r');

line = fgetl(fid);

varNames = textscan(line, '%s', 'Delimiter', ';');
varNames = varNames{:};

%%

% Replace the special characters by '_'.
for i = 1:length(varNames)
    varName = varNames{i};
    varName((varName < 'a' | varName > 'z') & ...
            (varName < 'A' | varName > 'Z') & ...
            (varName < '0' | varName > '9')) = '_';
    varNames{i} = varName;
end

fclose(fid);

%% Read the variables values over time.
A = dlmread(filename, ';', 1, 0);

%% Create a structure with a field per variable.
for i=1:length(varNames)
    variables.(varNames{i}) = A(:,i);
end

end