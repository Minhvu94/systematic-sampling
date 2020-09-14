close all
% mat = [1 1 0 1;
%        0 0 1 0; 
%        1 1 0 1; 
%        1 0 0 0];  % Your sample matrix
mat = simpleMap;
[r, c] = size(mat);                          % Get the matrix size
imagesc((1:c), (1:r), mat);          % Plot the image

% mymap = [1 1 1;
%          0 0 0];
% 
% 
% colormap(mymap);                              % Use a gray colormap
% axis equal                                   % Make axes grid sizes equal
% set(gca, 'XTick', 1:(c+1), 'YTick', 1:(r+1), ...  % Change some axes properties
%          'XLim', [1 c+1], 'YLim', [1 r+1])%, ...
% %          'GridLineStyle', '-', 'XGrid', 'on', 'YGrid', 'on');

%%
close all
clear all
load exampleMaps.mat
map = binaryOccupancyMap(simpleMap,2);

prmSimple = mobileRobotPRM(map,10);
% prmSimple = mobileRobotPRM(map);

show(prmSimple)
