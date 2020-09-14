clear all
close all
clc

% True if you want to display the results.
% WARNING:If the number of line segments is high, display might take too long.
is_show = true;

% SPEED TEST

% Randomly generate line segments.
n_line = 2;

% XY1 = rand(n_line,4);
% XY2 = 10*rand(n_line,4);

XY1 = [0  0  0  2;
       0  2  1  2;
       1  2  1 1.2;
       1 0.8 1  0;
       1  0  0  0];
   
x = [0.5; 1];
y = [-1.2; 1];
XY2 = [x' y'];

% a = 0.7143;
% z = a*y+(1-a)*x;
% scatter(z(1),z(2),15,'g','filled')
%% SPEED TEST METHOD 1.
tic
out = lineSegmentIntersect(XY1,XY2);
dt_1 = toc;

fprintf(1,'Method 1 took %.2f seconds for %.0f line segments...\n',dt_1,n_line);


%% PREPARE THE FIGURE.
if is_show
    figure
    hold on 
    line([XY1(:,1)';XY1(:,3)'],[XY1(:,2)';XY1(:,4)'],'Color','r');
    line([XY2(:,1)';XY2(:,3)'],[XY2(:,2)';XY2(:,4)'],'Color','k');
    
    scatter(out.intMatrixX(:),out.intMatrixY(:),[],'r');
    title('Intersection Points'); 
end


index = find(out.intAdjacencyMatrix==1);
a = out.intNormalizedDistance2To1(index);
z = a*y+(1-a)*x;
scatter(z(1),z(2),15,'g','filled')






