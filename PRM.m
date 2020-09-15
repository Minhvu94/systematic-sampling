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
clear all; close all; clc

K=2000;
xMin=0; xMax=10;
yMin=0; yMax=10;

e = 0.5;
constraints = [3      0      3       7;
               3     7+e     3       10;
               7      0      7      3-e;
               7      3      7       10;
               3      7    5-e/2      7;
             5-e/2    7    5-e/2    3-e;
             5-e/2   3-e     7      3-e;
               7      3    5+e/2      3;
             5+e/2    3    5+e/2    7+e;
             5+e/2   7+e     3      7+e];      

polyg1 = [3       0;
          3       7;
        5-e/2     7;
        5-e/2    3-e;
          7      3-e;
          7       0];
      
polyg2 = [3       10;
          3      7+e;
        5+e/2    7+e;
        5+e/2      3;
          7        3;
          7       10];  

figure(1); hold on; grid on;
line([constraints(:,1)';constraints(:,3)'],[constraints(:,2)';constraints(:,4)'],'Color','k');
axis([0 xMax 0 yMax]);

graph.node = [];
total_nodes = 0;

for iter = 1:1000
    % Randomize a new seed
    [xRand,yRand,n] = feasible_randomizer(polyg1,polyg2);
    total_nodes = total_nodes + n;
    plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');
    
    % Find the neighborhood of the random node 
    dist = Inf*ones(1,length(graph.node));
    for k = 1:length(graph.node)
        dist(k) = sqrt( (xRand-graph.node(k).x)^2 + (yRand-graph.node(k).y)^2 );
    end
    ind = find(dist<0.6);
    
    % Add node    
    graph.node(iter).x = xRand;
    graph.node(iter).y = yRand;
    graph.node(iter).edge = [];
    
    % Connect to the neighborhood nodes
    for j = 1:length(ind)
        % Check intersection with the constraint 
%         out = lineSegmentIntersect(constraints,[xNearest,yNearest,xRand,yRand]);   
        out = lineSegmentIntersect(constraints,[graph.node(ind(j)).x, graph.node(ind(j)).y, xRand, yRand]);
        intersect = find(out.intAdjacencyMatrix==1);
        
        if isempty(intersect)==1
            graph.node(ind(j)).edge = [graph.node(ind(j)).edge, iter]; 
            graph.node(iter).edge = [graph.node(iter).edge, ind(j)];
            plot([graph.node(ind(j)).x, xRand], [graph.node(ind(j)).y, yRand], 'r');            
        end
    end
    if total_nodes>=1000
        iter
        break
    end
end

%%
e = 0.5;
constraints = [3      0      3       7;
               3     7+e     3       10;
               7      0      7      3-e;
               7      3      7       10;
               3      7    5-e/2      7;
             5-e/2    7    5-e/2    3-e;
             5-e/2   3-e     7      3-e;
               7      3    5+e/2      3;
             5+e/2    3    5+e/2    7+e;
             5+e/2   7+e     3      7+e];
clc
figure(1); clf; hold on; grid on;
line([constraints(:,1)';constraints(:,3)'],[constraints(:,2)';constraints(:,4)'],'Color','k');
axis([0 10 0 10]);

polyg1 = [3       0;
          3       7;
        5-e/2     7;
        5-e/2    3-e;
          7      3-e;
          7       0];
      
polyg2 = [3       10;
          3      7+e;
        5+e/2    7+e;
        5+e/2      3;
          7        3;
          7       10];  
%       plot(polyg1(:,1),polyg1(:,2),'r')
%       plot(polyg2(:,1),polyg2(:,2),'r')
[xRand,yRand,~] = feasible_randomizer(polyg1,polyg2);
plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');


function [xRand,yRand,iter] = feasible_randomizer(polyg1,polyg2)

for iter=1:50

    xRand = (10-0)*rand;
    yRand = (10-0)*rand;
%     delete(plot_rand)
%     plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');
    in1 = inpolygon(xRand,yRand,polyg1(:,1),polyg1(:,2));
    in2 = inpolygon(xRand,yRand,polyg2(:,1),polyg2(:,2));
    if in1 == 0 && in2 == 0
        break
    end
end
end
