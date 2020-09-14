% % An example of rapidly-exploring random trees and path planning in 2-D
% % Ref: "Rapidly-Exploring Random Trees: A New Tool for Path Planning",
% % Steven M. LaValle, 1998
%~~~~
% Code can also be converted to function with input format
% [tree, path] = RRT(K, xMin, xMax, yMin, yMax, xInit, yInit, xGoal, yGoal, thresh)
% K is the number of iterations desired.
% xMin and xMax are the minimum and maximum values of x
% yMin and yMax are the minimum and maximum values of y
% xInit and yInit is the starting point of the algorithm
% xGoal and yGoal are the desired endpoints
% thresh is the allowed threshold distance between a random point the the
% goal point.
% Output is the tree structure containing X and Y vertices and the path
% found obtained from Init to Goal
%~~~~ 
% Written by: Omkar Halbe, Technical University of Munich, 31.10.2015
%~~~~ 
clear all; close all; clc
x = gallery('uniformdata',[1 10],0);
y = gallery('uniformdata',[1 10],1);
[vx,vy] = voronoi(x,y);

figure
hold on
voronoi(x,y);
% scatter(vx(:),vy(:),12,'r','filled')

axis equal


%%
% for mm = 1:100
clear all; close all;

K=2000;
xMin=0; xMax=3;
yMin=0; yMax=4;

xInit=1.5; yInit=2; %initial point for planner
xGoal=xMax; yGoal=yMax; %goal for planner
thresh=0.25;           %acceptable threshold for solution

constraints = [5 3.05 5  4;
               5  4  6  4;
               6  4  6  2;
               6  2  5  2;
               5  2  5 2.95];
           
constraints = constraints + [-4 -1 -4 -1];           

tree.vertex(1).x = xInit;
tree.vertex(1).y = yInit;
tree.vertex(1).xPrev = xInit;
tree.vertex(1).yPrev = yInit;
tree.vertex(1).indPrev = 0;
tree.vertex(1).TotalDens = 0;

figure(1); hold on; grid on;
line([constraints(:,1)';constraints(:,3)'],[constraints(:,2)';constraints(:,4)'],'Color','k');
plot(xInit, yInit, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
plot(xGoal, yGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
axis([0 3 0 4]);

xFree = xInit;
yFree = yInit;

for iter = 2:K
%     xRand = (xMax-xMin)*rand;
%     yRand = (yMax-yMin)*rand;
    % Randomize a new seed
    [xRand,yRand] = local_randomizer(xFree,yFree,0.5,1);
    plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
    drawnow;
%     pause(0.5);
    
    % Find the nearest node 
    dist = Inf*ones(1,length(tree.vertex));
    for j = 1:length(tree.vertex)
        dist(j) = sqrt( (xRand-tree.vertex(j).x)^2 + (yRand-tree.vertex(j).y)^2 );
    end
    [val, ind] = min(dist);
    
    xNearest = tree.vertex(ind).x;
    yNearest = tree.vertex(ind).y;
    tree.vertex(iter).xPrev = xNearest;
    tree.vertex(iter).yPrev = yNearest;
    tree.vertex(iter).indPrev = ind; 
    tree.vertex(iter).TotalDens = 0;

    % Check intersection with the constraint 
    out = lineSegmentIntersect(constraints,[xNearest,yNearest,xRand,yRand]);    
    intersect = find(out.intAdjacencyMatrix==1);
    
    if isempty(intersect)==1
        tree.vertex(iter).x = xRand; 
        tree.vertex(iter).y = yRand;
    else
        a = min(out.intNormalizedDistance2To1(intersect));
        if a>0.01
            a = a - 0.01;
            z = a*[xRand;yRand]+(1-a)*[xNearest;yNearest];
            tree.vertex(iter).x = z(1); 
            tree.vertex(iter).y = z(2); 
        else
            tree.vertex(iter).x = xNearest; 
            tree.vertex(iter).y = yNearest; 
        end
    end    
    
    plot([tree.vertex(iter).x; tree.vertex(ind).x],[tree.vertex(iter).y; tree.vertex(ind).y], 'r');
    delete(plot_rand)
%     if sqrt( (tree.vertex(iter).x-xGoal)^2 + (tree.vertex(iter).y-yGoal)^2 ) <= thresh
%         iter
%         break
%     end   
    if tree.vertex(iter).x < 0.99
%         iter
        break
    end
    
    % Update density and return the freest node 
    densMin = Inf;
    for k = 1:iter-1    
        dens_iter_k = density([tree.vertex(iter).x;tree.vertex(iter).y], [tree.vertex(k).x;tree.vertex(k).y]);
        tree.vertex(iter).TotalDens = tree.vertex(iter).TotalDens + dens_iter_k;
        tree.vertex(k).TotalDens = tree.vertex(k).TotalDens + dens_iter_k;
        if tree.vertex(k).TotalDens < densMin
            densMin = tree.vertex(k).TotalDens;
            indexFree = k;
        end
    end
    if tree.vertex(iter).TotalDens < densMin
        densMin = tree.vertex(iter).TotalDens;
        indexFree = iter;
    end
    
    xFree = tree.vertex(indexFree).x;
    yFree = tree.vertex(indexFree).y;
    
    pause(0);
end
iter
% end
CCC

if iter < K
    path.pos(1).x = xGoal; path.pos(1).y = yGoal;
    path.pos(2).x = tree.vertex(end).x; path.pos(2).y = tree.vertex(end).y;
    pathIndex = tree.vertex(end).indPrev;

    j=0;
    while 1
        path.pos(j+3).x = tree.vertex(pathIndex).x;
        path.pos(j+3).y = tree.vertex(pathIndex).y;
        pathIndex = tree.vertex(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end

    path.pos(end+1).x = xInit; path.pos(end).y = yInit;

    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    %     plot([tree.vertex(i).x; tree.vertex(ind).x],[tree.vertex(i).y; tree.vertex(ind).y], 'r');
    %     pause(0);
    end
else
    disp('No path found. Increase number of iterations and retry.');
end

%%
n=10000;
R=1;
x0 = 1;
y0 = 1;
local_randomizer(x0,y0,R,n)

function [x,y] = local_randomizer(x0,y0,R,n)
 t = 2*pi*rand(n,1);
 r = R*sqrt(rand(n,1));
 x = x0 + r.*cos(t);
 y = y0 + r.*sin(t);
%  scatter(x,y,15,'r','filled')
end

function density_value = density(x,y)
dist = norm(x-y);

if dist < 0.01
    density_value = 100;
else
    density_value = 1/dist;
end

end


