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
count = 0;

for current = 1:1000
    % Randomize a new seed
    [xRand,yRand,n] = feasible_randomizer(polyg1,polyg2);
    total_nodes = total_nodes + n;
    count = count + n;
    plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',3, 'MarkerFaceColor','r');
    
    % Find the neighborhood of the random node 
    dist = Inf*ones(1,length(graph.node));
    for k = 1:length(graph.node)
        dist(k) = sqrt( (xRand-graph.node(k).x)^2 + (yRand-graph.node(k).y)^2 );
    end
    ind = find(dist<1);
    
    % Add node    
    graph.node(current).x = xRand;
    graph.node(current).y = yRand;
    graph.node(current).edge = [];
    
    % Connect to the neighborhood nodes 
    for i = ind        
        [intersect,n] = connect([graph.node(i).x; graph.node(i).y], [xRand; yRand], polyg1, polyg2); 
        count = count + n;
        
        % Connect if no constraint intersection 
        if intersect==0
            graph.node(i).edge = [graph.node(i).edge, current]; 
            graph.node(current).edge = [graph.node(current).edge, i];
            plot([graph.node(i).x, xRand], [graph.node(i).y, yRand], 'r');            
        end
        
    end
    
%     if total_nodes>=1000
%         current
%         break
%     end
    
end
count
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

%%
x = [1;2];
y = [2;2];
% epsi = 0.3;
% pts = pts_between(x,y,ceil(norm(x-y)/epsi)-1);
% 
% scatter(pts(:,1),pts(:,2),10,'r','filled')
[intersect,count] = connect(x,y,polyg1,polyg2)
% v = (y-x);
% v = v/norm(v);
hold on
plot(polyg1(:,1),polyg1(:,2),'r')
plot(polyg2(:,1),polyg2(:,2),'r')
axis([0 10 0 10]);


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

function pts = pts_between(x,y,n_inner_points)
xvals = linspace(x(1), y(1), n_inner_points+2);
yvals = linspace(x(2), y(2), n_inner_points+2);
pts = [xvals(:), yvals(:)];
end

function [intersect,count] = connect(x,y,polyg1,polyg2)
epsi = 0.1;
pts = pts_between(x,y,ceil(norm(x-y)/epsi)-1);
pts = pts(2:end-1,:);

intersect = 0;
count = 0;

if isempty(pts) == 0
    
    for j = 1:size(pts,1)
        in1 = inpolygon(pts(j,1),pts(j,2),polyg1(:,1),polyg1(:,2));
        in2 = inpolygon(pts(j,1),pts(j,2),polyg2(:,1),polyg2(:,2));
        if in1==1 || in2==1
            intersect = 1;
            break
        end
    end
    count = j;

end


end









