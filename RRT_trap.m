%~~~~ 
% The original RRT in The trap example --> rapid exploration is blocked by the constraint 
% --> waisting a lot of samples 
%~~~~ 
%% original-RRT loop:
for mm = 1:100

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

xArray=xInit; yArray = yInit;

figure(1); hold on; grid on;
line([constraints(:,1)';constraints(:,3)'],[constraints(:,2)';constraints(:,4)'],'Color','k');
plot(xInit, yInit, 'ko', 'MarkerSize',10, 'MarkerFaceColor','k');
plot(xGoal, yGoal, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
axis([0 3 0 4]);

for iter = 2:K
    xRand = (xMax-xMin)*rand;
    yRand = (yMax-yMin)*rand;
    plot_rand = plot(xRand, yRand, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
    drawnow;
%     pause(0.5);
    
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
    
    pause(0);
end
iter
end
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




