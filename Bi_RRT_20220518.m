%%%%% parameters
clear all;close all;clc;
%%*************************************%%
addpath('C:\New project\Github'); % add the working path of your own
% ********* Original map generation parameters ********
L = 300; W =200;  % Length and width of the map
n = 9; radius = 20; % obstacle number and radius
step = 100;  % advancing step
Start = [20,20]; Goal = [L-20,W-20]; % starting and goal point
margin = [radius, radius]; % map margin for obstacle generation
% **********obstacle generation************
points(1,:) = margin + rand(1,2)*[L-2*radius,0;0,W-2*radius]; % first random point generation
for i = 2:n  % generate the following n random points
    generate_succ_flag = 0;  % set flag if generation succed
    collison_flag = 0; % collision check between existing points
    while generate_succ_flag == 0  % loop while not succeed
        temp_point = margin + rand(1,2)*[L-2*radius,0;0,W-2*radius]; % generate a random point
        collision_flag = 0; % set collision flag
        start_check = norm(temp_point-Start); % distance measurement
        goal_check = norm(temp_point-Goal); % distance measurement
        if start_check <= step + radius % distance out of the start point + step
            collision_flag = 1;  % collide with start
        else if goal_check <= step + radius % distance out of the start point + step
                collision_flag = 1;   % collide with goal
        else
            for j = 1:i-1 % collision check with the rest points
                dist_check = norm(temp_point-points(j,:)); % distance measurement
                if dist_check <= 2*radius % distance out of the radius
                    collision_flag = 1; % collide with other obstacles
                    break;
                end
            end
        end
        if collision_flag == 0  % if no collision
            generate_succ_flag = 1;  % new obstacle generation success flag
            points(i,:) = temp_point;  % creat a new obstacle point
        end
        end
    end
end
% figure(1); % begin to plot in figure 1
rectangle('Position',[0,0,L,W]);  % plot inside rectangle
hold on;
% plot(60,60,'bo');
for i = 1:n
    Fill_circle(points(i,1),points(i,2),radius,'k');  % plot all obstacles in black
end
t=0:0:0;
set(gca,'xtick',t); %这两句话可以去掉x轴的刻度和坐标值。
set(gca,'ytick',t); %这两句话可以去掉x轴的刻度和坐标值。
axis equal; % x and y axis equal to  each other
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%*************\\ get the screen image \\************
A=getframe(gcf);  % get the figure 1 screen as a map
imwrite(A.cdata,'C:\New project\Github\temp_map.png'); % save it as temporary map on computer
re_image = imread('C:\New project\Github\temp_map.png');
figure(2); % plot another map
imshow(re_image); % show the saved map as an image
hold on;
x_new_edge = 220;
y_new_edge =180;
Length_renew = 1305;
Width_renew = 860;
image_border = [x_new_edge,y_new_edge,Length_renew,Width_renew]; % create a border for the map
Veh_start = [300,960]; % start position approximately same as the original map by observation
Veh_goal = [1420,260]; % goal position
plot(300,960,'ro');  % plot the start position
plot(1420,260,'bo'); % plot the goal position
rectangle('Position',[x_new_edge,y_new_edge,Length_renew,Width_renew],"LineWidth",2,"EdgeColor",'k');  % plot inside rectangle
hold on;
plot_xy_point();% plot x,y,start and goal point in figure
%%*********************************************************************%%
%%*****************Initialize the Scenario*************************%%
map =im2bw(re_image); % input map read from a bmp file. for new maps write the file name here
hold on;
source=Veh_start; % source position in Y, X format
goal=Veh_goal; % goal position in Y, X format
stepsize=100;    % size of each step of the RRT
disTh=100;       % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 10000;
time_interval = 0.1; % pause time for ploting observation
pause_interval = 0.0077;% pause executing time
display=true;   % display of RRT
tic; % time counting
%%**************check if the start or goal point within map*****************%% 
if ~feasiblePoint(source,map), error('source lies on an obstacle or outside map'); end
if ~feasiblePoint(goal,map), error('goal lies on an obstacle or outside map'); end
%%**************display the map and range*****************%% 
% if display, imshow(map);rectangle('position',[1 1 size(map)-1],'edgecolor','k'); end
%%**************initialize two trees*****************%% 
RRTree1 = double([source -1]); % First RRT rooted at the source, representation node and parent index
RRTree2 = double([goal -1]);   % Second RRT rooted at the goal, representation node and parent index
counter=0;
%%**************assumption that the expansion is false *******************%%
tree1ExpansionFail = false; % sets to true if expansion after set number of attempts fails
tree2ExpansionFail = false; % sets to true if expansion after set number of attempts fails   false and false make true
%%*******************Mail loop**************************%%
while ~tree1ExpansionFail || ~tree2ExpansionFail  % loop to grow RRTs if true and expand successfully tree1 or tree2 succeed
    if ~tree1ExpansionFail % if tree 1 expand successfully
        %%********************** Expand tree 1 ******************%%
        [RRTree1,pathFound,tree1ExpansionFail] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map); % RRT 1 expands from source towards goal
        if ~tree1ExpansionFail && isempty(pathFound) && display % tree 1 extended, path not found, and display is true
            line([RRTree1(end,1);RRTree1(RRTree1(end,3),1)],[RRTree1(end,2);RRTree1(RRTree1(end,3),2)],'color','b'); % connection between extended node and its parental node
            counter=counter+1; % one more node added for each tree
            pause(time_interval);
%             M(counter)=getframe; % get the figure
        end
    end
    if ~tree2ExpansionFail
        %%********************** Expand tree 2 ******************%%
        [RRTree2,pathFound,tree2ExpansionFail] = rrtExtend(RRTree2,RRTree1,source,stepsize,maxFailedAttempts,disTh,map); % RRT 2 expands from goal towards source
        if ~isempty(pathFound), % if path found and pathFound returns the position of the newpoint and the number in both trees
            pathFound(3:4)=pathFound(4:-1:3);  % reverse the sequence of the newpoint in tree1 and tree2 if it is done on tree2,because extend tree2 makes it reverse
        end 
        if ~tree2ExpansionFail && isempty(pathFound) && display % tree 1 extended, path not found, and display is true
            line([RRTree2(end,1);RRTree2(RRTree2(end,3),1)],[RRTree2(end,2);RRTree2(RRTree2(end,3),2)],'color','r'); % connection between extended node and its parental node
            counter=counter+1; % one more node added for each tree
            pause(time_interval);
%             M(counter)=getframe; % get the figure
        end
    end
    if ~isempty(pathFound) % path found
         if display % as it is
            line([RRTree1(pathFound(1,3),1);pathFound(1,1);RRTree2(pathFound(1,4),1)],...
                [RRTree1(pathFound(1,3),2);pathFound(1,2);RRTree2(pathFound(1,4),2)],'color','green'); % connection between trees
            counter=counter+1;% one more node added for each tree
%             pause(time_interval);
%             M(counter)=getframe;% get the figure
         end
         %%********************** Path formulation ******************%%
        path=[pathFound(1,1:2)]; % add the last newpoint first
        prev=pathFound(1,3);     % add nodes from RRT 1 first
        while prev > 0 % if it is not -1
            path=[RRTree1(prev,1:2);path]; % the the previous node of newpoint in tree1, afterwards
            prev=RRTree1(prev,3); % get every previous node of tree1 through data chain
        end
        prev=pathFound(1,4); % % the the previous node of newpoint in tree2
        while prev > 0 % if it is not -1
            path=[path;RRTree2(prev,1:2)]; % get every previous node of tree2 through data chain
            prev=RRTree2(prev,3);% get every previous node of tree2 through data chain, forwards
        end
        break; % get the full path, and then quit
    end
end
%%**************************Out of the main loop***************************************%% 
time_cost = toc - time_interval*counter - pause_interval*counter; 
figure(3);
if size(pathFound,1)<=0, error('no path found. maximum attempts reached'); end % if there is no path
pathLength=0; % initialize the path length calculation
for i=1:length(path)-1, pathLength=pathLength+distanceCost(path(i,1:2),path(i+1,1:2)); end % calculating all path length
fprintf('processing time=%d \nPath Length=%d \n\n', time_cost,pathLength); % print the result
imshow(map); % show the image again
% rectangle('position',[1 1 size(map)-1],'edgecolor','k'); % plot the frame
line(path(:,1),path(:,2)); % plot the path
hold on;
plot(300,960,'ro');  % plot the start position
plot(1420,260,'bo'); % plot the goal position
plot_xy_point();
%%**************************Subfunction***************************************%% 
function [RRTree1,pathFound,extendFail] = rrtExtend(RRTree1,RRTree2,goal,stepsize,maxFailedAttempts,disTh,map)
pathFound=[]; %if path found, returns new node connecting the two trees, index of the nodes in the two trees connected
failedAttempts=0; % times of failure
x_new_edge = 220;
y_new_edge =180;
Length_renew = 1305;
Width_renew = 860;
while failedAttempts <= maxFailedAttempts % limited attempt times 10^4
    if rand < 1, %% set goal as destination
        sample = [x_new_edge,y_new_edge] + rand(1,2) .* [Length_renew,Width_renew]; % random sample
    else
        sample = goal;  % sample taken as goal to bias tree generation to goal
    end
%     plot(sample(1,1),sample(1,2),'r*');
    %%****************** Shortest distance node to the sample node among all tree nodes **********************************************%%
    [A, I] = min( distanceCost(RRTree1(:,1:2),sample) ,[],1); % A-distance, I-the number of shortest node. find the minimum value of each column
    closestNode = RRTree1(I(1),:); % get the closest node number. if the distances are equal, select the first one.
    %% moving from qnearest an incremental distance in the direction of rand
    theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % get the direction to extend sample to produce new node
    newPoint = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)])); % get the new point
%     plot(newPoint(1,1),newPoint(1,2),'bo');
    if ~checkPath(closestNode(1:2), newPoint, map) % if extension of closest node in tree to the new point is feasible
        failedAttempts = failedAttempts + 1; % if attempt fail, times plus +1
        continue;
    end
    %%************************Set the newpoint as sample point for the 2nd tree to grow**********************************%% 
    [A, I2] = min( distanceCost(RRTree2(:,1:2),newPoint) ,[],1); % find closest in the second tree
    if distanceCost(RRTree2(I2(1),1:2),newPoint) < disTh,        % if the node are close enough then the trees are connected
        pathFound=[newPoint I(1) I2(1)];  % 
        extendFail=false; % connected successfully
        break;
    end
    %%************************Check if new node is not already pre-existing in the tree**********************************%%
    %%************************Make sure that each node in the tree is at least one step apart**********************************%%
    [A, I3] = min( distanceCost(RRTree1(:,1:2),newPoint) ,[],1); % check if new node is not already pre-existing in the tree
    if distanceCost(newPoint,RRTree1(I3(1),1:2)) < disTh, % there exist a point within the tree, avoid the sample is near to the exisint point, and generate a nearby tree node.
        failedAttempts=failedAttempts+1;
        continue; 
    end
    RRTree1 = [RRTree1;newPoint I(1)]; % the position of node and parent node number
    extendFail=false;  % extended successfully
    break; % add node%% distanceCost.m
end
end
%%***********************Distant calculation***********************************%%
    function h=distanceCost(a,b)
        h = sqrt(sum((a-b).^2, 2));
    end
%% ***********************check if the path is feasible***********************************%%
    function feasible=checkPath(n,newPos,map)
        feasible=true;
        dir=atan2(newPos(1)-n(1),newPos(2)-n(2));
        for r=0:0.5:sqrt(sum((n-newPos).^2))
            posCheck=n+r.*[sin(dir) cos(dir)];
            if ~(feasiblePoint(ceil(posCheck),map) && feasiblePoint(floor(posCheck),map) && ...
                    feasiblePoint([ceil(posCheck(1)) floor(posCheck(2))],map) && feasiblePoint([floor(posCheck(1)) ceil(posCheck(2))],map))
            feasible=false;break;
            end
            if ~feasiblePoint(newPos,map), feasible=false; end
        end
    end
%%*********************** feasible if the Point feasible***********************************%%
    function feasible=feasiblePoint(point,map)
        feasible=true;
        % check if collission-free spot and inside maps
        if ~(point(1)>=1 && point(1)<=size(map,2) && point(2)>=1 && point(2)<=size(map,1) && map(point(2),point(1))==1)% chang map,2 to map,1
            feasible=false;
        end
    end
%%*****************  function of drawing a filled circle.*****************%%
function Fill_circle(x0,y0,r,color) % drawing a circle of obstacle.
aplha=0:pi/40:2*pi;  % 40 parts each circle
x = x0 + r*cos(aplha); % x coordinate of every part
y = y0 + r*sin(aplha); % y coordinate of every part
plot(x,y,'-'); % plot the segment
fill(x,y,color); % fill the color;
end
  %%***********************   
  function plot_xy_point()
text(200,130,'200','Color','k','FontSize',10);
text(300,130,'300','Color','k','FontSize',10);
text(400,130,'400','Color','k','FontSize',10);
text(500,130,'500','Color','k','FontSize',10);
text(600,130,'600','Color','k','FontSize',10);
text(700,130,'700','Color','k','FontSize',10);
text(800,130,'800','Color','k','FontSize',10);
text(900,130,'900','Color','k','FontSize',10);
text(1000,130,'1000','Color','k','FontSize',10);
text(1100,130,'1100','Color','k','FontSize',10);
text(1200,130,'1200','Color','k','FontSize',10);
text(1300,130,'1300','Color','k','FontSize',10);
text(1400,130,'1400','Color','k','FontSize',10);
text(1500,130,'1500','Color','k','FontSize',10);

text(100,200,'200','Color','k','FontSize',10);
text(100,300,'300','Color','k','FontSize',10);
text(100,400,'400','Color','k','FontSize',10);
text(100,500,'500','Color','k','FontSize',10);
text(100,600,'600','Color','k','FontSize',10);
text(100,700,'700','Color','k','FontSize',10);
text(100,800,'800','Color','k','FontSize',10);
text(100,900,'900','Color','k','FontSize',10);
text(100,1000,'1000','Color','k','FontSize',10);

text(310,980,'Start','Color','r','FontSize',10);
text(1430,280,'Goal','Color','b','FontSize',10);
end