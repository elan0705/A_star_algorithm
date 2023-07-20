clc;
clear all;
close all;

% initializing randomness for robot location and goal point
s = rng;

%maze binary map
maze =  [0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 1 1 1 1 1 1 0 0 0;
         0 0 0 1 1 1 1 1 1 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;
         0 0 0 0 0 0 0 0 0 0 0 0;];
% finding the occupied and unoccupied region     
open = find(maze==0);
open_list = indexing(maze,open);
closed = find(maze==1);
closed_list = indexing(maze,closed);

%initializing robot location
robo_x = randi(6);
robo_y = randi(3);
robot_loc = [robo_x,robo_y]; %[4,3];

%initializing goal location
goal_x = randi(6);
g = [10,12]
goal_y = randi(g)
goal_loc = [goal_x,goal_y]%[6,12];

%Validating if the goal & Robot loc is unoccupied
qg = 0;
map(:) = robot_loc
if any(open_list==robot_loc)
    if any(open_list==goal_loc)
%loop to find the path   
        while qg~=length(maze)
            s = successor(open_list,robot_loc);
            new_loc = weight(s,robot_loc,goal_loc,qg);
            qg = qg+1;
            closed_list(end+1,:) = robot_loc;
            robot_loc = new_loc;
            map(end+1,:) = robot_loc;
        end
        
        
        
    end
end

%index obtainer function
function a  = indexing(matrix,index_matrix)
    for i = 1:length(index_matrix)
        [x(i),y(i)] = ind2sub(size(matrix),index_matrix(i));
        a(i,:) = [x(i),y(i)];
    end
end

%Successor locator function
function n = successor(open_space, robot_current_loc)
    s1 = [robot_current_loc(1)-1, robot_current_loc(2)];
    s2 = [robot_current_loc(1)+1, robot_current_loc(2)];
    s3 = [robot_current_loc(1), robot_current_loc(2)-1];
    s4 = [robot_current_loc(1), robot_current_loc(2)+1];
    s5 = [robot_current_loc(1)-1, robot_current_loc(2)-1];
    s6 = [robot_current_loc(1)-1, robot_current_loc(2)+1];
    s7 = [robot_current_loc(1)+1, robot_current_loc(2)-1];
    s8 = [robot_current_loc(1)+1, robot_current_loc(2)+1];
    ns(:,:) = [[s1];[s2];[s3];[s4];[s5];[s6];[s7];[s8]];
    %disp(s(1,:))
    for j = 1:length(ns)
        if any(open_space==ns(j,:))
            n(j,:) = [ns(j,:)]
        end
    end
end

% sucessor weights calculator and location updator
function pos = weight(successor_array,robot_location,goal_location,qg)
    for k = 1:length(successor_array)
        sg(k) = qg + norm(successor_array(k,:)-robot_location);
        sh(k) = sqrt(((successor_array(k,1))-robot_location(1)).^2 + ((successor_array(k,2)-robot_location(2)).^2));
        sf(k) = sg(k)+sh(k);
    end
    mini = min(sf);
    indi(:) = find(sf,mini,'first');
    %disp(indi);
    posi = [indi];
    for p = 1:length(posi)
        %disp('abb')
        %disp(successor_array(indi(p),:))
        short_d(p) = norm(goal_location - [successor_array(indi(p),:)]);
    end
    [mini_short,short_idx] = min(short_d);
    %ind_short = find(short_d,mini_short);
    %disp(short_idx)
    pos = successor_array(indi(short_idx),:);
    
end
