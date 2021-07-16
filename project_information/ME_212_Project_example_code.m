% An optional code framework for you to use to complete the ME 212 project. 
% Feel free to write your own code from scratch or use this code. This code
% is not functional and is missing many components that your group will
% need to add, but will provide a basic structure of how you can solve
% this project.
% Update - Meaning: The variable was set to zero only to remove warnings in code. 
%
% Read Initial Ball Positions, ball 11 is cue ball
A = readtable('T.xlsx'); Bp = table2array(A(:,2:3)); 
Bp_final = cell(100,1); % Initialize a cell variable to store the ball positions
                        % at each step throughout the simulation.
Bp_final{1} = Bp; % Set the first cell in Bp_final to the initial ball positions 

% Set cue ball initial velocity
vb = zeros(11,2);
vb(11,1) = -4*39.3701;

% Define Problem Constants %
a = 88; b = 44;
mc = 200; mn = 160;
rc = 2.25; rn = 2.25;
ecn = 0.95; enn = 0.9; ebc = 0.7;
mubb = 0.05; mubs = 0.1; mubc = 0.2;
g = 9.81*39.3701;
ac = g*mubs;

m = zeros(11,1); % Ball moving indicator.
m(11) = 1; % Set cue ball to be moving initially. 

% Simple plot with velocity vectors
%{
figure % Open a figure window
set(gcf, 'Position',  [100, 100, 800, 400]) % Set the initial position and size of the window
hold on % Allows multiple things to be plotted on the same figure without overwriting. 
for i = 1:11 % Loop through the 11 balls
   scatter(Bp(i,1),Bp(i,2),550,'filled','MarkerEdgeColor','k','LineWidth',0.5) % Plot position of ball i 
   if m(i) % If ball i is moving
            plot([Bp(i,1),vb(i,1)/4+Bp(i,1)],[Bp(i,2),vb(i,2)/4+Bp(i,2)],'k') % Plot velocity of ball i
   end
end
xlim([0 a]) % Set x limits of plot
ylim([0 b]) % Set y limits of plot
colororder([0.9290 0.6940 0.1250; 0 0.4470 0.7410;1 0 0;0.4940 0.1840 0.5560;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880;0.6350 0.0780 0.1840;0 0 0;0.3010 0.7450 0.9330;0.5 0.5 0.5;1 1 1])
%}

% More detailed plot with ball numbers
%{
figure
set(gcf, 'Position',  [100, 100, 800, 400])
box on
xlim([0 a])
ylim([0 b])
for i = 1:11
    hold on
    scatter(Bp(i,1),Bp(i,2),550,'filled','MarkerEdgeColor','k','LineWidth',0.5)
    if i == 8
        text(Bp(i,1),Bp(i,2),num2str(i),'HorizontalAlignment','center','Color',[1 1 1]);
    elseif i == 11
        text(Bp(i,1),Bp(i,2),'C','HorizontalAlignment','center');
    else
        text(Bp(i,1),Bp(i,2),num2str(i),'HorizontalAlignment','center','Color',[1 1 1]);
    end
    colororder([0.9290 0.6940 0.1250; 0 0.4470 0.7410;1 0 0;0.4940 0.1840 0.5560;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880;0.6350 0.0780 0.1840;0 0 0;0.3010 0.7450 0.9330;0.5 0.5 0.5;1 1 1])
end
%}

n = 2; % Set simulation step counter at 2 (Initial ball position is 1)

while sum(m) ~= 0 % While any ball is moving

    dt = zeros(11,1); % Reset dt before entering for loop
    c = cell(11,1); % Set initial collision marker for each ball
    te = zeros(11,1); % Time to next event (Collision with wall, ball, or stopping)
    theta = zeros(11,1); % Angle ball is travelling, in rad
    
    for i = 1:11 % Determine what collision each ball will undergo + find time to next event for each ball. 
        
       if m(i) % If ball is moving
           
           dt(i) = 0; % Update. Find total distance ball can travel at current speed
           theta(i) = atan2(vb(i,2),vb(i,1)); % Find angle ball is travelling at
           
           % Check what wall the moving ball would hit, set collision marker to
           % specific wall. Be mindful of glancing collisions
           if 1 < 2 && 3 <= 4 || 5 == 6  % Left wall
               c{i} = 'L';
           elseif 1 < 2 % Top wall
               c{i} = 'T';
           elseif 3 < 4 % Right wall
               c{i} = 'R';
           else % Bottom wall
               c{i} = 'B'; 
           end
       
           dc = 0; % Update. Find distance to collision with wall.
           
           if dt(i) > dc % If distance to wall collision is less than total distance
                         % ball can travel, update dt to be the distance to the collision
               dt(i) = dc;
               te(i) = 0; % Update. Time to next event for ball i (collides with wall) 
           else % Ball will stop moving before hitting wall
               c{i} = 'S'; % Set collision marker to S for stop.
               te(i) = 0; % Update. Time to next event for ball i (ball stops)
           end
                  
           for j = 1:11 % Check for stationary ball collision
          
               if j ~= i && m(j) ~= 1 
                   
                   for L = 1:20
                  
                       xL = Bp(i,1)+cos(theta(i))*dt(i)*L/20; % project ball i forward in the x direction to xL.
                       yL = tan(theta(i))*(xL-Bp(i,1))+Bp(i,2); % calculate yL with xL. 
                       dL = ((xL-Bp(j,1))^2+(yL-Bp(j,2))^2)^0.5; % Distance between ball i and j
                       
                       if dL < 2*rc 
                           
                           syms x % or y, or t 
                           eqn = ((x-Bp(j,1))^2+(y-Bp(j,2))^2)^0.5 == 2*rc; % Update. Find when distance between i and j = 2*rc. 
                           S = solve(eqn,x);
                           x = double(S);
       
                           if 1 == 1 % Update. If collision does occur
                               
                               c{i} = j;
                               te(i) = 0; % Update. 
                               break % Breaks out of the "for L = 1:20" loop.
                               
                           end
                          
                       end
                   
                   end
               
               end
           
           end
          
           for j = 1:11 % Check for moving ball collision
            
               
               
           end
         
       end
    
    end
    
    t = 100; % Set t as high number which will be overwritten later on. 
    
    for i = 1:11 % Find the minimum time value of te for elements greater than 0.
                 % Set k as the ball that will undergo the next event
        if te(i) < t && te(i) ~= 0
            
            t = te(i); k = i; % Set k as ball that will undergo next event.
            
        end
        
    end  
    
    for i = 1:11
        
        if m(i) ~= 0
            
            Bp(i,:) = 0; % Update. Find position of ball i at next event
            vb(i,:) = 0; % Update. Find velocity of ball i at next event
            
        end
        
    end
    
    Bp_final{n} = Bp; n = n + 1; % Set cell n of Bp_final to the updated ball position.
    
    % Find the updated velocity for colliding or stopping balls.
    
    if isa(c{k},'double') % collides with stationary ball c{k}
        
        vb(k,:) = 0; % Update. Find new velocity of ball k
        vb(c{k},:) = 0; % Update. Find new velocity of ball c{k}

    elseif c{k} ~= 'S' % collides with wall    
        
        vb(k,:) = 0; % Update. Find new velocity of ball k
        
    elseif c{k} == 'S' % Ball stops without collision.
  
        vb(k,:) = [0 0]; m(k) = 0; % Ball k stops, set its velocity to 0
                                   % and set m(k) = 0 to signify it has
                                   % stopped moving.
        
    elseif c{k} == 0 % Update. Collides with moving ball
              
    end
      
    figure % Update. Show the updated ball position at each step.
    set(gcf, 'Position',  [100, 100, 800, 400])
    hold on
    for i = 1:11
        scatter(Bp(i,1),Bp(i,2),550,'filled','MarkerEdgeColor','k','LineWidth',0.5)
        if m(i) % If ball i is moving
            plot([Bp(i,1),vb(i,1)/4+Bp(i,1)],[Bp(i,2),vb(i,2)/4+Bp(i,2)],'k') % 
        end
    end
    xlim([0 a])
    ylim([0 b])
    colororder([0.9290 0.6940 0.1250; 0 0.4470 0.7410;1 0 0;0.4940 0.1840 0.5560;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880;0.6350 0.0780 0.1840;0 0 0;0.3010 0.7450 0.9330;0.5 0.5 0.5;1 1 1])
 
end

% Cycle through the stored ball positions in the same figure to animate
% movement
pause on
figure
box on
set(gcf, 'Position',  [100, 100, 800, 400])
xlim([0 a])
ylim([0 b])
for j = 1:length(Bp_final)
    clf % Clears the previous contents of the figure. 
    for i = 1:11
        hold on
        scatter(Bp_final{j}(i,1),Bp_final{j}(i,2),550,'filled','MarkerEdgeColor','k','LineWidth',0.5)
        if i == 8
            text(Bp_final{j}(i,1),Bp_final{j}(i,2),num2str(i),'HorizontalAlignment','center','Color',[1 1 1]);
        elseif i == 11
            text(Bp_final{j}(i,1),Bp_final{j}(i,2),'C','HorizontalAlignment','center');
        else
            text(Bp_final{j}(i,1),Bp_final{j}(i,2),num2str(i),'HorizontalAlignment','center','Color',[1 1 1]);
        end
    end
    colororder([0.9290 0.6940 0.1250; 0 0.4470 0.7410;1 0 0;0.4940 0.1840 0.5560;0.8500 0.3250 0.0980;0.4660 0.6740 0.1880;0.6350 0.0780 0.1840;0 0 0;0.3010 0.7450 0.9330;0.5 0.5 0.5;1 1 1])
    xlim([0 a])
    ylim([0 b])
    box on
    pause(0.2) % Pause for 0.2 s before updating the figure. 
    
end
%}

%{
[F1, F2] = ExampleFunction(1,2,3);

function [Ex1, Ex2] = ExampleFunction(n1,n2,n3)

    Ex1 = [n1 n2 n3];
    Ex2 = [n1+n2 n2+n3 n1*n3];

end
%}