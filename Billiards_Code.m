close all;
clear all;
clc;

time_slice = 0.001;
latest_frame = 0;

% Background
patch([ 0, 0,88,88],[ 0,44,44, 0], 'g','FaceAlpha',0.5); 
grid on;
axis equal;
axis([0 PoolBall.table_length 0 PoolBall.table_width])
hold on;

% balls = [PoolBall(0.200, 2.25, [66.00,22.00],  [255,255,255]/255, 'C', true),
%          PoolBall(0.160, 2.25, [20,22],        [255,255,  0]/255, '1',  false),
%          PoolBall(0.160, 2.25, [16.05,24.251], [  0,  0,255]/255, '2',  false),
%          PoolBall(0.160, 2.25, [16.05,19.751], [255,  0,  0]/255, '3',  false),
%          PoolBall(0.160, 2.25, [12.1,26.501],  [144,  0,255]/255, '4',  false),
%          PoolBall(0.160, 2.25, [12.1,22],      [255,144,  0]/255, '5',  false),
%          PoolBall(0.160, 2.25, [12.1,17.499],  [  0,255,  0]/255, '6',  false),
%          PoolBall(0.160, 2.25, [8.15,28.752],  [255,255,144]/255, '7',  false),
%          PoolBall(0.160, 2.25, [8.15,24.251],  [  0,  0,  0]/255, '8',  false),
%          PoolBall(0.160, 2.25, [8.15,19.749],  [138, 92, 51]/255, '9',  false),
%          PoolBall(0.160, 2.25, [8.15,15.248],  [  0,  0,144]/255, '10', false)];
% balls(1).set_vel([-100,0]);

% Create array of Pool Balls with given specifications
balls = [PoolBall(0.200, 2.25, [66.00,22.00], [255,255,255]/255, 'C', true),
         PoolBall(0.160, 2.25, [40.08,21.23], [255,255,  0]/255, '1',  false),
         PoolBall(0.160, 2.25, [17.52,37.29], [  0,  0,255]/255, '2',  false),
         PoolBall(0.160, 2.25, [12.53, 2.75], [255,  0,  0]/255, '3',  false),
         PoolBall(0.160, 2.25, [10.47,25.10], [144,  0,255]/255, '4',  false),
         PoolBall(0.160, 2.25, [27.32, 4.87], [255,144,  0]/255, '5',  false),
         PoolBall(0.160, 2.25, [16.25,13.80], [  0,255,  0]/255, '6',  false),
         PoolBall(0.160, 2.25, [ 7.60,21.49], [255,255,144]/255, '7',  false),
         PoolBall(0.160, 2.25, [10.81, 9.21], [  0,  0,  0]/255, '8',  false),
         PoolBall(0.160, 2.25, [26.19,21.84], [138, 92, 51]/255, '9',  false),
         PoolBall(0.160, 2.25, [19.78,26.81], [  0,  0,144]/255, '10', false)];

% Determine Closest ball to cue and angle for shot
closest_ball = balls(1).find_nearest(balls(2:end));
vector_between = closest_ball.pos - balls(1).pos;
theta_closest = atan2(vector_between(2), vector_between(1));

% Create array of Cue Velocities to test
cue_velocities = [
                  [ -cos(deg2rad( 0))*  4,   sin(deg2rad( 0))*  4], % 4 m/s directly left
                  [ -cos(deg2rad(15))*2.5,   sin(deg2rad(15))*2.5], % 2.5 m/s @ 15° upward toward the left
                  [ -cos(deg2rad(30))*1.5,   sin(deg2rad(30))*1.5], % 1.5 m/s @ 30° upward toward the left
                  [cos(theta_closest)*1.5, sin(theta_closest)*1.5], % 1.5 m/s @ angle to closest ball
                 ];
cue_velocities = cue_velocities * 39.3701; % Convert from m/s to inch/sec

writerObj = VideoWriter('elasticCollision.avi');
writerObj.FrameRate = 1 / time_slice;
writerObj.Quality = 95;
open(writerObj);


% Interate over each cue velocity to test
for i=1: length(cue_velocities)
    % Reset the pool balls
    for j=1: length(balls)
        balls(j).reset;
    end
    % Set the cue velocity for the test
    balls(1).set_vel(cue_velocities(i,:));

    % Simulate the shot
    while(true)
        % Check if any balls are still moving.
        any_moving=false;
        for j=1:length(balls)
            if (any(balls(j).vel))
                any_moving=true;
            end
        end
        % If none are moving, stop calculating frames.
        if (not(any_moving))
            break;
        end
        
        % Calculate motion for each ball
        for j=1:length(balls)
            balls(j).move(time_slice); % Update the position with velocity and apply surface friction
            balls(j).compute_wall_collisions(); % Compute the velocity changes from wall collisions
            % Collide each ball with all the balls after it in the array
            % (Covers all combinations)
            if j+1 <= length(balls)
                for k=j+1:length(balls)
                    balls(j).compute_ball_collision(balls(k));
                end
            end
            balls(j).draw(); % Draw the current position of the ball on the graph
        end

        drawnow; % Render the graph
        latest_frame = getframe(gcf); % Save the rendered graph as a video frame
        writeVideo(writerObj, latest_frame); % Write the frame to the overall video file
    end

    fprintf("Simulation %i: Cue velocity = [%6.3f,%6.3f]\n", i, cue_velocities(i,1), cue_velocities(i,2));
    fprintf("    Final Ball positions:\n")
    for j=1: length(balls)
        fprintf("    Ball %2s: [%7.4f, %7.4f]\n", balls(j).label, balls(j).pos(1), balls(j).pos(2));
    end
    fprintf("\n");

    % Add the latest frame to the video repeatedly for the next 1 second
    % (Creates a 1 second pause between video sections)
    for j=1: round(1/time_slice)
        writeVideo(writerObj, latest_frame)
    end
end
close(writerObj);
