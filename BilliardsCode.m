% Clear the commands and variables
close all;
clear all;
clc;
output_dir = "out/matlab/";

% Start logging the output to diary/log file
dfile = output_dir + 'output.log';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on;

% Set the time interval for calculations
% (Determines precision)
time_slice = 0.000001; % Recompute position and velocity every 1/1,000,000 of a second
slices_per_sec = 1 / time_slice; % Theoretical frames/sec
frame_divider = round(slices_per_sec / 60); % Aim for roughly 60 fps for output video
frame_rate = slices_per_sec/frame_divider;

% Create frame and video objects for storing video frames
combinedVideo = VideoWriter(output_dir + 'videos/combinedCollisions.avi');
combinedVideo.FrameRate = frame_rate;
combinedVideo.Quality = 95;
open(combinedVideo);

% Setup and draw pool table background
grid on;
hold on;
axis equal;
axis off;
patch([ 0, 0,88,88],[ 0,44,44, 0], 'g','FaceAlpha',0.5); 
axis([0 PoolBall.table_length 0 PoolBall.table_width]);

% Create array of Pool Balls with given properties and locations
%                 | Mass(g) | Rad(in) | Position(in) |    Colour(RGB)   | TT | is_cue?
balls = [PoolBall(       200,     2.25, [66.00,22.00], [255,255,255]/255, 'C',   true), ...
         PoolBall(       160,     2.25, [40.08,21.23], [255,255,  0]/255, '1',  false), ...
         PoolBall(       160,     2.25, [17.52,37.29], [  0,  0,255]/255, '2',  false), ...
         PoolBall(       160,     2.25, [12.53, 2.75], [255,  0,  0]/255, '3',  false), ...
         PoolBall(       160,     2.25, [10.47,25.10], [144,  0,255]/255, '4',  false), ...
         PoolBall(       160,     2.25, [27.32, 4.87], [255,144,  0]/255, '5',  false), ...
         PoolBall(       160,     2.25, [16.25,13.80], [  0,255,  0]/255, '6',  false), ...
         PoolBall(       160,     2.25, [ 7.60,21.49], [163,166, 27]/255, '7',  false), ...
         PoolBall(       160,     2.25, [10.81, 9.21], [  0,  0,  0]/255, '8',  false), ...
         PoolBall(       160,     2.25, [26.19,21.84], [138, 92, 51]/255, '9',  false), ...
         PoolBall(       160,     2.25, [19.78,26.81], [  0,  0,144]/255, '10', false)];
ball_pairs = num2cell(nchoosek(balls, 2), 2); % Create Cell array of ball pair combinations
cue_ball = balls(1); % Synonym for Cue Ball

% Determine Closest ball to cue and angle for shot
closest_ball = balls(1).find_nearest(balls(2:end));
vector_between = closest_ball.pos - balls(1).pos;
theta_closest = atan2(vector_between(2), vector_between(1));
fprintf("Closest ball to Cue ball: Ball %2s\n", closest_ball.label) % Log closest ball
fprintf("Attitude to closest ball: %7.3f deg\n\n", rad2deg(theta_closest)); % Log attitude to closest ball

% Create the array of Cue Ball Velocities to test
cue_velocities = ...
[
    [ -cos(deg2rad( 0))*  4,   sin(deg2rad( 0))*  4];... % 4 m/s directly left
    [ -cos(deg2rad(15))*2.5,   sin(deg2rad(15))*2.5];... % 2.5 m/s @ 15 deg upward toward the left
    [ -cos(deg2rad(30))*1.5,   sin(deg2rad(30))*1.5];... % 1.5 m/s @ 30 deg upward toward the left
    [cos(theta_closest)*1.5, sin(theta_closest)*1.5];... % 1.5 m/s @ angle to closest ball
];


%% Physics Simulation
% Interate over each cue velocity to test
for i=1: length(cue_velocities)
    % Reset the pool balls
    arrayfun(@(ball) ball.reset, balls);

    %% Setup Frame and Video
    % Create frame and video objects for storing video frames
    individualVideo = VideoWriter(output_dir + 'videos/' + sprintf("collision_%i", i) + '.avi');
    individualVideo.FrameRate = frame_rate;
    individualVideo.Quality = 95;
    open(individualVideo);
    % Save the start layout as an image
    exportgraphics(gca,output_dir + sprintf("images/Simulation_%i_start.png", i),'Resolution',600)

    % Set the cue velocity for the test
    cue_ball.set_vel(cue_velocities(i,:));

    % Simulate the shot
    frame_counter=0;
    while(true)
        % Break loop if no balls are moving
        if (~any(arrayfun(@(ball) any(ball.vel), balls))) break, end;

        % Increment frame counter
        frame_counter = frame_counter + 1;
        
        % Ball physics
        arrayfun(@(ball) ball.move(time_slice), balls); % Update the position with velocity and apply rolling friction
        arrayfun(@(ball) ball.compute_wall_collisions(), balls); % Compute the velocity changes from wall collisions
        arrayfun(@(pair) pair{1}(1).compute_ball_collision(pair{1}(2)), ball_pairs); % Check collisions in each pair of balls
        
        % Render and write frames to the videos, subject to the frame divider
        if (frame_counter == 0 || mod(frame_counter, frame_divider) == 0)
            arrayfun(@(ball) ball.draw, balls); % Draw all the balls
            frame = getframe(gcf); % Save the rendered graph as a video frame
            writeVideo(combinedVideo, frame); % Write the frame to the video file
            writeVideo(individualVideo, frame) % Write the frame to the video file
        end
    end
    
    % Add the frame to the video repeatedly for the next 1 second
    % (Creates a 1 second pause at the end of the collision)
    arrayfun(@(ball) ball.draw, balls); % Draw all the balls
    frame = getframe(gcf); % Save the rendered graph as a video frame
    for j=1: frame_rate
        writeVideo(combinedVideo, frame) % Write the frame to the video file
        writeVideo(individualVideo, frame) % Write the frame to the video file
    end
    close(individualVideo); % Finish the specific collision video
    exportgraphics(gca,output_dir + sprintf("images/Simulation_%i_end.png", i),'Resolution',600) % Save the final layout of the balls as an image

    % Output the simulation results
    fprintf("Simulation %i: Cue velocity = [%6.3f,%6.3f] (m/s)\n", i, cue_velocities(i,1), cue_velocities(i,2));
    fprintf("    Final Ball positions [X,Y]:\n")
    for j=1: length(balls) % List all ball positions
        fprintf("    Ball %2s: [%7.4f, %7.4f] (in)\n", ...
                balls(j).label,                        ...
                convlength(balls(j).pos(1),'m','in'),  ...
                convlength(balls(j).pos(2),'m','in')); ...
    end
    fprintf("\n");    
end

%% Clean up Script
close(combinedVideo); % Wrap up the video file
diary off; % Close the diary/log file
