%% PoolBall class
% Carries the properties and states of a ball on the table
classdef PoolBall < handle
    %% Shared Properties
    properties (Constant)
        table_length = convlength(88,'in','m');  % Length of pool table (x-direction)
        table_width =  convlength(44,'in','m'); % Width of pool table (y-direction)

        mu_bb = 0.05; % Coefficient of friction between 2 balls
        mu_bs = 0.10; % Coefficient of friction between ball and surface
        mu_bc = 0.20; % Coefficient of friction between ball and wall cushion/bumper

        e_cn = 0.95; % Coefficient of resistution between cue ball and numbered ball
        e_nn = 0.90; % Coefficient of restitution between 2 numbered balls
        e_bc = 0.70; % Coefficient of restitution between ball and wall cushion/bumper
        
        gravity = 9.80665; % Earths gravity in m/s
    end
    %% Particular Properties
    properties
        mass; % Mass of the ball (kg)
        rad;  % Radius of the ball (m)
        home_pos % Starting position of the ball (m)
        pos; % Current position of the ball (m)
        vel; % Current velocity of the ball (m/s)
        color; % Color of the ball (RGB matrix)
        label; % Label of the ball (1-2 Characters)
        is_cue; % Boolean specifying if this ball is a/the cue ball
        colliding_balls= PoolBall.empty; % List of balls currently in collision with this
        ball_img; % Image object for the ball
        spot_img; % White spot object for the ball
        label_img; % Label img for the ball
    end
    %% Methods
    methods
        %% PoolBall Constructor
        % Initializes the ball given the specified parameters
        function PB = PoolBall(mass,rad,pos,color,label,is_cue)
            PB.mass = mass / 1000; % Convert grams to kilograms
            PB.rad = convlength(rad, 'in', 'm'); % Convert inches to meters
            PB.home_pos = convlength(pos, 'in', 'm'); % Convert inches to meters
            PB.pos = PB.home_pos;
            PB.vel = [0,0]; % Keep m/s as m/s
            PB.color = color;
            PB.label = label;
            PB.is_cue = is_cue;
            a=[0:0.1:2*pi];
            Xcircle=cos(a);
            Ycircle=sin(a);
            PB.ball_img = patch (PB.pos(1)+PB.rad*Xcircle, PB.pos(2)+PB.rad*Ycircle, PB.color, 'FaceAlpha',1);
            PB.spot_img = patch (PB.pos(1)+PB.rad*Xcircle/2, PB.pos(2)+PB.rad*Ycircle/2, 'w', 'FaceAlpha', 1, 'LineStyle' ,'none');
            PB.label_img = text(PB.pos(1), PB.pos(2), PB.label, 'FontSize', 7, 'Color', 'b', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontWeight', 'bold');
        end
        %% PoolBall Move
        % Update the position and velocity of the pool ball based in the
        % velocity and surface friction
        function move(obj, time_slice)
            vel_angle = atan2(obj.vel(2), obj.vel(1));
            old_vel = norm(obj.vel); % Get the normal of the old velocity
            new_vel = old_vel - (time_slice * PoolBall.mu_bs * PoolBall.gravity); % Loss of velocity from friction
            if (new_vel < 0) 
                new_vel = 0; % Check if the new velocity was reduced past 0
            end
            effective_vel = [cos(vel_angle) * (old_vel + new_vel)/2, sin(vel_angle) * (old_vel + new_vel)/2];
            obj.vel = [cos(vel_angle) * new_vel, sin(vel_angle) * new_vel];
            obj.pos = obj.pos + effective_vel * time_slice; % Update the objects position
        end
        %% PoolBall Wall Collisions
        % Compute velocity changes due to wall/cushion/bumper collisions
        function compute_wall_collisions(obj)
            % Check if we collided with the x=max or x=0 "bumpers"
            max_x = PoolBall.table_length;
            max_y = PoolBall.table_width;
            if(obj.pos(1) > (max_x - obj.rad) || obj.pos(1) < obj.rad)
                old_vel = obj.vel; % Hold the original velocity for comparison
                obj.vel(1) = -(obj.vel(1)*PoolBall.e_bc); % Invert and reduce the perpendicular velocity by restitution factor
                obj.pos(1) = min(max(obj.pos(1),obj.rad),max_x-obj.rad); % Put the ball back within the limits (Eliminate glitches)
                obj.vel(2) = obj.vel(2) - sign(obj.vel(2))*(PoolBall.mu_bc * abs(obj.vel(1)-old_vel(1))); % Solve for frictional loss to parallel velocity
                if (sign(obj.vel(2)) ~= sign(old_vel(2))) % Ensure we didnt reverse direction due to friction
                    obj.vel(2) = 0;
                end
            end
            % Check if we collided with the y=max or y=0 "bumpers"
            if(obj.pos(2) > (max_y - obj.rad) || obj.pos(2) < obj.rad)
                old_vel = obj.vel;  % Solve for frictional loss to parallel velocity
                obj.vel(2) = -(obj.vel(2)*PoolBall.e_bc);  % Solve for frictional loss to parallel velocity
                obj.pos(2) = min(max(obj.pos(2),obj.rad),max_y-obj.rad);  % Solve for frictional loss to parallel velocity
                obj.vel(1) = obj.vel(1) - sign(obj.vel(1))*(PoolBall.mu_bc * abs(obj.vel(2)-old_vel(2)));  % Solve for frictional loss to parallel velocity
                if (sign(obj.vel(1)) ~= sign(old_vel(1)))  % Solve for frictional loss to parallel velocity
                    obj.vel(1) = 0;
                end
            end
        end
        %% PoolBall Ball Collisions
        % Compute velocity cahnges to ball<->ball collisions
        function compute_ball_collision(ball_a, ball_b)
            % Check if the balls collided
            if (norm(ball_a.pos - ball_b.pos) < (ball_a.rad + ball_b.rad))
                % Check if we are still colliding with this ball
                %   Approximation method needed for time-slice based approach.
                %   This avoids calculating multiple collisions while balls are
                %   clipped through eachother. Decreased time-slice reduces approximation
                %   error here.
                if (~isempty(ball_a.colliding_balls))
                    for i=1: length(ball_a.colliding_balls)
                        if (isequal(ball_b, ball_a.colliding_balls(i)))
                            return;
                        end
                    end
                end
                % Else, add the ball to the reference array
                ball_a.colliding_balls(end+1) = ball_b;
                % Check if either ball is the cue ball, select the correct e
                if (ball_a.is_cue || ball_b.is_cue)
                    e = PoolBall.e_bc;
                else
                    e = PoolBall.e_nn;
                end
                % Get the angles velocities, relative to the impact
                difference = ball_b.pos - ball_a.pos;
                impact_angle = atan2(difference(2), difference(1));
                ball_a_v = norm(ball_a.vel); % Absolute velocity of A
                ball_a_v_angle = atan2(ball_a.vel(2), ball_a.vel(1)); % Angle of velocity of A
                ball_b_v = norm(ball_b.vel); % Absolute velocity of B
                ball_b_v_angle = atan2(ball_b.vel(2), ball_b.vel(1)); % Angle of velocity of B
                ball_a_norm_v = cos(ball_a_v_angle - impact_angle) * ball_a_v; % norm portion of A velocity (compared to impact direction)
                ball_a_tan_v = sin(ball_a_v_angle - impact_angle) * ball_a_v;  % tan portion of A velocity (compared to impact direction)
                ball_b_norm_v = cos(ball_b_v_angle - impact_angle) * ball_b_v; % norm portion of B velocity (compared to impact direction)
                ball_b_tan_v = sin(ball_b_v_angle - impact_angle) * ball_b_v;  % tan portion of B velocity (compared to impact direction)
                % Create system of equations for the normal velocities, and solve
                syms new_ball_a_norm_v new_ball_b_norm_v
                norm_momentum = ball_a_norm_v * ball_a.mass + ball_b_norm_v * ball_b.mass == new_ball_a_norm_v * ball_a.mass + new_ball_b_norm_v * ball_b.mass;
                norm_restitution = e == (new_ball_a_norm_v - new_ball_b_norm_v)/(ball_b_norm_v - ball_a_norm_v);
                [A,B] = equationsToMatrix([norm_momentum, norm_restitution], [new_ball_a_norm_v, new_ball_b_norm_v]);
                X = linsolve(A,B);
                new_ball_a_norm_v = X(1); % Post-impact normal velocity of A
                new_ball_b_norm_v = X(2); % Post-impact normal velocity of B
                % Determine frictional losses on tangent velocities
                new_ball_a_tan_v = ball_a_tan_v - sign(ball_a_tan_v)*(PoolBall.mu_bb * (ball_a_norm_v - new_ball_a_norm_v));
                new_ball_b_tan_v = ball_b_tan_v - sign(ball_b_tan_v)*(PoolBall.mu_bb * (ball_b_norm_v - new_ball_b_norm_v));
                % Ensure frictional loss didnt change sign of velocity
                if (sign(new_ball_a_tan_v) ~= sign(ball_a_tan_v))
                    new_ball_a_tan_v = 0;
                end
                if (sign(new_ball_b_tan_v) ~= sign(ball_b_tan_v))
                    new_ball_b_tan_v = 0;
                end
                % Apply new velocities to balls
                ball_a.vel = [double(cos(impact_angle)*new_ball_a_norm_v - cos(pi/2 - impact_angle)*new_ball_a_tan_v), ...
                              double(sin(impact_angle)*new_ball_a_norm_v + sin(pi/2 - impact_angle)*new_ball_a_tan_v)];
                ball_b.vel = [double(cos(impact_angle)*new_ball_b_norm_v - cos(pi/2 - impact_angle)*new_ball_b_tan_v), ...
                              double(sin(impact_angle)*new_ball_b_norm_v + sin(pi/2 - impact_angle)*new_ball_b_tan_v)];
                return;
            % Check if we finished colliding with this ball
            elseif (~isempty(ball_a.colliding_balls))
                for i=1: length(ball_a.colliding_balls)
                    if (isequal(ball_b, ball_a.colliding_balls(i)))
                        ball_a.colliding_balls(i) = []; % Delete the ball reference
                    end
                end
            end
        end
        %% PoolBall Draw
        % Draw the ball on the graph at the current position
        function draw(obj)
            a=[0:0.1:2*pi];
            Xcircle=cos(a);
            Ycircle=sin(a);
            set(obj.ball_img,'XData',obj.pos(1)+obj.rad*Xcircle, 'YData', obj.pos(2)+obj.rad*Ycircle);
            set(obj.spot_img,'XData',obj.pos(1)+obj.rad*Xcircle/2, 'YData', obj.pos(2)+obj.rad*Ycircle/2);
            set(obj.label_img,'Position', [obj.pos(1) obj.pos(2) 0]);
        end
        %% PoolBall Reset
        % Reset the position and velocity of the ball
        function reset(obj)
            obj.pos = obj.home_pos;
            obj.vel = [0,0];
            obj.draw();
        end
        %% PoolBall Set Velocity
        % Set the velocity of the pool ball (Clear than value accessor)
        function set_vel(obj, vel)
            obj.vel = vel;
        end
        %% PoolBall Find Closest
        % Determine the closest ball from a given list of ball
        function closest = find_nearest(obj, balls)
            closest = PoolBall.empty;
            min_dist = norm([PoolBall.table_length, PoolBall.table_width]);
            for i=1: length(balls)
                distance = norm(balls(i).pos - obj.pos);
                if (distance < min_dist)
                    min_dist = distance;
                    closest = balls(i);
                end
            end
        end
    end
end