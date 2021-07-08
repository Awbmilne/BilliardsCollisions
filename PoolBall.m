classdef PoolBall < handle
    properties (Constant)
        table_length = 88;
        table_width = 44;

        mu_bb = 0.05;
        mu_bs = 0.10;
        mu_bc = 0.20;

        e_cn = 0.95;
        e_nn = 0.90;
        e_bc = 0.70;
    end
    properties
        mass;
        rad;
        home_pos
        pos;
        home_vel
        vel;
        color;
        label;
        is_cue;
        colliding_balls= PoolBall.empty;
        ball_img;
        spot_img;
        label_img;
    end
    methods
        function PB = PoolBall(mass,rad,pos,color,label,is_cue)
            PB.mass = mass;
            PB.rad = rad;
            PB.home_pos = pos;
            PB.pos = pos;
            PB.vel = [0,0];
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
        function move(obj, time_slice)
            gravity=9.80665;
            vel_angle = atan2(obj.vel(2), obj.vel(1));
            old_vel = norm(obj.vel); % Get the normal of the old velocity
            new_vel = old_vel - ((time_slice * PoolBall.mu_bs * gravity)); % Loss of velocity from friction
            if (new_vel < 0) 
                new_vel = 0; % Check if the new velocity was reduced past 0
            end
            effective_vel = (old_vel + new_vel)/2;
            effective_vel = [cos(vel_angle) * effective_vel, sin(vel_angle) * effective_vel];
            obj.vel = [cos(vel_angle) * new_vel, sin(vel_angle) * new_vel];
            obj.pos = obj.pos + effective_vel * time_slice; % Update the objects position
        end
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
        function compute_ball_collision(ball_a, ball_b)
            % Check if the balls collided
            if (norm(ball_a.pos - ball_b.pos) < (ball_a.rad + ball_b.rad))
                % Check if we are still colliding with this ball
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
                ball_a_v = norm(ball_a.vel);
                ball_a_v_angle = atan2(ball_a.vel(2), ball_a.vel(1));
                ball_b_v = norm(ball_b.vel);
                ball_b_v_angle = atan2(ball_b.vel(2), ball_b.vel(1));
                ball_a_norm_v = cos(ball_a_v_angle - impact_angle) * ball_a_v;
                ball_a_tan_v = sin(ball_a_v_angle - impact_angle) * ball_a_v;
                ball_b_norm_v = cos(ball_b_v_angle - impact_angle) * ball_b_v;
                ball_b_tan_v = sin(ball_b_v_angle - impact_angle) * ball_b_v;
                % Create system of equations for the normal velocities, and solve
                syms new_ball_a_norm_v new_ball_b_norm_v
                norm_momentum = ball_a_norm_v * ball_a.mass + ball_b_norm_v * ball_b.mass == new_ball_a_norm_v * ball_a.mass + new_ball_b_norm_v * ball_b.mass;
                norm_restitution = e == (new_ball_a_norm_v - new_ball_b_norm_v)/(ball_b_norm_v - ball_a_norm_v);
                [A,B] = equationsToMatrix([norm_momentum, norm_restitution], [new_ball_a_norm_v, new_ball_b_norm_v]);
                X = linsolve(A,B);
                new_ball_a_norm_v = X(1);
                new_ball_b_norm_v = X(2);
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
                ball_a.vel = [double(cos(impact_angle)*new_ball_a_norm_v - cos(pi/2 - impact_angle)*new_ball_a_tan_v),
                              double(sin(impact_angle)*new_ball_a_norm_v + sin(pi/2 - impact_angle)*new_ball_a_tan_v)];
                ball_b.vel = [double(cos(impact_angle)*new_ball_b_norm_v - cos(pi/2 - impact_angle)*new_ball_b_tan_v),
                              double(sin(impact_angle)*new_ball_b_norm_v + sin(pi/2 - impact_angle)*new_ball_b_tan_v)];
                return;
            % Check if we stopped colliding with this ball
            elseif (~isempty(ball_a.colliding_balls))
                for i=1: length(ball_a.colliding_balls)
                    if (isequal(ball_b, ball_a.colliding_balls(i)))
                        ball_a.colliding_balls(i) = []; % Delete the ball reference
                    end
                end
            end
        end
        function draw(obj)
            a=[0:0.1:2*pi];
            Xcircle=cos(a);
            Ycircle=sin(a);
            set(obj.ball_img,'XData',obj.pos(1)+obj.rad*Xcircle, 'YData', obj.pos(2)+obj.rad*Ycircle);
            set(obj.spot_img,'XData',obj.pos(1)+obj.rad*Xcircle/2, 'YData', obj.pos(2)+obj.rad*Ycircle/2);
            set(obj.label_img,'Position', [obj.pos(1) obj.pos(2) 0]);
        end
        function reset(obj)
            obj.pos = obj.home_pos;
            obj.vel = [0,0];
        end
        function set_vel(obj, vel)
            obj.vel = vel;
        end
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