classdef ReConArmClass < handle
    % ReConArm: 3R+P spatial manipulator (RRPR-like) with telescoping middle link
    % q = [theta1; theta2; theta4; d3]
    % params: a3 (link length), d1 (base offset), theta24_des (fixed endpoint orientation)
    %Calculation for this IK can be found in our Google Doc planner
    
    properties
        a3      % length of last link
        d1      % base offset along z
        theta24_des  % preferred fixed orientation (theta2 + theta4)
    end
    
    properties (Access = private)
        q   % current joint state [4x1]
    end
    
    methods
        %% Constructor
        function this = ReConArmClass(a3, d1, theta24_des)
            if nargin < 3
                % default: tool z-axis pointing roughly downward
                theta24_des = -pi/2;
            end
            this.a3 = a3;
            this.d1 = d1;
            this.theta24_des = theta24_des;
            this.q = zeros(4,1);
        end
        
        %% Setters / getters
        function setJoint(this, q)
            this.q = q(:);
        end
        
        function q = getJoint(this)
            q = this.q;
        end
        
        %% Forward kinematics T04 (from DH matrix we calculated)
        function T = fk(this, q)
            q = q(:);
            theta1 = q(1);
            theta2 = q(2);
            theta4 = q(3);
            d3     = q(4);
            
            a3 = this.a3; %I need to look closely into this variable name
            d1 = this.d1;
            
            c1 = cos(theta1); s1 = sin(theta1);
            c2 = cos(theta2); s2 = sin(theta2);
            c23 = cos(theta2+theta4);
            s23 = sin(theta2+theta4);
            
            % rotation + position terms from your FK
            nx = -c1*c23;
            ox = -c1*s23;
            ax =  s1;
            px = -c1*(a3*c23 - d3*s2);
            
            ny = -s1*c23;
            oy = -s1*s23;
            ay = -c1;
            py = -s1*(a3*c23 - d3*s2);
            
            nz = -s23;
            oz = -c23;
            az =  0;
            pz = d1 - a3*s23 - d3*c2;
            
            T = [nx ox ax px;
                 ny oy ay py;
                 nz oz az pz;
                 0  0  0  1 ];
        end
        
        %% Position of joints and end-effector (for plotting)
        function [O, P1, P2, P4] = jointPositions(this, q)
            % O: base origin (0,0,0). location of J1
            % P1: after joint 1 (assume at (0,0,d1)). location of J2
            % P2: after joint 2 and prismatic (end of telescoping link) location of J4
            % P4: end-effector
            
            if nargin < 2
                q = this.q;
            end
            q = q(:);
            theta1 = q(1);
            theta2 = q(2);
            d3     = q(4);
            
            a3 = this.a3;
            d1 = this.d1;
            
            c1 = cos(theta1); s1 = sin(theta1);
            c2 = cos(theta2); s2 = sin(theta2);
            
            % Base
            O = [0;0;0];
            
            % After joint 1 (rotation only, plus offset d1 along z)
            P1 = [0; 0; d1];
            
            % Direction of link2 (after joint2: rotation about y in plane containing z)
            % link-2 length before telescoping is not explicitly used, but
            % we treat (a2 + d3) as telescoping along joint-2 direction.
            % For visualization, assume link2 length = d3 (for now).
            % Position of P2 (end of telescoping link):
            % Start at P1, rotate by theta1 about z and theta2 about y, then extend along -z of that frame
            % For simplicity, use the p-part from FK with a3=0:
            % px2 = -c1*(- d3*s2) = c1*d3*s2
            % py2 = -s1*(- d3*s2) = s1*d3*s2
            % pz2 = d1 - 0 - d3*c2
            P2 = [ c1*d3*s2;
                   s1*d3*s2;
                   d1 - d3*c2 ];
            
            % P4 from full FK
            T4 = this.fk(q);
            P4 = T4(1:3,4);
        end
        
        %% Inverse kinematics: position-only with fixed theta23
        function q_solutions = ik_positionOnly(this, p)
            % p = [x; y; z] desired end-effector position
            % uses a fixed theta24_des = this.theta24_des
            % returns struct array like:
            % q_solutions(k).q   (4x1)
            % q_solutions(k).valid
            
            px = p(1); py = p(2); pz = p(3);
            
            a3 = this.a3;
            d1 = this.d1;
            theta23 = this.theta24_des;
            
            c23 = cos(theta23);
            s23 = sin(theta23);
            
            % 1) theta1 from projection onto XY
            theta1 = atan2(-py, -px);  % robust
            
            % 2) R from position projection
            R = -( px*cos(theta1) + py*sin(theta1) );  % = a3*c23 - d3*sin(theta2)
            
            % 3) N and C from equations (derivation in our Google Doc)
            N = a3*c23 - R;                  % = d3*sin(theta2)
            C = d1 - a3*s23 - pz;            % = d3*cos(theta2)
            
            sq = N^2 + C^2;
            q_solutions = struct('q',{},'valid',{});
            if sq < 0
                return;  % unreachable numerically
            end
            
            d3_pos =  sqrt(sq);
            d3_neg = -d3_pos;
            
            % Candidate 1: d3 >= 0 preferred
            candidates = [];
            if d3_pos >= 0
                candidates = [candidates d3_pos];
            end
            % if you want to allow negative, uncomment:
            if d3_neg >= 0 % if you want both, drop this condition
                candidates = [candidates d3_neg];
            end
            
            for k = 1:numel(candidates)
                d3 = candidates(k);
                if d3 ~= 0
                    theta2 = atan2(N, C);    % sin = N/d3, cos = C/d3
                %else
                    %theta2 = atan2(N, C);    % degenerate; still defined
                end
                theta4 = theta23 - theta2;
                
                q = [theta1; theta2; theta4; d3];
                q_solutions(end+1).q = q; %#ok<AGROW>
                q_solutions(end).valid = true;
            end
        end
        %%
        function q_solutions = ik_pose(this, p, theta23_des)
            % Inverse kinematics for position + desired theta23
            % p = [x; y; z] desired end-effector position
            % theta23_des = desired (theta2 + theta4)
            % returns struct array with fields:
            %   q     : [theta1; theta2; theta4; d3]
            %   valid : boolean
            
            px = p(1); py = p(2); pz = p(3);
        
            a3 = this.a3;
            d1 = this.d1;
            theta23 = theta23_des;
        
            c23 = cos(theta23);
            s23 = sin(theta23);
        
            % 1) theta1 from XY projection (same as before, robust)
            theta1 = atan2(-py, -px);
        
            % 2) R from position equation:
            % R = a3*c23 - d3*sin(theta2) = -(px*cos(theta1) + py*sin(theta1))
            R = -( px*cos(theta1) + py*sin(theta1) );
        
            % 3) N and C from FK equations:
            % N = a3*c23 - R = d3*sin(theta2)
            % C = d1 - a3*s23 - pz = d3*cos(theta2)
            N = a3*c23 - R;
            C = d1 - a3*s23 - pz;
        
            sq = N^2 + C^2;
            q_solutions = struct('q',{},'valid',{});
            if sq < 0
                return;  % numerically unreachable
            end
        
            d3_pos =  sqrt(sq);
            d3_neg = -d3_pos;
        
            candidates = [];
            % allow both signs if you want; usually you want d3 >= 0
            if d3_pos >= 0
                candidates = [candidates d3_pos];
            end
            % comment this out if you *only* want positive d3:
            % if d3_neg >= 0
            %     candidates = [candidates d3_neg];
            % end
        
            for k = 1:numel(candidates)
                d3 = candidates(k);
                if d3 ~= 0
                    theta2 = atan2(N, C);  % since sin(theta2)=N/d3, cos(theta2)=C/d3
                else
                    theta2 = atan2(N, C);
                end
                theta4 = theta23 - theta2;
        
                q = [theta1; theta2; theta4; d3];
                q_solutions(end+1).q = q; %#ok<AGROW>
                q_solutions(end).valid = true;
            end
        end

        %% Cubic joint trajectory between two 4x1 joint vectors
        function Q = cubicTrajectory(this, q_init, q_final, t_total, N)
            q_init  = q_init(:);
            q_final = q_final(:);
            T = linspace(0, t_total, N);
            Q = zeros(4, N);
            
            for j = 1:4
                qi = q_init(j);
                qf = q_final(j);
                vi = 0; vf = 0;
                M = [ 0    0    0 1;
                      t_total^3 t_total^2 t_total 1;
                      0    0    1 0;
                      3*t_total^2 2*t_total 1 0 ];
                coeff = M \ [qi; qf; vi; vf];
                Q(j,:) = coeff(1).*T.^3 + coeff(2).*T.^2 + coeff(3).*T + coeff(4);
            end
        end
        
        %% Plot robot in 3D
        function plotRobot(this, q)
            if nargin < 2
                q = this.q;
            end
            [O,P1,P2,P4] = this.jointPositions(q);
        
            % Link polyline
            X = [O(1) P1(1) P2(1) P4(1)];
            Y = [O(2) P1(2) P2(2) P4(2)];
            Z = [O(3) P1(3) P2(3) P4(3)];
            
            % Clear axes but keep settings
            cla;
            hold on;
        
            % 1) Draw links
            plot3(X, Y, Z, '-', 'LineWidth', 3);
        
            % 2) Draw joints (base, J1, J2) as filled circles
            plot3(O(1),  O(2),  O(3),  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
            plot3(P1(1), P1(2), P1(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
            plot3(P2(1), P2(2), P2(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
        
            % 3) Draw end-effector as big red star
            plot3(P4(1), P4(2), P4(3), 'r*', 'MarkerSize', 12, 'LineWidth', 1.5);
        
            % Axes cosmetics
            axis equal;
            grid on;
            xlabel('X'); ylabel('Y'); zlabel('Z');
        
            % Make viewing angle nicer
            view(45, 25);
        
            % Set consistent axis limits (edit to match your robot scale)
            axis([-1 3 -1 3 0 3]);
        
            % Optional: add a subtle title
            title('ReCon Arm Motion');
        
            hold off;
        end
        
        %% Animate motion
        function animateMotion(this, Q, t_total)
            [~, N] = size(Q);
            T = linspace(0, t_total, N);
            for k = 1:N
                qk = Q(:,k);
                this.setJoint(qk);
                this.plotRobot(qk);
                Xmin = -5;
                Xmax = 5;
                Ymin = -5;
                Ymax = 5;
                Zmin = 0;
                Zmax = 5;
                axis([Xmin Xmax Ymin Ymax Zmin Zmax]);  % adjust for our link sizes
                drawnow
                if k < N
                    pause(T(k+1) - T(k));
                end
                cla
            end
        end
    end
end
