%{
- This program computes the forward kinematics of our proposed ReConArm
- It also computes the forward kinematics (FK) of a conventional 3R
manipulator
- The result of this is used to estimate the workspace of both manipulators
to see determine if our ReCon Arm achieves a wider workspace for the same
parameters as the conventional 3R manipulator
%}

function [matrix] = dh_link(theta,alpha,rx,dz,angle)
%{obtain from Matlab opensource community to easily compute homogenous
%matrix given a DH-Table
%}
if angle == "d"
    matrix = [
        cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) rx*cosd(theta);...
        sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) rx*sind(theta);...
        0 sind(alpha) cosd(alpha) dz;...
        0 0 0 1 ...
    ];
end
if angle == "r"
    matrix = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) rx*cos(theta);...
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) rx*sin(theta);...
        0 sin(alpha) cos(alpha) dz;...
        0 0 0 1 ...
    ];
end
matrix = vpa(matrix);
end


function [DH_HTM] = DH_HTM(Matrix,angtype)
% Input Matrix: DH Table of (n,4) Dimension, else throw error
% Output matrix: Homogenous transformation: Dimension (4,4)
if size(Matrix,2) ~= 4
    error("Matrix must have 4 columns");
end
output = eye(4);
len = size(Matrix,1); % Number of Rows
for i = 1 : len
    params = Matrix(i,:);
    theta = params(1);
    alpha = params(2);
    rx = params(3);
    dz = params(4);
    next = dh_link(theta,alpha,rx,dz,angtype);
    output = output * next;
end
output = simplify(output);
DH_HTM = output;
end

%FK for CONVENTIONAL MATRIX
syms a2 a3 d1 theta1 theta2 theta3
%matrix = [theta, alpha, a ,d];
conventional3R_matrix = [theta1,pi/2,0,d1; theta2,0,a2,0; theta3,0,a3,0];
DH_HTM(conventional3R_matrix, 'r')

%FK for ReCon ARM MATRIX
syms a3 d1 d3 theta1 theta2 theta4 %d1 is a constant
%matrix = [theta, alpha, a ,d];
ReConARM_matrix = [theta1,pi/2,0,d1; theta2,pi/2,0,0; pi,pi/2,0,d3; theta4,0,a3,0;];
DH_HTM(ReConARM_matrix, 'r')