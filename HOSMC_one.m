%%MATLAB Code for Higher Order Sliding Mode Control (1st order)
%% Piyuri_Patrale
clc;
clear all; 
close all; 

%% Linearized model of inverted pendulum
A=[0 1 0 0;0 0 -1.56 0;0 0 0 1;0 0 46.87 0];           % Matrix A
B=[0;0.97;0;-3.98];                                    % Matrix B
D=0;
e1=[0 0 0 1];                                          % e1 to find first vector of column C
iden = eye(4);                                         % generates a 4x4 identitiy matrix  
G_s= (A+5*iden)^3                                      % Transfer function G(s)

%% Controllability Matrix P_con
P_con= [B A*B A*A*B A*A*A*B]                            % Controllability matrix
P_inv= inv(P_con)                                       % Finding inverse of P matrix 
P_last= P_con(4,:)                                      % To extract last column of controllability matrix
T_simi=[P_last; P_last*A; P_last*A*A; P_last*A*A*A]     % Similarity transformation matrix T
T_inv= inv(T_simi)                                      % Inverse of transformation matrix
P_new=P_inv*(T_simi)                                     % New matrix P
G_s1= T_inv*G_s*T_simi

%% Matrix C
C = e1*P_new*G_s1*T_inv                                 % to find gain matrix

%% Transfer function
[num,den]=ss2tf(A,B,C,D)                                % finds num & den from state space
G_Snew=tf(num,den)                                        % New TF
%% A,B,C after performing similarity transformation
A_new=T_inv*A*T_simi
B_new=T_inv*B
C_new = C*T_simi


