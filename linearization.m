close all; 
clc; 

z_trim = [0 -1.5976e-07 0 0]';  %Equilibrium point found in costfn_iterations.m 
u_str = 0; 
e= 0.0001;
e1=e*[1 0 0 0]';
e2=e*[0 1 0 0]';
e3=e*[0 0 1 0]'; 
e4=e*[0 0 0 1]'; 
 

z_dot = nonlinear(z_trim,u_str); 


%%%% Small value epsilon added to system @ equilibrium point %%%%
z1_dot= nonlinear(z_trim+e1,u_str);
z2_dot= nonlinear(z_trim+e2,u_str); 
z3_dot= nonlinear(z_trim+e3,u_str); 
z4_dot= nonlinear(z_trim+e4,u_str); 

%%%%% 'A' matrix given by, A=df/dz %%%%

dgiz1=(z1_dot-z_dot)/e; 
dgiz2=(z2_dot-z_dot)/e;
dgiz3=(z3_dot-z_dot)/e;
dgiz4=(z4_dot-z_dot)/e;

A_linear= [dgiz1 dgiz2 dgiz3 dgiz4] 

%%%% 'B' matrix given by, B=df/du %%%%

z_dotu_str= nonlinear(z_trim,u_str+e); 

B_linear=(z_dotu_str-z_dot)/e


%%%%% Control by applying servo input voltage %%%%

Rm=2.6;
Kt=0.0077;
nm=1;
Km=0.0077;
Kg=3.71;
ng=1;
r_mp=0.0063; 

A_linear(3,3) = (A_linear(3,3))- ((B_linear(3))*(ng)*(Kg^2)*(Kt)*(Km/(r_mp^2)/Rm) )

A_linear(4,3) = (A_linear(4,3))- ((B_linear(4))*(ng)*(Kg^2)*(Kt)*(Km/(r_mp^2)/Rm) )

B_linear = ng*Kg*nm*Kt/r_mp/Rm*B_linear




