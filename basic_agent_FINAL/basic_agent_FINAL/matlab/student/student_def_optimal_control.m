%%
%             Agent Logic
%       Optimal Control Solution
%                2024
%
%%
clear; clc; close all

%% Plant equations
syms s(t) v(t) a(t) u(t) l1(t) l2(t) l3(t);
% ode1 = ...
% ode2 = ...
% ode3 = ...
% - Use diff() function to derive the function

%% Cost function inside the integral
% L = 

%% Hamiltonian
% H = L + ...
% - Use rhs() function for getting the right hand side

%% Solving the Hamiltonian
syms opt_u;
% opt_u = ...
% - Use solve subs() to replace variable inside the equation
%   and functionalDerivative() to detive w.r.t. a variable 

%% Second Optimality Condition
% Dl1 = ...
% Dl2 = ...
% Dl3 = ...
% - Use diff(,t) to derive w.r.t. the time 
%   and functionalDerivative() to detive w.r.t. a variable 

%% Substitute the optimal solution opt_u to state equations
% ode3s = ...
% - Replace the ode3 equation with the solution of the optimal control problem

%% Boundary condition on initial and final states  
% ICs = ... 
% FCs = ... 
% - Write the condition as a string

%% Find the solution of the OCP imposing the boundary condition
% sol_opt = ...
% - Use the function dsolve([], , ) to obtain a solution of the optimal
%   control problem

disp('Optimal polynomial longitudinal position:');
% pretty(sol_opt.s)

disp('Optimal polynomial velocity:');
% pretty(sol_opt.v)

disp('Optimal polynomial acceleration:');
% pretty(sol_opt.a)

%% Assign to functions the solutions found

%% Obtain optimal control solution
% sol_opt.j = ...
% - Use the subs function on the opt_u with the value of l3

%% Export the solution in a matlab function
syms t v0 a0 sf vf af T             
% s_opt_fun = matlabFunction(sol_opt.s,'Vars',[t,v0,a0,sf,vf,af,T],'File','s_opt.m');
% v_opt_fun = ...
% a_opt_fun = ...
% j_opt_fun = ...
% - Use the matlabFunction function to generate a matlab function using a
%   symbolic function

%% Export the coefficent list in a matlab function
% the coeffs are moltiplied by [1,2,6,24,120] to obtain the value of c1, c2, c3, c4, c5
% coef_list_var = [0,coeffs(sol_opt.s,t) .* [1,2,6,24,120]];
% coef_list_fun = ...
% - Use the matlabFunction function to generate a matlab function using a
%   symbolic function 

%% Export the total cost in a matlab function 
% total_cost_var = simplify(int(sol_opt.j^2,t,0,T));
% total_cost_fun = ...
% - Use the matlabFunction function to generate a matlab function using a
%   symbolic function

%% Inspect solutions 

Tmax  = 4.;
t_list = linspace(0,Tmax,100);

v0val = 10;
a0val = 1;
xfval = 90;
vfval = 25;
afval = 0.;

%% The functions work both for vector (t_list) or number (t)
% s_list = s_opt_fun(t_list, v0val, a0val, xfval, vfval, afval, Tmax);
% v_list = ...
% a_list = ...
% j_list = ...

figure(1)
%% Position
% subplot(2,4,1)
% plot(t_list, s_list)
% grid on
% xlabel('Time (s)','Interpreter','latex');
% ylabel('Position $(m)$','Interpreter','latex');
    
%% Velocity
% subplot(2,4,2)
% ...
    
%% Acceleration
% subplot(2,4,3)
% ...

%% Control
% subplot(2,4,4)
% ...
    
%% Velocity on position
% subplot(2,4,6)
% ...

%% Acceleration on position
% subplot(2,4,7)
% ...

%% Control on position
% subplot(2,4,8)
% ...