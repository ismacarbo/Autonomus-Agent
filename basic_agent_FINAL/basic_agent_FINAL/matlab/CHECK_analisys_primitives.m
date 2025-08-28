%% Inspect primitive solutions

% Definition of the functions
coeffs_sfun = @(coeffs,t) 1*coeffs(2).*t+1/2*coeffs(3).*t.^2+1/6*coeffs(4).*t.^3+1/24*coeffs(5).*t.^4+1/120*coeffs(6).*t.^5;
coeffs_vfun = @(coeffs,t) coeffs(2)+1*coeffs(3).*t+1/2*coeffs(4).*t.^2+1/6*coeffs(5).*t.^3+1/24*coeffs(6).*t.^4;
coeffs_afun = @(coeffs,t) coeffs(3)+1*coeffs(4).*t+1/2*coeffs(5).*t.^2+1/6*coeffs(6).*t.^3;
coeffs_jfun = @(coeffs,t) coeffs(4)+1*coeffs(5).*t+1/2*coeffs(6).*t.^2;

% Initional state
v0val = 10;
a0val = 5;

% Final position
sfval = 20;

% PASS
vfval_pass = 25;
afval = 0;
Topt_pass  = final_opt_time_pass(v0val,a0val,sfval,vfval_pass);
t_list_pass = linspace(0,Topt_pass,100);
assert(25,final_opt_vel_pass(v0val,a0val,sfval,Topt_pass));

% STOP
vfval_stop = 0;
Topt_stop = final_opt_time_stop(v0val,a0val,sfval);
t_list_stop = linspace(0,Topt_stop,100);




% the functions work both for vector (t_list) or number (t) 
% PASS primitives
s_list_pass = s_opt(t_list_pass, v0val, a0val, sfval, vfval_pass, afval, Topt_pass);
v_list_pass = v_opt(t_list_pass, v0val, a0val, sfval, vfval_pass, afval, Topt_pass);
a_list_pass = a_opt(t_list_pass, v0val, a0val, sfval, vfval_pass, afval, Topt_pass);
j_list_pass = j_opt(t_list_pass, v0val, a0val, sfval, vfval_pass, afval, Topt_pass);
coeffsval_pass = coef_list(v0val, a0val, sfval, vfval_pass, afval, Topt_pass);
% STOP primitives
s_list_stop = s_opt(t_list_stop, v0val, a0val, sfval, vfval_stop, afval, Topt_stop);
v_list_stop = v_opt(t_list_stop, v0val, a0val, sfval, vfval_stop, afval, Topt_stop);
a_list_stop = a_opt(t_list_stop, v0val, a0val, sfval, vfval_stop, afval, Topt_stop);
j_list_stop = j_opt(t_list_stop, v0val, a0val, sfval, vfval_stop, afval, Topt_stop);
coeffsval_stop = coef_list(v0val, a0val, sfval, vfval_stop, afval, Topt_stop);


figure(1)
% position
subplot(1,4,1);
plot(t_list_pass, coeffs_sfun(coeffsval_pass,t_list_pass));
hold on;
plot(t_list_pass, s_list_pass);
plot(t_list_stop, coeffs_sfun(coeffsval_stop,t_list_stop));
plot(t_list_stop, s_list_stop);
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Position $(m)$','Interpreter','latex');
legend('Pass Position with coef','Pass Position s\_opt', ...
       'Stop Position with coef','Stop Position s\_opt', ...
       'Interpreter','latex');

% velocity
subplot(1,4,2);
plot(t_list_pass, coeffs_vfun(coeffsval_pass,t_list_pass));
hold on;
plot(t_list_pass, v_list_pass);
plot(t_list_stop, coeffs_vfun(coeffsval_stop,t_list_stop));
plot(t_list_stop, v_list_stop);
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Velocity $(\frac{m}{s})$','Interpreter','latex');
legend('Pass Velocity coef','Pass Velocity v\_opt', ...
    'Stop Velocity coef','Stop Velocity v\_opt',...
    'Interpreter','latex');

% acceleration
subplot(1,4,3);
plot(t_list_pass, coeffs_afun(coeffsval_pass,t_list_pass));
hold on;
plot(t_list_pass, a_list_pass);
plot(t_list_stop, coeffs_afun(coeffsval_stop,t_list_stop));
plot(t_list_stop, a_list_stop);
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Acceleration $(\frac{m}{s^2})$','Interpreter','latex');
legend('Pass Acceleration coef','Pass Acceleration a\_opt',...
    'Stop Acceleration coef','Stop Acceleration a\_opt',...
    'Interpreter','latex');

% jerk
subplot(1,4,4);
plot(t_list_pass, coeffs_jfun(coeffsval_pass,t_list_pass));
hold on;
plot(t_list_pass, j_list_pass);
plot(t_list_stop, coeffs_jfun(coeffsval_stop,t_list_stop));
plot(t_list_stop, j_list_stop);
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Jerk $(\frac{m}{s^3})$','Interpreter','latex');
legend('Pass Jerk coef','Pass Jerk j\_opt',...
    'Stop Jerk coef','Stop Jerk j\_opt',...
    'Interpreter','latex');

%%
figure(2);
% velocity
subplot(1,3,1);
plot(s_list_pass, v_list_pass);
hold on;
plot(s_list_stop, v_list_stop);
grid on;
xlabel('Postion (m)','Interpreter','latex');
ylabel('Velocity $(\frac{m}{s})$','Interpreter','latex');
legend('Pass Velocity', 'Stop Velocity',...
    'Interpreter','latex');

% acceleration
subplot(1,3,2);
plot(s_list_pass, a_list_pass);
hold on;
plot(s_list_stop, a_list_stop);
grid on;
xlabel('Postion (m)','Interpreter','latex');
ylabel('Acceleration $(\frac{m}{s^2})$','Interpreter','latex');
legend('Pass Acceleration', 'Stop Acceleration',...
    'Interpreter','latex');

% jerk
subplot(1,3,3);
plot(s_list_pass, j_list_pass);
hold on;
plot(s_list_stop, j_list_stop);
grid on;
xlabel('Time (s)','Interpreter','latex');
ylabel('Jerk $(\frac{m}{s^3})$','Interpreter','latex');
legend('Pass Jerk','Stop Jerk',...
    'Interpreter','latex');
