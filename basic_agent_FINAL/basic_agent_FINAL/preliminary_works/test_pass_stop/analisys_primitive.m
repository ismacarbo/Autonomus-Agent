%% Analisys
myd = logout;

%% Time Plot
figure(2);
plot(myd.time,myd.req_vel,'--r','LineWidth',4);
hold on;
plot(myd.time,myd.vel,'-b','LineWidth',2);
legend('Req vel', 'Vel');
xlabel('time (s)');
ylabel('vel (m/s)');

% Functions
coeffs_sfun = @(coeffs,t) 1*coeffs(2).*t+1/2*coeffs(3).*t.^2+1/6*coeffs(4).*t.^3+1/24*coeffs(5).*t.^4+1/120*coeffs(6).*t.^5;
coeffs_vfun = @(coeffs,t) coeffs(2)+1*coeffs(3).*t+1/2*coeffs(4).*t.^2+1/6*coeffs(5).*t.^3+1/24*coeffs(6).*t.^4;

% Test primitives
for ind = 1:10:length(myd.time)
    tt = myd.time(ind);
    Tfinal = myd.T(ind);
    time_primitive_idx = find(myd.time >= tt & myd.time <= Tfinal + tt);
    times_primitive_vect = myd.time(time_primitive_idx);
    vel_primitive_vect = coeffs_vfun([myd.C0(ind),myd.C1(ind),myd.C2(ind),myd.C3(ind),myd.C4(ind),myd.C5(ind)],times_primitive_vect-tt);
    plot(times_primitive_vect,vel_primitive_vect,'-g','LineWidth',0.5,'HandleVisibility','off');
end
ylim([-1,50]);

%% Space Plot
figure(3);
plot(myd.dist,myd.req_vel,'--r','LineWidth',3);
hold on;
plot(myd.dist,myd.vel,'-b','LineWidth',3);
legend('Req vel', 'Vel');
xlabel('space (m)');
ylabel('vel (m/s)');

% Test primitives
for ind = 1:10:length(myd.time)
    tt = myd.time(ind);
    Tfinal = myd.T(ind);
    time_primitive_idx = find(myd.time >= tt & myd.time <= Tfinal + tt);
    times_primitive_vect = myd.time(time_primitive_idx);
    CC = [myd.C0(ind),myd.C1(ind),myd.C2(ind),myd.C3(ind),myd.C4(ind),myd.C5(ind)];
    vel_primitive_vect = coeffs_vfun(CC,times_primitive_vect-tt);
    pos_primitive_vect = coeffs_sfun(CC,times_primitive_vect-tt);
    plot(pos_primitive_vect+myd.dist(ind),vel_primitive_vect,'-g','LineWidth',0.5,'HandleVisibility','off');
end
ylim([-1,22]);