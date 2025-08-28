%% Analisys
myd = logout;

%% Time Plot
figure(1);
plot(myd.time,myd.req_vel,'--r','LineWidth',4);
hold on;
plot(myd.time,myd.vel,'-b','LineWidth',2);
legend('Rq Vel','Vel');
ylabel('vel (m/s)')
xlabel('t (s)')

%% Space Plot
figure(2);
plot(myd.dist,myd.req_vel,'--r','LineWidth',3);
hold on;
plot(myd.dist,myd.vel,'-b','LineWidth',3);
legend('Rq Vel','Vel');
ylabel('vel (m/s)');
xlabel('s (m)');

