%% Analisys
myd = logout;

%% Time Plot
%% Plot acc vs req_acc
figure(1);
plot(myd.time,myd.req_acc,'--r','LineWidth',4);
hold on;
plot(myd.time,myd.acc,'-b','LineWidth',2);
legend('Rq Acc','Acc');
ylabel('vel (m/s^2)');
xlabel('t (s)');
ylim([-10,10])

%% Plot Vel vs req_vel
figure(2);
plot(myd.time,myd.req_vel,'--r','LineWidth',4);
hold on;
plot(myd.time,myd.vel,'-b','LineWidth',2);
legend('Rq Vel','Vel');
ylabel('vel (m/s)');
xlabel('t (s)');
ylim([0,30])

%% Space Plot
%% Plot acc vs req_acc
figure(3);
plot(myd.dist,myd.req_acc,'--r','LineWidth',3);
hold on;
plot(myd.dist,myd.acc,'-b','LineWidth',3);
legend('Rq Acc','Acc');
ylabel('vel (m/s^2)');
xlabel('s (m)');
ylim([-10,10])

%% Plot Vel vs req_vel
figure(4);
plot(myd.dist,myd.req_vel,'--r','LineWidth',3);
hold on;
plot(myd.dist,myd.vel,'-b','LineWidth',3);
legend('Rq Vel','Vel');
ylabel('vel (m/s)');
xlabel('s (m)');
ylim([0,30])


