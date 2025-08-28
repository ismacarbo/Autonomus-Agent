%% Analisys
myd = logout;

%% Time Plot 
figure(1);
plot(myd.time,myd.req_vel,'--r','LineWidth',4);
hold on;
plot(myd.time,myd.vel,'-b','LineWidth',2);
plot(myd.time,myd.phase,':','Color','black','LineWidth',2);
plot(myd.time,myd.tmin,'-r','LineWidth',2);
plot(myd.time,myd.tmax,'-b','LineWidth',2);
plot(myd.time,myd.trt1,':','Color',[1,1,0],'LineWidth',2);
plot(myd.time,myd.trt2,':','Color',[1,0.5,0.5],'LineWidth',2);
plot(myd.time,myd.trt3,':','Color',[1,0,1],'LineWidth',2);
legend('Req vel', 'Vel','phase','Time Min','Time Max','t1','t2','t3');
xlabel('time (s)');
ylabel('vel (m/s)');
ylim([-1,25]);