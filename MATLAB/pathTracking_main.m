%plots data from purepursuit.slx

clear; clc; close all;

%set noise parameters
minNoise = -0.0000001;
maxNoise = 0.000001;

% Used to optimize (trade studies)
%lookahead_vec = [0.05 0.1 0.2 0.3 0.5];
%linear_velo_vec = [0.01 0.05 0.1 0.2 0.3]; 
%angular_velo_vec = [0.25 0.5 0.75 1 1.25];

    %run the simulation
    lookahead = .2; %lookahead_vec(i);
    linear_velo = 0.1; %linear_velo_vec(i);
    angular_velo = 0.5; % angular_velo_vec(i);

    out_s = sim("stanleypathtracking.slx");
    out_v = sim("purepursuit_adv.slx");
    
    %get info from simout
    IC = out_v.IC;
    position_var = out_v.position.Data; % + [ones(h,1)*IC(1) ones(h,1)*IC(2)];
    position_s = out_s.position.Data; % + [ones(h,1)*IC(1) ones(h,1)*IC(2)];

    %plot position of Variable Lookahead PP
    figure(1); hold on
    plot(position_var(:,1),position_var(:,2),'LineWidth',1)
    hold on

    %plot position of Stanley
    plot(position_s(:,1),position_s(:,2),'LineWidth',1)
    hold on
    
    %find time to reach end
    %[~,finIndex] = min(vecnorm(position - [ones(height(position),1)*position(end,1) ones(height(position),1)*position(end,2)],2,2));
    %tend = out.tout(finIndex);
    
    %make pretty
    title(['Path Tracking Comparison, x_0 = ' num2str(IC(1)) ', y_0 = ' num2str(IC(2)) ', \theta_0 = ' num2str(IC(3))])
    axis equal
    xlabel('X (m)'); ylabel('Y (m)');

% Plot waypoints of desired trajectory

scatter(out_s.waypoints(:,1),out_s.waypoints(:,2),'Marker','o')
hold on

% Plot start and end and create legend
scatter(position_s(1,1),position_s(2,2),'marker','x','LineWidth',1,'MarkerEdgeColor','green')
scatter(position_s(end,1),position_s(end,2),'marker','x','LineWidth',1,'MarkerEdgeColor','red')
legend('PP w/ Variable Lookahead', 'Stanley', 'Waypoints', 'Start', 'end','location','southeast')

% Plot crosstrack error of all methods
figure(2);
time_vec_v = linspace(0,out_v.tout(end),length(out_v.cross_error.signals.values));
lpf = lowpass(out_v.cross_error.signals.values, 0.002, 0.1); hold on
plot(time_vec_v, lpf);

hold on 

time_vec_c = linspace(0,out_v.tout(end),length(out_s.cross_error.Data));
lpf = lowpass(out_s.cross_error.Data, 0.0005, 0.1); 
plot(time_vec_c, lpf);

title(['Cross Error vs Time, x_0 = ' num2str(IC(1)) ', y_0 = ' num2str(IC(2)) ', \theta_0 = ' num2str(IC(3))])
legend('PP w/ Variable Lookahead', 'Stanley', 'location','northeast');
ylim([0 0.25]);
xlabel('Cross Error (m)'); ylabel('Time (s)');

