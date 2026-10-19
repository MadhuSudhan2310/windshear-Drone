function results = run_drone_windshear_study()
%RUN_DRONE_WINDSHEAR_STUDY  How wind shear degrades quadrotor PID performance.
%   Study A: calm vs shear, PID vs PD-only (no integral action)
%   Study B: shear intensity sweep (Ug = 0..20 m/s)
%   Study C: position-loop gain sets (drift vs overshoot trade-off)

mdl = 'drone_windshear_pid';
if ~bdIsLoaded(mdl)
    if exist([mdl '.slx'],'file'), load_system(mdl); else, build_drone_windshear_model; end
end
evalin('base','drone_windshear_params');
get = @(n) evalin('base',n);
put = @(n,v) assignin('base',n,v);
xKi0 = get('x_Ki'); zKi0 = get('z_Ki');
setPD  = @() setKi(0,0);          % remove integral action
setPID = @() setKi(xKi0,zKi0);    % restore integral action
peak = @(ts) max(abs(ts.Data));

%% ---------- Study A ----------
cases = {'Calm air',0,true; 'Shear, PID',1,true; 'Shear, PD only',1,false};
fig1 = figure('Name','Study A','Position',[100 100 900 750]);
for i = 1:size(cases,1)
    put('ShearOn',cases{i,2});
    if cases{i,3}, setPID(); else, setPD(); end
    o = sim(mdl,'StopTime','tstop');
    subplot(4,1,1); hold on; plot(o.x_log.Time,o.x_log.Data,'LineWidth',1.4); ylabel('x [m]');
    subplot(4,1,2); hold on; plot(o.z_log.Time,o.z_log.Data,'LineWidth',1.4); ylabel('z dev [m]');
    subplot(4,1,3); hold on; plot(o.th_log.Time,rad2deg(o.th_log.Data),'LineWidth',1.4); ylabel('\theta [deg]');
    subplot(4,1,4); hold on; plot(o.T_log.Time,o.T_log.Data,'LineWidth',1.4); ylabel('Thrust [N]'); xlabel('t [s]');
    R(i).name = cases{i,1}; %#ok<AGROW>
    R(i).x_peak_m = peak(o.x_log);  R(i).z_peak_m = peak(o.z_log);
    R(i).theta_peak_deg = rad2deg(peak(o.th_log)); R(i).thrust_peak_N = max(o.T_log.Data);
    R(i).x_final_m = o.x_log.Data(end);
end
subplot(4,1,1); legend(cases(:,1)','Location','best'); grid on;
for k=2:4, subplot(4,1,k); grid on; end
sgtitle('Study A: wind shear effect on drone position control');
setPID(); put('ShearOn',1);
results.A = struct2table(R); disp(results.A);

%% ---------- Study B ----------
Uv = 0:4:20; n = numel(Uv);
xp = zeros(n,2); zp = xp; thp = xp; xf = xp;       % columns: PID, PD
for i = 1:n
    put('Ug',Uv(i)); put('Aw',Uv(i)*3/8);
    for c = 1:2
        if c==1, setPID(); else, setPD(); end
        o = sim(mdl,'StopTime','tstop');
        xp(i,c) = peak(o.x_log); zp(i,c) = peak(o.z_log);
        thp(i,c) = rad2deg(peak(o.th_log)); xf(i,c) = o.x_log.Data(end);
    end
end
setPID(); put('Ug',8); put('Aw',3);
figure('Name','Study B','Position',[150 150 700 450]);
plot(Uv,xp(:,1),'o-',Uv,xp(:,2),'s-','LineWidth',1.5); grid on;
xlabel('Final wind speed U_g [m/s]'); ylabel('Peak position error [m]');
legend('PID','PD only','Location','northwest'); title('Study B: position error vs shear intensity');
results.B = table(Uv(:),xp(:,1),xp(:,2),xf(:,2),thp(:,1),zp(:,1), ...
  'VariableNames',{'Ug_mps','x_peak_PID_m','x_peak_PD_m','x_final_PD_m','theta_peak_deg','z_peak_PID_m'});
disp(results.B); writetable(results.B,'drone_windshear_results.csv');

%% ---------- Study C ----------
sets = [0.10 0.01 0.15; 0.23 0.03 0.245; 0.50 0.08 0.40; 1.0 0.2 0.6];
x0 = [get('x_Kp') get('x_Ki') get('x_Kd')];
C = zeros(size(sets,1),3);
for i = 1:size(sets,1)
    put('x_Kp',sets(i,1)); put('x_Ki',sets(i,2)); put('x_Kd',sets(i,3));
    o = sim(mdl,'StopTime','tstop');
    C(i,:) = [peak(o.x_log), rad2deg(peak(o.th_log)), max(o.T_log.Data)];
end
put('x_Kp',x0(1)); put('x_Ki',x0(2)); put('x_Kd',x0(3));
results.C = table(sets(:,1),sets(:,2),sets(:,3),C(:,1),C(:,2),C(:,3), ...
  'VariableNames',{'x_Kp','x_Ki','x_Kd','x_peak_m','theta_peak_deg','thrust_peak_N'});
disp(results.C);
saveas(fig1,'studyA_drone_shear_response.png');
end

function setKi(kx,kz)
assignin('base','x_Ki',kx); assignin('base','z_Ki',kz);
end
