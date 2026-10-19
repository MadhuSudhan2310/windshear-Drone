function build_drone_windshear_model()
%BUILD_DRONE_WINDSHEAR_MODEL  Builds Simulink model 'drone_windshear_pid':
%   planar quadrotor (x, z, pitch) + wind shear + cascaded PID control.
%   Altitude PID -> thrust;  position PID -> pitch command -> attitude PID -> torque.
%   Requires MATLAB + Simulink only.

evalin('base','drone_windshear_params');
mdl = 'drone_windshear_pid';
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl); open_system(mdl);
B = @(name,lib,x,y,varargin) addb(mdl,name,['simulink/' lib],x,y,varargin{:});

%% ---------------- Wind generator ----------------
B('Clock','Sources/Clock',30,40);
B('Bias_t0','Math Operations/Bias',100,40,'Bias','-t0');
B('Gain_1overT','Math Operations/Gain',170,40,'Gain','1/Tshear');
B('Sat_0_1','Discontinuities/Saturation',240,40,'UpperLimit','1','LowerLimit','0');
B('Fcn_Uenv','User-Defined Functions/Fcn',320,20,'Expr','Ug*(1-cos(pi*u[1]))/2');
B('Mux_Uz','Signal Routing/Mux',410,20,'Inputs','2');
B('Fcn_uw','User-Defined Functions/Fcn',480,20,'Expr','u[1]*((z0+u[2])/z0)^alphas');
B('Fcn_ww','User-Defined Functions/Fcn',320,80,'Expr','-Aw*sin(pi*u[1])^2');
B('ShearOn_uw','Math Operations/Gain',570,20,'Gain','ShearOn');
B('ShearOn_ww','Math Operations/Gain',570,80,'Gain','ShearOn');
ln(mdl,'Clock',1,'Bias_t0',1); ln(mdl,'Bias_t0',1,'Gain_1overT',1);
ln(mdl,'Gain_1overT',1,'Sat_0_1',1);
ln(mdl,'Sat_0_1',1,'Fcn_Uenv',1); ln(mdl,'Sat_0_1',1,'Fcn_ww',1);
ln(mdl,'Fcn_Uenv',1,'Mux_Uz',1);  ln(mdl,'Mux_Uz',1,'Fcn_uw',1);
ln(mdl,'Fcn_uw',1,'ShearOn_uw',1); ln(mdl,'Fcn_ww',1,'ShearOn_ww',1);

%% ---------------- Quadrotor dynamics ----------------
% x axis
B('Mux_x','Signal Routing/Mux',700,20,'Inputs','4');
B('Fcn_xdd','User-Defined Functions/Fcn',770,20,'Expr','(u[1]*sin(u[2])-cdx*(u[3]-u[4]))/m');
B('Int_xd','Continuous/Integrator',860,20);
B('Int_x','Continuous/Integrator',930,20);
% z axis
B('Mux_z','Signal Routing/Mux',700,120,'Inputs','4');
B('Fcn_zdd','User-Defined Functions/Fcn',770,120,'Expr','(u[1]*cos(u[2])-m*g-cdz*(u[3]-u[4]))/m');
B('Int_zd','Continuous/Integrator',860,120);
B('Int_z','Continuous/Integrator',930,120);
% pitch axis
B('Mux_q','Signal Routing/Mux',700,220,'Inputs','2');
B('Fcn_qd','User-Defined Functions/Fcn',770,220,'Expr','(u[1]-kq*u[2])/J');
B('Int_q','Continuous/Integrator',860,220);
B('Int_theta','Continuous/Integrator',930,220);

ln(mdl,'Mux_x',1,'Fcn_xdd',1); ln(mdl,'Fcn_xdd',1,'Int_xd',1); ln(mdl,'Int_xd',1,'Int_x',1);
ln(mdl,'Int_xd',1,'Mux_x',3);  ln(mdl,'ShearOn_uw',1,'Mux_x',4);
ln(mdl,'Mux_z',1,'Fcn_zdd',1); ln(mdl,'Fcn_zdd',1,'Int_zd',1); ln(mdl,'Int_zd',1,'Int_z',1);
ln(mdl,'Int_zd',1,'Mux_z',3);  ln(mdl,'ShearOn_ww',1,'Mux_z',4);
ln(mdl,'Mux_q',1,'Fcn_qd',1);  ln(mdl,'Fcn_qd',1,'Int_q',1);  ln(mdl,'Int_q',1,'Int_theta',1);
ln(mdl,'Int_q',1,'Mux_q',2);
ln(mdl,'Int_theta',1,'Mux_x',2); ln(mdl,'Int_theta',1,'Mux_z',2);
ln(mdl,'Int_z',1,'Mux_Uz',2);    % wind height profile uses altitude

%% ---------------- Motors (first-order lag) ----------------
B('Motor_thrust','Continuous/Transfer Fcn',560,320,'Numerator','1','Denominator','[tau_m 1]');
B('Bias_hover','Math Operations/Bias',640,320,'Bias','m*g');
B('Motor_torque','Continuous/Transfer Fcn',560,400,'Numerator','1','Denominator','[tau_m 1]');
ln(mdl,'Motor_thrust',1,'Bias_hover',1);
ln(mdl,'Bias_hover',1,'Mux_x',1); ln(mdl,'Bias_hover',1,'Mux_z',1);
ln(mdl,'Motor_torque',1,'Mux_q',1);

%% ---------------- Controllers ----------------
% Altitude
B('z_cmd','Sources/Constant',30,320,'Value','zcmd');
B('Sum_z_err','Math Operations/Sum',100,320,'Inputs','+-');
B('PID_altitude','Continuous/PID Controller',170,310,'P','z_Kp','I','z_Ki','D','z_Kd','N','z_N', ...
  'LimitOutput','on','UpperSaturationLimit','m*g','LowerSaturationLimit','-m*g','AntiWindupMode','clamping');
ln(mdl,'z_cmd',1,'Sum_z_err',1); ln(mdl,'Int_z',1,'Sum_z_err',2);
ln(mdl,'Sum_z_err',1,'PID_altitude',1); ln(mdl,'PID_altitude',1,'Motor_thrust',1);
% Position -> pitch command -> attitude
B('x_cmd','Sources/Constant',30,400,'Value','xcmd');
B('Sum_x_err','Math Operations/Sum',100,400,'Inputs','+-');
B('PID_position','Continuous/PID Controller',170,390,'P','x_Kp','I','x_Ki','D','x_Kd','N','x_N', ...
  'LimitOutput','on','UpperSaturationLimit','theta_max','LowerSaturationLimit','-theta_max','AntiWindupMode','clamping');
B('Sum_th_err','Math Operations/Sum',290,400,'Inputs','+-');
B('PID_attitude','Continuous/PID Controller',360,390,'P','th_Kp','I','th_Ki','D','th_Kd','N','th_N', ...
  'LimitOutput','on','UpperSaturationLimit','taumax','LowerSaturationLimit','-taumax','AntiWindupMode','clamping');
ln(mdl,'x_cmd',1,'Sum_x_err',1); ln(mdl,'Int_x',1,'Sum_x_err',2);
ln(mdl,'Sum_x_err',1,'PID_position',1); ln(mdl,'PID_position',1,'Sum_th_err',1);
ln(mdl,'Int_theta',1,'Sum_th_err',2); ln(mdl,'Sum_th_err',1,'PID_attitude',1);
ln(mdl,'PID_attitude',1,'Motor_torque',1);

%% ---------------- Logging and scope ----------------
logs = {'x_log','Int_x'; 'z_log','Int_z'; 'th_log','Int_theta'; ...
        'T_log','Bias_hover'; 'uw_log','ShearOn_uw'; 'ww_log','ShearOn_ww'};
for i = 1:size(logs,1)
    nm = ['Log_' logs{i,1}];
    B(nm,'Sinks/To Workspace',1050,20+45*i,'VariableName',logs{i,1},'SaveFormat','Timeseries');
    ln(mdl,logs{i,2},1,nm,1);
end
B('Scope','Sinks/Scope',1050,340,'NumInputPorts','4');
ln(mdl,'Int_x',1,'Scope',1); ln(mdl,'Int_z',1,'Scope',2);
ln(mdl,'Int_theta',1,'Scope',3); ln(mdl,'ShearOn_uw',1,'Scope',4);

set_param(mdl,'StopTime','tstop','Solver','ode45','MaxStep','0.01');
save_system(mdl);
fprintf('Built and saved model "%s". Press Run, or call run_drone_windshear_study.\n',mdl);
end

function p = addb(mdl,name,src,x,y,varargin)
p = [mdl '/' name];
add_block(src,p,'Position',[x y x+60 y+30]);
for k = 1:2:numel(varargin), set_param(p,varargin{k},varargin{k+1}); end
end

function ln(mdl,a,pa,b,pb)
add_line(mdl,[a '/' num2str(pa)],[b '/' num2str(pb)],'autorouting','on');
end
