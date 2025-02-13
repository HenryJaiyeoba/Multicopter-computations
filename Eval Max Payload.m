%in this script we evaluate max. payload (assuming that motors are
%redundantly powerful) and range with this payload
%NOTE: this is typically to be run after we select parameters from
%multicopt_calc.m

clear

g_grav = 9.8;
rho = 1.22;

m_battery = 4*2.5; %total battery weight
m_motors = 8*0.6; %with ESC's
m_frame = 4; %kg, weight of bare frame (no batteries, payload, or motors+ESC's), can be made dependent on size and total weight
Rprop = 0.35;
Nprop = 8; %number of propellers

power_rated_kW = 1.1; %kW, RATED power of a single motor

p2w_batt = 1.3; %max.power-to-weight (kW/kg) for the selected sort of battery (ex.: LiPo - 5.7, LiIon - 1.5, solid state - 1.3)
e2w_batt = 900; %energy-to-weight (kJ/kg) for the selected sort of battery (ex.: LiPo - 450, LiIon - 650, solid state - 900)

RPMvec = 2000:50:4000;

%PROPELLER DATA, should repeat multicopt_calc.m
%from datasheet (copied from prop_static_thrust.m):
Rref = 0.25;
P1ref = 305;
Fprop1ref = 2.2*g_grav;
omega1_ref = 3559 * 2*pi/60; %convert rpm to rad/sec
%assume Fprop = alp * rho * omega^2 * R^4, Pconsumed = beta * rho * omega^3 * R^5
alp_est = Fprop1ref / (rho * omega1_ref^2 * Rref^4);
beta_est = P1ref / (rho * omega1_ref^3 * Rref^5);

%CALCULATIONS
power_batt_kW = m_battery*p2w_batt;

omega_vec = RPMvec * 2*pi/60; %convert rpm to rad/sec

Fprop_vec = Nprop * alp_est * rho * omega_vec.^2 .* Rprop.^4;
Power_vec = Nprop * beta_est * rho * omega_vec.^3 .* Rprop.^5;
m_payload_vec = Fprop_vec/g_grav - (m_battery + m_motors + m_frame);
t_hover = 1000*e2w_batt*m_battery ./ Power_vec;

pow_normalTF = ( Power_vec < 1000*min(power_batt_kW, power_rated_kW*Nprop) );
m_payload_vec(~pow_normalTF) = NaN;
Fprop_vec(~pow_normalTF) = NaN;
Power_vec(~pow_normalTF) = NaN;
t_hover(~pow_normalTF) = NaN;

%estimate range depending on bank angle
Vpayload = 0.1; %qub.m, volume of the payload bay (we assume it is a cubic box) 
Cdrag = 1.2;

theta_vec = (5:0.5:20)*pi/180; %array of bank angles - we will determine an optimal one
Seff = Vpayload^(2/3); %effective area producing drag

thetaopt_arr = NaN(size(RPMvec)); %to store optimal bank angle (for max. range)
speedopt_arr = NaN(size(RPMvec)); %to store optimal speed
range_arr = NaN(size(RPMvec)); %maximal range
m_payload_arr = NaN(size(RPMvec));
for k = 1:length(RPMvec)
    Fprop = Fprop_vec(k);
    Power = Power_vec(k);
    m_total_vec = Fprop*cos(theta_vec) / g_grav;
    m_payload_curvec = m_total_vec - (m_battery + m_motors + m_frame);

    %calculate hor. prop. force, and according cruise speed
    Fhor_vec = Fprop*sin(theta_vec);
    speed_vec = sqrt(Fhor_vec / (0.5*rho*Cdrag*Seff));
    t_cruise_vec = m_battery*e2w_batt / (0.001*Power);
    range_vec = t_cruise_vec.*speed_vec;

    %find optimal parameters of cruising
    [~, ind_opt] = max(range_vec);
    thetaopt_arr(k) = theta_vec(ind_opt);
    speedopt_arr(k) = speed_vec(ind_opt);
    range_arr(k) = range_vec(ind_opt);
    m_payload_arr(k) = m_payload_curvec(ind_opt);
end
thetaopt_arr(~pow_normalTF) = NaN;



%VISUALIZE
figure
subplot(311)
plot(RPMvec, m_payload_vec, '.-', RPMvec, Fprop_vec/g_grav, '.-'); grid on
legend('payload', 'total')
xlabel('RPM'); ylabel('weight, kg')
subplot(312)
plot(RPMvec, 0.001*Power_vec/Nprop, '.-'); grid on
xlabel('RPM'); ylabel('single motor power, kW')
subplot(313)
plot(RPMvec, t_hover/60, '.-'); grid on
xlabel('RPM'); ylabel('hover time, min')

figure
subplot(221)
plot(RPMvec, thetaopt_arr*180/pi, '.-'); grid on
xlabel('RPM'); ylabel('opt. bank angle, deg.')
subplot(223)
plot(RPMvec, speedopt_arr*3.6, '.-'); grid on
xlabel('RPM'); ylabel('opt. speed, km/h')
subplot(222)
plot(RPMvec, range_arr/1000, '.-'); grid on
xlabel('RPM'); ylabel('range, km')
subplot(224)
plot(RPMvec, m_payload_arr, '.-'); grid on
xlabel('RPM'); ylabel('payload, kg')



