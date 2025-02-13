%in this script we evaluate capabilities of the multicopter drone

close all
clear

g_grav = 9.8;
rho = 1.22;

Nprop = 8; %number of propellers
RPMvec = 1000:100:10000;
Diamvec = 0.2:0.02:1;

%required specs
Thovermin = 30 * 60; %minimal hover time

%set some empirical/guessed parameters
p2w_batt = 5.7; %max.power-to-weight (kW/kg) for the selected sort of battery (ex.: LiPo - 5.7, LiIon - 1.5)
e2w_batt = 450; %energy-to-weight (kJ/kg) for the selected sort of battery (ex.: LiPo - 450, LiIon - 650)
m_frame = 4; %kg, weight of bare frame (no batteries, payload, or motors+ESC's), can be made dependent on size and total weight
m_payload = 10;
p2w_motor = 3; %"cruise" power-to-weight (kW/kg) for the motors (ESC's included!) - this should be much less than max. ratio, because we should not operate close to max. power

%from datasheet (copied from prop_static_thrust.m):
Rref = 0.25;
P1ref = 305;
Fprop1ref = 2.2*g_grav;
omega1_ref = 3559 * 2*pi/60; %convert rpm to rad/sec

%assume Fprop = alp * rho * omega^2 * R^4, Pconsumed = beta * rho * omega^3 * R^5
alp_est = Fprop1ref / (rho * omega1_ref^2 * Rref^4);
beta_est = P1ref / (rho * omega1_ref^3 * Rref^5);

[RPMmatr, Diammatr] = meshgrid(RPMvec, Diamvec);
omegamatr = RPMmatr * 2*pi/60; %convert rpm to rad/sec
Rmatr = Diammatr/2;

Fprop_matr = Nprop * alp_est * rho * omegamatr.^2 .* Rmatr.^4;
Power_matr = Nprop * beta_est * rho * omegamatr.^3 .* Rmatr.^5;

m_motors_matr = 0.001*Power_matr / p2w_motor;
m_total_matr = Fprop_matr/g_grav;
m_batt_matr = m_total_matr - m_payload - m_frame - m_motors_matr;
m_batt_matr(m_batt_matr(:)<=0) = NaN;

%estimate hover time from energy capacity of the battery
t_hover_matr = m_batt_matr*e2w_batt ./ (0.001*Power_matr);

%check power condition: battery discharge rate should be high enough
enoughpow_indsTF = (0.001*Power_matr(:)) < (m_batt_matr(:)*p2w_batt);
t_hover_matr(~enoughpow_indsTF) = NaN;

%crop hover time below threshold
hoverenough_indsTF = (t_hover_matr(:)>Thovermin);
t_hover_crop = t_hover_matr;
m_total_crop = m_total_matr;
m_batt_crop = m_batt_matr;
Pmotor_crop = Power_matr/Nprop;
t_hover_crop(~hoverenough_indsTF) = NaN;
m_total_crop(~hoverenough_indsTF) = NaN;
m_batt_crop(~hoverenough_indsTF) = NaN;
Pmotor_crop(~hoverenough_indsTF) = NaN;


%estimate range depending on bank angle
Vpayload = 0.1; %qub.m, volume of the payload bay (we assume it is a cubic box) 
Cdrag = 1.2;

theta_vec = (5:0.5:30)*pi/180; %array of bank angles - we will determine an optimal one
Seff = Vpayload^(2/3); %effective area producing drag

thetaopt_matr = NaN(size(RPMmatr)); %to store optimal bank angle (for max. range)
speedopt_matr = NaN(size(RPMmatr)); %to store optimal speed
range_matr = NaN(size(RPMmatr)); %maximal range
for k = 1:length(RPMmatr(:))
    Fprop = Fprop_matr(k);
    Power = Power_matr(k);
    m_total_vec = Fprop*cos(theta_vec) / g_grav;
    m_batt_vec = m_total_vec - m_payload - m_frame - m_motors_matr(k);
    m_batt_vec(m_batt_vec<=0) = NaN;

    %calculate hor. prop. force, and according cruise speed
    Fhor_vec = Fprop*sin(theta_vec);
    speed_vec = sqrt(Fhor_vec / (0.5*rho*Cdrag*Seff));
    t_cruise_vec = m_batt_vec*e2w_batt / (0.001*Power);
    range_vec = t_cruise_vec.*speed_vec;

    %find optimal parameters of cruising
    [~, ind_opt] = max(range_vec);
    thetaopt_matr(k) = theta_vec(ind_opt);
    speedopt_matr(k) = speed_vec(ind_opt);
    range_matr(k) = range_vec(ind_opt);
end
thetaopt_matr(~hoverenough_indsTF) = NaN;
speedopt_matr(~hoverenough_indsTF) = NaN;
range_matr(~hoverenough_indsTF) = NaN;


%VISUALIZE
figure
subplot(121)
mesh(RPMmatr, Diammatr, log10(Fprop_matr/g_grav)); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('log10(total thrust, kg)')
subplot(122)
mesh(RPMmatr, Diammatr, log10(Power_matr/1000)); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('log10(total power, kW)')

figure
subplot(121)
mesh(RPMmatr, Diammatr, m_batt_matr); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('battery weight, kg')
subplot(122)
mesh(RPMmatr, Diammatr, t_hover_matr/60); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('hover time, min')

figure
subplot(221)
mesh(RPMmatr, Diammatr, t_hover_crop/60); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('hover time, min')
subplot(222)
mesh(RPMmatr, Diammatr, m_total_crop); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('total weight, kg')
subplot(223)
mesh(RPMmatr, Diammatr, Pmotor_crop/1000); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('single motor power, kW')
subplot(224)
mesh(RPMmatr, Diammatr, m_batt_crop); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('battery weight, kg')

%results of range optimization
figure
subplot(131)
mesh(RPMmatr, Diammatr, thetaopt_matr*180/pi); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('opt. bank angle, deg.')
subplot(132)
mesh(RPMmatr, Diammatr, speedopt_matr*3.6); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('opt. speed, km/h')
subplot(133)
mesh(RPMmatr, Diammatr, range_matr/1000); grid on
xlabel('RPM'); ylabel('prop.diam., m'); zlabel('range, km')