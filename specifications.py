import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
g_grav = 9.8
rho = 1.22

# Drone parameters
Nprop = 8  # number of propellers
RPMvec = np.arange(1000, 10001, 100)
Diamvec = np.arange(0.2, 1.01, 0.02)

# Required specs
Thovermin = 30 * 60  # minimal hover time

# Empirical/guessed parameters
p2w_batt = 5.7  # max power-to-weight (kW/kg) for selected battery type
e2w_batt = 450  # energy-to-weight (kJ/kg) for selected battery type
m_frame = 4  # kg, weight of bare frame
m_payload = 10
p2w_motor = 3  # "cruise" power-to-weight (kW/kg) for motors

# From datasheet
Rref = 0.25
P1ref = 305
Fprop1ref = 2.2 * g_grav
omega1_ref = 3559 * 2 * np.pi / 60  # convert rpm to rad/sec

# Calculate coefficients
alp_est = Fprop1ref / (rho * omega1_ref**2 * Rref**4)
beta_est = P1ref / (rho * omega1_ref**3 * Rref**5)

# Create meshgrid
RPMmatr, Diammatr = np.meshgrid(RPMvec, Diamvec)
omegamatr = RPMmatr * 2 * np.pi / 60  # convert rpm to rad/sec
Rmatr = Diammatr / 2

# Calculate matrices
Fprop_matr = Nprop * alp_est * rho * omegamatr**2 * Rmatr**4
Power_matr = Nprop * beta_est * rho * omegamatr**3 * Rmatr**5

m_motors_matr = 0.001 * Power_matr / p2w_motor
m_total_matr = Fprop_matr / g_grav
m_batt_matr = m_total_matr - m_payload - m_frame - m_motors_matr
m_batt_matr[m_batt_matr <= 0] = np.nan

# Estimate hover time
t_hover_matr = m_batt_matr * e2w_batt / (0.001 * Power_matr)

# Check power condition
enoughpow_indsTF = (0.001 * Power_matr.flatten()) < (m_batt_matr.flatten() * p2w_batt)
t_hover_matr_flat = t_hover_matr.flatten()
t_hover_matr_flat[~enoughpow_indsTF] = np.nan
t_hover_matr = t_hover_matr_flat.reshape(t_hover_matr.shape)

# Crop hover time below threshold
hoverenough_indsTF = t_hover_matr.flatten() > Thovermin
t_hover_crop = t_hover_matr.copy()
m_total_crop = m_total_matr.copy()
m_batt_crop = m_batt_matr.copy()
Pmotor_crop = Power_matr / Nprop

# Apply the mask to cropped arrays
mask = ~hoverenough_indsTF.reshape(t_hover_matr.shape)
t_hover_crop[mask] = np.nan
m_total_crop[mask] = np.nan
m_batt_crop[mask] = np.nan
Pmotor_crop[mask] = np.nan

# Range estimation parameters
Vpayload = 0.1  # cubic meters, volume of payload bay
Cdrag = 1.2
theta_vec = np.deg2rad(np.arange(5, 30.5, 0.5))  # array of bank angles
Seff = Vpayload**(2/3)  # effective area producing drag

# Initialize optimization matrices
thetaopt_matr = np.full(RPMmatr.shape, np.nan)
speedopt_matr = np.full(RPMmatr.shape, np.nan)
range_matr = np.full(RPMmatr.shape, np.nan)

# Optimization loop
for i in range(RPMmatr.shape[0]):
    for j in range(RPMmatr.shape[1]):
        Fprop = Fprop_matr[i, j]
        Power = Power_matr[i, j]
        
        m_total_vec = Fprop * np.cos(theta_vec) / g_grav
        m_batt_vec = m_total_vec - m_payload - m_frame - m_motors_matr[i, j]
        m_batt_vec[m_batt_vec <= 0] = np.nan
        
        # Calculate horizontal propulsive force and cruise speed
        Fhor_vec = Fprop * np.sin(theta_vec)
        speed_vec = np.sqrt(Fhor_vec / (0.5 * rho * Cdrag * Seff))
        t_cruise_vec = m_batt_vec * e2w_batt / (0.001 * Power)
        range_vec = t_cruise_vec * speed_vec
        
        # Find optimal parameters
        if not np.all(np.isnan(range_vec)):
            ind_opt = np.nanargmax(range_vec)
            thetaopt_matr[i, j] = theta_vec[ind_opt]
            speedopt_matr[i, j] = speed_vec[ind_opt]
            range_matr[i, j] = range_vec[ind_opt]

# Apply hover threshold mask to optimization results
thetaopt_matr[mask] = np.nan
speedopt_matr[mask] = np.nan
range_matr[mask] = np.nan

# modified DiameterVec
Diamvec_modified = np.arange(0.7, 1.01, 0.02)  # Modified diameter range
RPMvec_modifdied = np.arange(1000, 3001, 100)
RPMmatr_mod, Diammatr_mod = np.meshgrid(RPMvec_modifdied, Diamvec_modified)
omegamatr_mod = RPMmatr_mod * 2 * np.pi / 60
Rmatr_mod = Diammatr_mod / 2

# Recalculate matrices for modified diameter range
Fprop_matr_mod = Nprop * alp_est * rho * omegamatr_mod**2 * Rmatr_mod**4
Power_matr_mod = Nprop * beta_est * rho * omegamatr_mod**3 * Rmatr_mod**5

m_motors_matr_mod = 0.001 * Power_matr_mod / p2w_motor
m_total_matr_mod = Fprop_matr_mod / g_grav
m_batt_matr_mod = m_total_matr_mod - m_payload - m_frame - m_motors_matr_mod
m_batt_matr_mod[m_batt_matr_mod <= 0] = np.nan

# Initialize optimization matrices for modified range
thetaopt_matr_mod = np.full(RPMmatr_mod.shape, np.nan)
speedopt_matr_mod = np.full(RPMmatr_mod.shape, np.nan)
range_matr_mod = np.full(RPMmatr_mod.shape, np.nan)

# Optimization loop for modified range
for i in range(RPMmatr_mod.shape[0]):
    for j in range(RPMmatr_mod.shape[1]):
        Fprop = Fprop_matr_mod[i, j]
        Power = Power_matr_mod[i, j]
        
        m_total_vec = Fprop * np.cos(theta_vec) / g_grav
        m_batt_vec = m_total_vec - m_payload - m_frame - m_motors_matr_mod[i, j]
        m_batt_vec[m_batt_vec <= 0] = np.nan
        
        Fhor_vec = Fprop * np.sin(theta_vec)
        speed_vec = np.sqrt(Fhor_vec / (0.5 * rho * Cdrag * Seff))
        t_cruise_vec = m_batt_vec * e2w_batt / (0.001 * Power)
        range_vec = t_cruise_vec * speed_vec
        
        if not np.all(np.isnan(range_vec)):
            ind_opt = np.nanargmax(range_vec)
            thetaopt_matr_mod[i, j] = theta_vec[ind_opt]
            speedopt_matr_mod[i, j] = speed_vec[ind_opt]
            range_matr_mod[i, j] = range_vec[ind_opt]

# Plotting functions
def create_3d_plot(x, y, z, xlabel, ylabel, zlabel):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    surf = ax.plot_surface(x, y, z, cmap='viridis')
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_zlabel(zlabel)
    plt.colorbar(surf)
    return fig

# Create plots
plt.figure(figsize=(15, 6))
ax1 = plt.subplot(121, projection='3d')
surf1 = ax1.plot_surface(RPMmatr, Diammatr, np.log10(Fprop_matr/g_grav), cmap='viridis')
ax1.set_xlabel('RPM')
ax1.set_ylabel('prop.diam., m')
ax1.set_zlabel('log10(total thrust, kg)')
plt.colorbar(surf1)

ax2 = plt.subplot(122, projection='3d')
surf2 = ax2.plot_surface(RPMmatr, Diammatr, np.log10(Power_matr/1000), cmap='viridis')
ax2.set_xlabel('RPM')
ax2.set_ylabel('prop.diam., m')
ax2.set_zlabel('log10(total power, kW)')
plt.colorbar(surf2)

# Battery and hover time plots
plt.figure(figsize=(15, 6))
ax3 = plt.subplot(121, projection='3d')
surf3 = ax3.plot_surface(RPMmatr, Diammatr, m_batt_matr, cmap='viridis')
ax3.set_xlabel('RPM')
ax3.set_ylabel('prop.diam., m')
ax3.set_zlabel('battery weight, kg')
plt.colorbar(surf3)

ax4 = plt.subplot(122, projection='3d')
surf4 = ax4.plot_surface(RPMmatr, Diammatr, t_hover_matr/60, cmap='viridis')
ax4.set_xlabel('RPM')
ax4.set_ylabel('prop.diam., m')
ax4.set_zlabel('hover time, min')
plt.colorbar(surf4)

# Cropped results plots
plt.figure(figsize=(15, 12))
ax5 = plt.subplot(221, projection='3d')
surf5 = ax5.plot_surface(RPMmatr, Diammatr, t_hover_crop/60, cmap='viridis')
ax5.set_xlabel('RPM')
ax5.set_ylabel('prop.diam., m')
ax5.set_zlabel('hover time, min')
plt.colorbar(surf5)

ax6 = plt.subplot(222, projection='3d')
surf6 = ax6.plot_surface(RPMmatr, Diammatr, m_total_crop, cmap='viridis')
ax6.set_xlabel('RPM')
ax6.set_ylabel('prop.diam., m')
ax6.set_zlabel('total weight, kg')
plt.colorbar(surf6)

ax7 = plt.subplot(223, projection='3d')
surf7 = ax7.plot_surface(RPMmatr, Diammatr, Pmotor_crop/1000, cmap='viridis')
ax7.set_xlabel('RPM')
ax7.set_ylabel('prop.diam., m')
ax7.set_zlabel('single motor power, kW')
plt.colorbar(surf7)

ax8 = plt.subplot(224, projection='3d')
surf8 = ax8.plot_surface(RPMmatr, Diammatr, m_batt_crop, cmap='viridis')
ax8.set_xlabel('RPM')
ax8.set_ylabel('prop.diam., m')
ax8.set_zlabel('battery weight, kg')
plt.colorbar(surf8)

# Range optimization plots
plt.figure(figsize=(15, 5))
ax9 = plt.subplot(131, projection='3d')
surf9 = ax9.plot_surface(RPMmatr_mod, Diammatr_mod, np.rad2deg(thetaopt_matr_mod), cmap='viridis')
ax9.set_xlabel('RPM')
ax9.set_ylabel('prop.diam., m')
ax9.set_zlabel('opt. bank angle, deg.')
plt.colorbar(surf9)

ax10 = plt.subplot(132, projection='3d')
surf10 = ax10.plot_surface(RPMmatr_mod, Diammatr_mod, speedopt_matr_mod*3.6, cmap='viridis')
ax10.set_xlabel('RPM')
ax10.set_ylabel('prop.diam., m')
ax10.set_zlabel('opt. speed, km/h')
plt.colorbar(surf10)

ax11 = plt.subplot(133, projection='3d')
surf11 = ax11.plot_surface(RPMmatr_mod, Diammatr_mod, range_matr_mod/1000, cmap='viridis')
ax11.set_xlabel('RPM')
ax11.set_ylabel('prop.diam., m')
ax11.set_zlabel('range, km')
plt.colorbar(surf11)

# plt.tight_layout()
plt.show()