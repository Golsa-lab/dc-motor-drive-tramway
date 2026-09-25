% Vehicle and Motor Parameters
m_tram = 15000; % [kg]
m_passengers = 130 * 80; % [kg]
m_total = m_tram + m_passengers;

rho = 13/74; % Gearbox ratio
d = 0.68; % Wheel diameter [m]
beta = 0.81; % Friction [Nms]

% Electrical Parameters
Ra = 0.39; % [Ohm]
tau_a = 10e-3; % [s]
La = tau_a * Ra; % [H]
Kt = 1.06; % [Nm/A]

Re = 12; % [Ohm]
tau_e = 0.1; % [s]
Le = tau_e * Re; % [H]
I_en = 5;
V_en = 60;

% Inertia calculation
Jeq = m_total * (rho^2 * d^2) / 4

% Nominal values for Field Weakening logic
Omega_b = 970 * (2*pi/60) % Base speed in rad/s
En = 600 - Ra*156 % Nominal Back-EMF (adjust based on motor specs) kt*Omega_b
V_r = Omega_b * rho * d/2 %Rated speed [m/s]
V_max = 42 * (1000/3600) %[m/s]
Ia_max = 156;
L=[0,1,3,4,6,8,9,10];
V_ref=[V_r/2,V_r,V_r,V_max,V_r,V_r,V_r/2,0];
S_ref = [0, 0, 5, 0, 0, -5, 0, 0] / 100;
s=tf('s');
plant_a= 1/(Ra+s*La);
plant_e=1/(Re+s*Le);
plant_M = Kt / (Jeq * s + beta);



BW_a=100;
BW_e=10;
BW_M=0.5;

PM_a = 85;   % loop corrente
PM_e = 60;   % loop eccitazione
PM_v = 65;   % loop velocità: medio-> compromesso overshoot/undershoot

opt_a = pidtuneOptions('CrossoverFrequency', BW_a, 'PhaseMargin', PM_a);
opt_e = pidtuneOptions('CrossoverFrequency', BW_e, 'PhaseMargin', PM_e);
opt_M = pidtuneOptions('CrossoverFrequency', BW_M, 'PhaseMargin', PM_v);

% Tune the controller (e.g., for a PI controller)
[C_a, info_a] = pidtune(plant_a ,'PI', opt_a);
Kp_ia=C_a.Kp
Ki_ia=C_a.Ki
[C_e, info_e] = pidtune(plant_e,'PI', opt_e);
Kp_ie=C_e.Kp
Ki_ie=C_e.Ki
[C_M, info_M] = pidtune(plant_M, 'PI', opt_M);
Kp_v=C_M.Kp
Ki_v=C_M.Ki