%% ISTrain — Energy Recovery Challenge (script)
% Travagem de 15 km/h -> 0 com travagem regenerativa + atualização do supercap.
% Inclui limites por aderência, binário por motor, corrente máxima e tensão do supercap.

clc; clear;

%% -------------------- Parâmetros do veículo / via -----------------------
m_loco   = 600;      % [kg] massa locomotiva
g        = 9.81;     % [m/s^2]
Crr      = 0.004;    % [-] rolling resistance coeff
rw       = 0.100;    % [m] raio da roda
mech_eff = 0.85;     % [-] eficiência mecânica (drivetrain)
G        = 10;       % [-] relação de caixa
mu       = 0.35;     % [-] coef. aderência roda-carril
n_m      = 2;        % [-] nº de motores
SF       = 30;       % [%] margem de segurança no binário

v_max    = 15/3.6;   % [m/s] 15 km/h (limite prova)
a_cmd    = 1.3;      % [m/s^2] desaceleração-alvo (TS.1b)

%% -------------------- Limites elétricos e eficiências -------------------
I_max     = 300;     % [A] limite de corrente no DC bus
Vbus      = 48;      % [V] tensão nominal / limite do supercap

eta_sc    = 0.90;    % eficiência de carga do supercap
eta_dc    = 0.85;    % eficiência DC/DC (caminho de carga)
eta_tx    = mech_eff;% eficiência mecânica (transmissão)
eta_motor = 0.90;    % eficiência motor+inversor em regeneração
eta_reg   = eta_tx * eta_motor;       % rodas -> DC bus
eta_path  = eta_reg * eta_dc * eta_sc;% rodas -> energia armazenada (info)

%% -------------------- Seleção do Supercapacitor -------------------------
% cap_choice = 1: 3x16V58F em série (19.33F);
% cap_choice = 2: 48V 83F;
% cap_choice = 3: 48V 165F
cap_choice = 3;

switch cap_choice
    case 1
        C   = 19.33;          % [F]
        ESR = 3*0.0029;       % [ohm] aprox (soma em série)
        cap_name = '3x16V 58F (19.33F)';
    case 2
        C   = 83;             % [F]
        ESR = 0.0022;         % [ohm] aprox
        cap_name = '48V 83F';
    case 3
        C   = 165;            % [F]
        ESR = 0.0011;         % [ohm] aprox
        cap_name = '48V 165F';
    otherwise
        error('cap_choice inválido');
end

%% -------------------- Pré-carga do supercap (V0) ------------------------
use_optimal_V0 = true;     % <— colocar false para definir V0 manualmente
Erecov_event   = 7.8e3;    % [J] energia alvo do evento (se usar V0 ótimo)

if use_optimal_V0
    % V0 ótimo por evento (derivado do teu critério): V0 = sqrt(Vin^2 - 2E/C)
    Vin = Vbus;
    V0  = sqrt(max(Vin^2 - (2*Erecov_event)/C, 0));
else
    V0  = 0;                % [V] define manualmente
end
V0 = min(V0, Vbus);         % clamp de segurança

%% -------------------- Timeline e perfis básicos -------------------------
dt     = 0.01;                          % [s]
t_stop = v_max / a_cmd;                 % [s] tempo de travagem se adesão permitir
t      = (0:dt:t_stop).';               % [s]
N      = numel(t);

% Velocidade alvo (desaceleração constante)
v      = max(v_max - a_cmd.*t, 0);      % [m/s]
x      = cumtrapz(t, v);                % [m] posição

% Resistência ao rolamento (aprox constante)
F_rr   = Crr * m_loco * g;              % [N]

% Limite por aderência
F_adh  = mu  * m_loco * g;              % [N]

%% -------------------- Estados e logs ------------------------------------
Vcap   = zeros(N,1);  Vcap(1) = V0;     % [V]
Ecap   = zeros(N,1);  Ecap(1) = 0.5*C*V0^2;  % [J]
P_rec  = zeros(N,1);                    % [W] potência armazenada
I_bus  = zeros(N,1);                    % [A] corrente de carga no bus
F_regen= zeros(N,1);                    % [N] força regenerativa aplicada
T_perM = zeros(N,1);                    % [Nm] binário por motor (lado motor)

%% -------------------- Loop de simulação (fase de travagem) --------------
for k = 1:N-1
    % Força total necessária para cumprir a_cmd
    F_req_total = m_loco * a_cmd;                      % [N]

    % Força que o motor precisa efetivamente gerar (desconta rolamento)
    F_need_from_motor = max(F_req_total - F_rr, 0);    % [N]

    % Limite por aderência
    F_by_adh = min(F_need_from_motor, F_adh);          % [N]

    % Binário pedido por motor (lado motor)
    T_req_per_motor = (F_by_adh * rw) / (G * mech_eff) / n_m; % [Nm]

    % Limite de binário seguro por motor a partir da aderência (com margem)
    T_adh_per_motor   = (F_adh * rw) / (G * mech_eff) / n_m;  % [Nm]
    T_limit_per_motor = (1 - SF/100) * T_adh_per_motor;       % [Nm]

    % Escalonar força se binário pedido > limite
    scale = min(1, T_limit_per_motor / max(T_req_per_motor, eps));
    F_used = F_by_adh * scale;                          % [N]
    F_regen(k) = F_used;

    % Binário real por motor para logging
    T_perM(k) = (F_used * rw) / (G * mech_eff) / n_m;   % [Nm]

    % Potência mecânica extraída nas rodas (regenerativa)
    P_wheel = F_used * v(k);                            % [W]

    % Potência disponível no DC bus (pós motor/inversor)
    P_toDC  = eta_reg * P_wheel;                        % [W]

    % Limites elétricos: corrente e ESR (buck para Vcap < Vbus)
    I_esr   = max((Vbus - Vcap(k)) / max(ESR,1e-6), 0); % [A] limite instantâneo
    I_allow = min(I_max, I_esr);
    P_Ilim  = Vbus * I_allow;                           % [W]

    % Potência que efetivamente entra no supercap (após DC/DC + cap)
    P_cap_possible = min(P_toDC, P_Ilim) * eta_dc * eta_sc;

    % Não exceder a tensão do supercap
    if Vcap(k) >= Vbus - 1e-3
        P_cap_possible = 0;
    end

    % Atualização do estado do supercap
    dE          = P_cap_possible * dt;                  % [J]
    Ecap(k+1)   = Ecap(k) + dE;
    Vcap(k+1)   = min(sqrt(2*Ecap(k+1)/C), Vbus);

    % Registos
    P_rec(k) = P_cap_possible;
    I_bus(k) = (P_cap_possible > 0) * P_cap_possible / max(Vbus,1e-6);
end

% Resultados finais
E_recovered = Ecap(end);                          % [J]
Vcap_end    = Vcap(end);                          % [V]
idx_stop    = find(v<=0,1,'first');
s_stop      = x(idx_stop);                        % [m]

%% -------------------- Report --------------------------------------------
fprintf('--- ISTrain Energy Recovery ---\n');
fprintf('Capacitor: %s | C=%.2f F | ESR=%.4f ohm\n', cap_name, C, ESR);
fprintf('V0 (pre-charge): %.2f V  ->  Vcap_end: %.2f V\n', V0, Vcap_end);
fprintf('Recovered energy: %.1f J (%.2f Wh)\n', E_recovered, E_recovered/3600);
fprintf('Braking time: %.2f s | distance: %.1f m\n', t_stop, s_stop);
fprintf('Overall path eff. (wheel->stored): %.2f %%\n', 100*eta_path);

%% -------------------- Plots ---------------------------------------------
figure('Name','ISTrain — Energy Recovery','Color','w');

subplot(2,3,1);
plot(t, v*3.6,'LineWidth',1.8); grid on;
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Velocidade (15 km/h \rightarrow 0)');

subplot(2,3,2);
plot(t, P_rec/1000,'LineWidth',1.8); hold on; grid on;
plot(t, (m_loco*a_cmd.*v)/1000,'--','LineWidth',1.2);
xlabel('Time [s]'); ylabel('Power [kW]');
legend('Pot. armazenada','m a \cdot v (ref)','Location','best');
title('Potência regenerativa');

subplot(2,3,3);
plot(t, Vcap,'LineWidth',1.8); grid on;
xlabel('Time [s]'); ylabel('V_{cap} [V]');
title(sprintf('V_{cap} (V_0=%.1f V \\rightarrow %.1f V)', V0, Vcap_end));
ylim([0, Vbus+2]);

subplot(2,3,4);
plot(t, Ecap/1000,'LineWidth',1.8); grid on;
xlabel('Time [s]'); ylabel('E_{cap} [kJ]');
title(sprintf('Energia recuperada = %.2f kJ', E_recovered/1000));

subplot(2,3,5);
plot(t, F_regen,'LineWidth',1.8); hold on; grid on;
yline(F_rr,'--','F_{rr}');
yline(F_adh,':','F_{adh}');
xlabel('Time [s]'); ylabel('Force [N]');
title('Forças: regen, rolamento e aderência');
legend('F_{regen}','F_{rr}','F_{adh}','Location','best');

subplot(2,3,6);
plot(t, T_perM,'LineWidth',1.8); hold on; grid on;
T_adh_per_motor = (F_adh*rw)/(G*mech_eff)/n_m;
yline((1-SF/100)*T_adh_per_motor,'--','T_{lim} (com SF)');
xlabel('Time [s]'); ylabel('Torque per motor [Nm]');
title('Binário por motor (lado motor)');
