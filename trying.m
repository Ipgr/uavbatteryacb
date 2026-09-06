function uav_battery_thermal_catapult_advanced()

clear; clc; close all;
t_start_calc = tic;
set(groot, ...
    'defaultAxesXColor',[0 0 0], ...
    'defaultAxesYColor',[0 0 0], ...
    'defaultTextColor',[0 0 0], ...
    'defaultLegendTextColor',[0 0 0], ...
    'defaultLegendColor',[1 1 1], ...
    'defaultLegendEdgeColor',[0 0 0], ...
    'defaultAxesGridColor',[0 0 0]);

print_header('ЗАПУСК');

%% 1) ПАРАМЕТРЫ
T_amb      = 35.0;   T0 = 28.0;   t_end_sim = 12000;
V_cruise   = 20.83;  H_flight = 400;

n_series   = 6;      n_parallel = 3; U_nom = 22.2;
Q_batt_cap = 15.0*3600; SOC_0 = 1.00;

C_core = 360; C_shell = 212;
R_core_shell = 0.045; R_shell_pcm1 = 0.020; R_pcm12 = 0.120;

m_pcm_design = 0.14; pcm_split = 0.5;
c_solid = 2100; c_liquid = 2400; L_pcm = 200000;
T_sol = 35.0; T_melt = 38.0; T_liq = 41.0;
dT_melt = T_liq - T_sol; eps_tanh = 0.8;

A_rad = 0.0200; epsilon = 0.9; sigma_SB = 5.67e-8;

L_rad = 0.20; A_wetted = 0.030; A_front = 0.0025; Cd_form = 0.9;

P_elec_boost = 220; P_elec_cruise = 170; tau_P = 8.0; tau_h = 1.0;

T_opt_thresh = 40.0; T_warn = 45.0; T_crit = 60.0;

RelTol = 1e-6; AbsTol = 1e-8; MaxStep = 0.2;

%% 2) STRUCT p
p = struct();
p.T_amb=T_amb; p.T0=T0; p.t_end_sim=t_end_sim; p.V_cruise=V_cruise; p.H_flight=H_flight;
p.n_series=n_series; p.n_parallel=n_parallel; p.U_nom=U_nom; p.Q_batt_cap=Q_batt_cap; p.SOC_0=SOC_0;
p.C_core=C_core; p.C_shell=C_shell; p.R_core_shell=R_core_shell; p.R_shell_pcm1=R_shell_pcm1; p.R_pcm12=R_pcm12;
p.m_pcm_design=m_pcm_design; p.pcm_split=pcm_split; p.c_solid=c_solid; p.c_liquid=c_liquid; p.L_pcm=L_pcm;
p.T_sol=T_sol; p.T_melt=T_melt; p.T_liq=T_liq; p.dT_melt=dT_melt; p.eps_tanh=eps_tanh;
p.A_rad=A_rad; p.epsilon=epsilon; p.sigma_SB=sigma_SB;
p.L_rad=L_rad; p.A_wetted=A_wetted; p.A_front=A_front; p.Cd_form=Cd_form;
p.P_elec_boost=P_elec_boost; p.P_elec_cruise=P_elec_cruise; p.tau_P=tau_P; p.tau_h=tau_h;
p.T_opt_thresh=T_opt_thresh; p.T_warn=T_warn; p.T_crit=T_crit;
p.RelTol=RelTol; p.AbsTol=AbsTol; p.MaxStep=MaxStep;

% --- Piecewise mission profile (W, s) ---
p.t_takeoff   = 90;      p.P_takeoff   = 600;
p.t_climb     = 300;     p.P_climb     = 420;
p.t_cruise1   = 4200;    p.P_cruise1   = 280;
p.t_maneuver  = 180;     p.P_maneuver  = 500;
p.t_cruise2   = 6000;    p.P_cruise2   = 260;
p.t_descent   = 900;     p.P_descent   = 200;
p.t_landing   = 330;     p.P_landing   = 150;
p.t_end_sim = p.t_takeoff + p.t_climb + p.t_cruise1 + p.t_maneuver + ...
              p.t_cruise2 + p.t_descent + p.t_landing;

% --- PATCH 1: масса планера и аэродинамика ---
p.m_empty   = 3.50;   % кг, пустой планер
p.m_payload = 0.80;   % кг, полезная нагрузка
p.m_batt    = 1.20;   % кг, батарея
p.S_ref     = 0.42;   % м², площадь крыла
p.b_span    = 2.10;   % м, размах
p.e_oswald  = 0.82;   % коэффициент Освальда
p.CD0       = 0.028;  % базовое сопротивление планера
p.eta_prop  = 0.78;   % КПД пропеллера
p.eta_motor = 0.88;   % КПД мотора
p.g         = 9.81;
p.rho_rad   = 2700;   % кг/м³ алюминий
p.t_rad     = 0.002;  % м, толщина пластины радиатора
m_radiator  = p.rho_rad * p.A_rad * p.t_rad;
p.m_total   = p.m_empty + p.m_payload + p.m_batt + m_pcm_design + m_radiator;

% --- PATCH 2: RC-параметры батареи (модель Тевенина) ---
p.R1 = 0.005;   % Ом
p.C1 = 3500;    % Ф

fprintf('[МАССА]  m_total=%.3f кг (радиатор=%.0f г, PCM=%.0f г)\n', ...
    p.m_total, m_radiator*1000, m_pcm_design*1000);

print_variable_dictionary_full(p);

%% 3) RUNS
fprintf('[ ОСНОВНОЙ РАСЧЁТ  m_pcm=%.2f кг ]\n', m_pcm_design);
res_design = run_simulation_4node(m_pcm_design, p);
fprintf('  Шагов: %d\n\n', length(res_design.t));

fprintf('[ КОНТРОЛЬНЫЙ РАСЧЁТ  m_pcm=0, без радиатора ]\n');
p_noRad = p;
p_noRad.A_rad    = 0.0;
p_noRad.epsilon  = 0.0;
p_noRad.A_wetted = 0.0;
p_noRad.A_front  = 0.0;
res_noPCM = run_simulation_4node(0, p_noRad);
fprintf('  Готово\n\n');

compare_cooling_variants_4cases(p);

%% 4) POST
t = res_design.t; T_core=res_design.T_core; T_shell=res_design.T_shell;
T_pcm1=res_design.T_pcm1; T_pcm2=res_design.T_pcm2; N=length(t);

Q_ohm_v=zeros(N,1); Q_RC_v=zeros(N,1); Q_ent_v=zeros(N,1); Q_gen_v=zeros(N,1);
I_v=zeros(N,1); SOC_v=zeros(N,1); h_v=zeros(N,1);
Q_conv_v=zeros(N,1); Q_rad_v=zeros(N,1); P_drag_v=zeros(N,1);
f1_v=zeros(N,1); f2_v=zeros(N,1);

for i=1:N
    [Ii,SOCi]=mission_current_soc(t(i),p);
    [Rpack,dUdT]=battery_maps(T_core(i),SOCi,p);
    Q_ohm_v(i)=Ii^2*Rpack;
    Q_RC_v(i)=res_design.V_RC(i)*Ii;
    Q_ent_v(i)=Ii*(T_core(i)+273.15)*dUdT;
    Q_gen_v(i)=Q_ohm_v(i)+Q_RC_v(i)+Q_ent_v(i);

    h_v(i)=h_external(T_pcm2(i),p.T_amb,p.V_cruise,p.H_flight,p);
    Q_conv_v(i)=h_v(i)*p.A_rad*(T_pcm2(i)-p.T_amb);
    Q_rad_v(i)=p.epsilon*p.sigma_SB*p.A_rad*((T_pcm2(i)+273.15)^4-(p.T_amb+273.15)^4);

    [~,P_drag_v(i)] = radiator_drag(T_pcm2(i), p.T_amb, p.V_cruise, p.H_flight, p);
    dT1_sign_post = branch_sign_from_samples(T_pcm1, i);
    dT2_sign_post = branch_sign_from_samples(T_pcm2, i);
    f1_v(i)=f_liquid(T_pcm1(i),p,dT1_sign_post);
    f2_v(i)=f_liquid(T_pcm2(i),p,dT2_sign_post);
    I_v(i)=Ii; SOC_v(i)=SOCi;
end

E_gen=trapz(t,Q_gen_v); E_ohm=trapz(t,Q_ohm_v); E_RC=trapz(t,Q_RC_v); E_ent=trapz(t,Q_ent_v);
E_conv=trapz(t,Q_conv_v); E_rad=trapz(t,Q_rad_v); E_drag=trapz(t,P_drag_v);

m1=m_pcm_design*p.pcm_split; m2=m_pcm_design*(1-p.pcm_split);
f1_end_sign = branch_sign_from_samples(T_pcm1, N);
f2_end_sign = branch_sign_from_samples(T_pcm2, N);
f1_init = f_liquid(T_pcm1(1), p, branch_sign_from_samples(T_pcm1, 1));
f2_init = f_liquid(T_pcm2(1), p, branch_sign_from_samples(T_pcm2, 1));
f1_end  = f_liquid(T_pcm1(end), p, f1_end_sign);
f2_end  = f_liquid(T_pcm2(end), p, f2_end_sign);
E_lat = m1*p.L_pcm*(f1_end-f1_init) + m2*p.L_pcm*(f2_end-f2_init);
E_pcm1 = pcm_stored_energy(T_pcm1(end), T_pcm1(1), m1, p, f1_end_sign);
E_pcm2 = pcm_stored_energy(T_pcm2(end), T_pcm2(1), m2, p, f2_end_sign);
E_pcm_sensible = E_pcm1 + E_pcm2 - E_lat;
E_stored_total = p.C_core*(T_core(end)-p.T0) + p.C_shell*(T_shell(end)-p.T0) + E_pcm1 + E_pcm2;
E_out_total = E_conv+E_rad+E_stored_total;
E_balance = abs(E_gen-E_out_total)/max(E_gen,1)*100;

[T_core_max,idx_cm]=max(T_core); t_core_max=t(idx_cm);
[T_shell_max,idx_sm]=max(T_shell); t_shell_max=t(idx_sm);

dt_avg=mean(diff(t));
t_above_opt=sum(T_shell>p.T_opt_thresh)*dt_avg;
t_above_warn=sum(T_shell>p.T_warn)*dt_avg;
t_above_crit=sum(T_shell>p.T_crit)*dt_avg;

deg_integrated=trapz(t,2.^((T_shell-25)./10))/p.t_end_sim;

%% 5) SENSITIVITY
m_pcm_grid=linspace(0,0.25,21); n_grid=length(m_pcm_grid);
Tcore_max_grid=zeros(n_grid,1); Tshell_max_grid=zeros(n_grid,1); Tpcm2_max_grid=zeros(n_grid,1);

for k=1:n_grid
    r=run_simulation_4node(m_pcm_grid(k),p);
    Tcore_max_grid(k)=max(r.T_core);
    Tshell_max_grid(k)=max(r.T_shell);
    Tpcm2_max_grid(k)=max(r.T_pcm2);
end

T_target=44.5;
idx_safe=find(Tcore_max_grid<=T_target & Tshell_max_grid<=p.T_warn,1,'first');
if ~isempty(idx_safe)
    m_opt=m_pcm_grid(idx_safe); T_opt_knee=Tcore_max_grid(idx_safe); opt_found=true;
else
    [T_opt_knee,idx_k]=min(Tcore_max_grid); m_opt=m_pcm_grid(idx_k); opt_found=false;
end
[~,idx_design]=min(abs(m_pcm_grid-m_pcm_design));

%% 6) CONSOLE REPORT
W=72; SEP=['╠' repmat('═',1,W) '╣']; TOP=['╔' repmat('═',1,W) '╗']; BOT=['╚' repmat('═',1,W) '╝'];
fprintf('\n%s\n',TOP);
fprintf('║%s║\n',center_text('ПОЛНЫЙ ТЕХНИЧЕСКИЙ ОТЧЁТ | CATAPULT UAV',W));
fprintf('%s\n',SEP);
fprintf('║%s║\n',section_title('1. КОНФИГУРАЦИЯ',W)); fprintf('%s\n',SEP);
fprintf('║  %-70s║\n',sprintf('6S3P, U=%.1fV, Ccore=%.1f, Cshell=%.1f',p.U_nom,p.C_core,p.C_shell));
fprintf('║  %-70s║\n',sprintf('Rcs=%.3f, Rsp=%.3f, Rp12=%.3f K/W',p.R_core_shell,p.R_shell_pcm1,p.R_pcm12));
fprintf('║  %-70s║\n',sprintf('PCM=%.0f g, Tsol=%.0f, Tliq=%.0f',m_pcm_design*1000,p.T_sol,p.T_liq));
fprintf('║  %-70s║\n',sprintf('m_total=%.3f кг, RC: R1=%.4f Ohm, C1=%.0f F',p.m_total,p.R1,p.C1));
fprintf('%s\n',SEP);
fprintf('║%s║\n',section_title('2. ТЕМПЕРАТУРЫ',W)); fprintf('%s\n',SEP);
fprintf('║  %-70s║\n',sprintf('Tcore max = %.2f°C @ %.1fs',T_core_max,t_core_max));
fprintf('║  %-70s║\n',sprintf('Tshell max= %.2f°C @ %.1fs',T_shell_max,t_shell_max));
fprintf('║  %-70s║\n',sprintf('T>40: %.1fs | T>45: %.1fs | T>60: %.1fs',t_above_opt,t_above_warn,t_above_crit));
fprintf('%s\n',SEP);
fprintf('║%s║\n',section_title('3. ЭНЕРГОБАЛАНС',W)); fprintf('%s\n',SEP);
fprintf('║  %-70s║\n',sprintf('Egen=%.1fJ (Ohm=%.1fJ, RC=%.1fJ, Ent=%.1fJ)',E_gen,E_ohm,E_RC,E_ent));
fprintf('║  %-70s║\n',sprintf('Econv=%.1fJ, Erad=%.1fJ, Estored=%.1fJ (PCM=%.1fJ, Lat=%.1fJ)', ...
    E_conv,E_rad,E_stored_total,E_pcm1+E_pcm2,E_lat));
fprintf('║  %-70s║\n',sprintf('PCM sensible=%.1fJ, Edrag=%.1fJ, Residual=%.4f%%',E_pcm_sensible,E_drag,E_balance));
fprintf('%s\n',SEP);
fprintf('║%s║\n',section_title('4. ОПТИМУМ PCM',W)); fprintf('%s\n',SEP);
if opt_found
    fprintf('║  %-70s║\n',sprintf('m_opt=%.1fg -> Tcore_max=%.2f°C',m_opt*1000,T_opt_knee));
else
    fprintf('║  %-70s║\n',sprintf('Цель %.1f°C не достигнута до 250g',T_target));
end
fprintf('║  %-70s║\n',sprintf('Проект %.0fg: Tcore=%.2f°C, Tshell=%.2f°C', ...
    m_pcm_design*1000,Tcore_max_grid(idx_design),Tshell_max_grid(idx_design)));
fprintf('%s\n',SEP);
fprintf('║%s║\n',section_title('5. РЕСУРС',W)); fprintf('%s\n',SEP);
fprintf('║  %-70s║\n',sprintf('Arrhenius factor avg: %.4f',deg_integrated));
fprintf('║  %-70s║\n',sprintf('SOC final: %.1f%%',SOC_v(end)*100));
fprintf('%s\n',BOT);

%% 7) CSV
fid=fopen('thermal_report_catapult.csv','w');
fprintf(fid,'t_s,T_core_C,T_shell_C,T_pcm1_C,T_pcm2_C,I_A,SOC,Q_ohm_W,Q_RC_W,Q_ent_W,Q_gen_W,h_W_m2K,Q_conv_W,Q_rad_W,f1,f2,Pdrag_W\n');
for i=1:N
    fprintf(fid,'%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n',...
        t(i),T_core(i),T_shell(i),T_pcm1(i),T_pcm2(i),I_v(i),SOC_v(i),Q_ohm_v(i),Q_RC_v(i),Q_ent_v(i),Q_gen_v(i),...
        h_v(i),Q_conv_v(i),Q_rad_v(i),f1_v(i),f2_v(i),P_drag_v(i));
end
fclose(fid);

%% 8) FIGURES
C_darkblue=[0.05 0.25 0.55]; C_orange=[0.90 0.45 0.00];
C_green=[0.10 0.55 0.15]; C_red=[0.80 0.05 0.05];
C_purple=[0.50 0.00 0.75]; C_gray=[0.45 0.45 0.45];
FS=11; FST=12; LW=2.4; LWs=1.4;

fig1=figure('Color','w','Position',[25,70,1200,860]);
tl1=tiledlayout(fig1,3,2,'TileSpacing','compact','Padding','compact');
title(tl1,'CATAPULT UAV — Тепловая динамика узлов','FontSize',13,'FontWeight','bold','Color','k');

ax11=nexttile(tl1,[1 2]);
plot(ax11,res_noPCM.t,res_noPCM.T_core,':','Color',C_gray,'LineWidth',1.8,'DisplayName','T_{core} без PCM/Рад');
hold(ax11,'on');
plot(ax11,t,T_core,'-','Color',C_red,'LineWidth',LW,'DisplayName','T_{core}');
plot(ax11,t,T_shell,'-','Color',C_darkblue,'LineWidth',LW,'DisplayName','T_{shell}');
plot(ax11,t,T_pcm1,'-','Color',C_purple,'LineWidth',LW,'DisplayName','T_{pcm1}');
plot(ax11,t,T_pcm2,'-','Color',C_orange,'LineWidth',LW,'DisplayName','T_{pcm2}');
% вертикальные линии фаз миссии
phase_t = cumsum([0, p.t_takeoff, p.t_climb, p.t_cruise1, p.t_maneuver, p.t_cruise2, p.t_descent]);
phase_names = {'takeoff','climb','cruise1','maneuver','cruise2','descent','landing'};
yl = ylim(ax11);
for pp=2:length(phase_t)
    xline(ax11, phase_t(pp), '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
        'Label', phase_names{pp-1}, 'LabelVerticalAlignment','bottom','FontSize',7);
end
yline(ax11,40,':','Color',[0 0.6 0],'LineWidth',1.4,'Label','40°C');
yline(ax11,45,'--','Color',C_orange,'LineWidth',1.4,'Label','45°C');
yline(ax11,60,'--','Color',C_red,'LineWidth',1.4,'Label','60°C');
hold(ax11,'off'); grid(ax11,'on'); box(ax11,'on');
xlabel(ax11,'Время, с','FontSize',FS,'FontWeight','bold','Color','k');
ylabel(ax11,'Температура, °C','FontSize',FS,'FontWeight','bold','Color','k');
title(ax11,'Температуры 4 узлов + фазы миссии','FontSize',FST,'FontWeight','bold','Color','k');
legend(ax11,'Location','northeast','FontSize',8,'NumColumns',2);
set(ax11,'XColor','k','YColor','k');

ax12=nexttile(tl1);
yyaxis(ax12,'left');
plot(ax12,t,Q_gen_v,'-k','LineWidth',LW);
ylabel(ax12,'Q_{gen}, Вт','Color','k');
yyaxis(ax12,'right');
plot(ax12,t,h_v,'-','Color',C_green,'LineWidth',LW);
ylabel(ax12,'h_{out}, Вт/(м²К)','Color','k');
grid(ax12,'on'); box(ax12,'on');
xlabel(ax12,'Время, с','Color','k');
title(ax12,'Q_{gen}(t) и h(t)','Color','k');

ax13=nexttile(tl1);
plot(ax13,t,Q_conv_v,'-','Color',C_darkblue,'LineWidth',LW,'DisplayName','Q_{conv}');
hold(ax13,'on');
plot(ax13,t,Q_rad_v,'--','Color',C_orange,'LineWidth',LWs,'DisplayName','Q_{rad}');
plot(ax13,t,P_drag_v,'-.','Color',C_gray,'LineWidth',LWs,'DisplayName','P_{drag}');
hold(ax13,'off'); grid(ax13,'on'); box(ax13,'on');
xlabel(ax13,'Время, с','Color','k'); ylabel(ax13,'Вт','Color','k');
title(ax13,'Теплоотвод и аэродинамическая цена','Color','k');
legend(ax13,'Location','best');

ax14=nexttile(tl1);
plot(ax14,t,f1_v*100,'-','Color',C_purple,'LineWidth',LW,'DisplayName','f_{liq,1}');
hold(ax14,'on');
plot(ax14,t,f2_v*100,'--','Color',C_orange,'LineWidth',LW,'DisplayName','f_{liq,2}');
hold(ax14,'off'); grid(ax14,'on'); box(ax14,'on');
xlabel(ax14,'Время, с','Color','k'); ylabel(ax14,'Доля жидкости, %','Color','k');
title(ax14,'Плавление PCM по слоям','Color','k');
legend(ax14,'Location','best');

fig2=figure('Color','w','Position',[50,60,1000,680]);
ax21=axes(fig2);
plot(ax21,m_pcm_grid*1000,Tcore_max_grid,'-o','Color',C_red,'LineWidth',LW,...
    'MarkerFaceColor',C_red,'DisplayName','T_{core,max}');
hold(ax21,'on');
plot(ax21,m_pcm_grid*1000,Tshell_max_grid,'-s','Color',C_darkblue,'LineWidth',LW,...
    'MarkerFaceColor',C_darkblue,'DisplayName','T_{shell,max}');
plot(ax21,m_pcm_grid*1000,Tpcm2_max_grid,'-^','Color',C_orange,'LineWidth',LWs,...
    'MarkerFaceColor',C_orange,'DisplayName','T_{pcm2,max}');
if opt_found
    plot(ax21,m_opt*1000,T_opt_knee,'p','Color',C_red,'MarkerFaceColor',C_red,...
        'MarkerSize',17,'DisplayName','Инж. оптимум');
end
yline(ax21,40,'--','Color',[0 0.6 0],'LineWidth',1.4,'Label','Лимит 40°C');
yline(ax21,45,'--','Color',C_orange,'LineWidth',1.4,'Label','Warn 45°C');
hold(ax21,'off'); grid(ax21,'on'); box(ax21,'on');
xlabel(ax21,'Масса PCM, г','FontSize',FS+1,'FontWeight','bold','Color','k');
ylabel(ax21,'Температура, °C','FontSize',FS+1,'FontWeight','bold','Color','k');
title(ax21,'Сенситивити и инженерный оптимум','FontSize',FST+1,'FontWeight','bold','Color','k');
legend(ax21,'Location','northeast');

patch_figure_readability(fig1);
patch_figure_readability(fig2);
fprintf('CSV: thermal_report_catapult.csv\n');
fprintf('Runtime: %.2f s\n', toc(t_start_calc));

end
% ================== END MAIN ==================

% -----------------------------------------------------------------------
function res = run_simulation_4node(m_pcm, p)
% 5 состояний: [T_core, T_shell, T_pcm1, T_pcm2, V_RC]
rhs  = @(t,y) ode4_rhs(t,y,m_pcm,p);
opts = odeset('RelTol',p.RelTol,'AbsTol',p.AbsTol,'MaxStep',p.MaxStep);
[t,y] = ode15s(rhs,[0 p.t_end_sim],[p.T0; p.T0; p.T0; p.T0; 0.0],opts);
res.t      = t;
res.T_core  = y(:,1);
res.T_shell = y(:,2);
res.T_pcm1  = y(:,3);
res.T_pcm2  = y(:,4);
res.V_RC    = y(:,5);
end

% -----------------------------------------------------------------------
function dydt = ode4_rhs(t,y,m_pcm,p)
Tc   = y(1);
Ts   = y(2);
T1   = y(3);
T2   = y(4);
V_RC = y(5);

% ---- ток и SOC ----
[I, SOC]      = mission_current_soc(t, p);
[Rpack, dUdT] = battery_maps(Tc, SOC, p);

% ---- RC-динамика (Тевенин) ----
dV_RC = (I*p.R1 - V_RC) / (p.R1 * p.C1);

% ---- тепловыделение ----
Q_ohm = I^2 * Rpack;
Q_RC  = V_RC * I;                          % доп. тепло от RC-блока
Q_ent = I * (Tc+273.15) * dUdT;
Qgen  = Q_ohm + Q_RC + Q_ent;

% ---- тепловые потоки ----
Qcs = (Tc - Ts) / p.R_core_shell;
Qsp = (Ts - T1) / p.R_shell_pcm1;

% PATCH 5: динамическое R_pcm12 (жидкий PCM хуже проводит)
dT1_sign = branch_sign_local(Qsp - (T1 - T2) / p.R_pcm12);
f1_now      = f_liquid(T1, p, dT1_sign);   % используем ту же ветвь гистерезиса
k_pcm       = 0.20*(1-f1_now) + 0.10*f1_now;  % 0.20 тв / 0.10 жидк. Вт/(м·К)
L_pcm_layer = 0.01;                        % м, толщина слоя
A_pcm_cs    = 0.02;                        % м², поперечное сечение
R_pcm12_dyn = L_pcm_layer / (k_pcm * A_pcm_cs);
Q12 = (T1 - T2) / R_pcm12_dyn;

% PATCH 6: знак dT для гистерезиса PCM берём из локального теплобаланса
dT1_sign = branch_sign_local(Qsp - Q12);

% ---- внешний теплообмен ----
h     = h_external(T2, p.T_amb, p.V_cruise, p.H_flight, p);
Qconv = h * p.A_rad * (T2 - p.T_amb);
Qrad  = p.epsilon * p.sigma_SB * p.A_rad * ...
        ((T2+273.15)^4 - (p.T_amb+273.15)^4);
dT2_sign = branch_sign_local(Q12 - Qconv - Qrad);

% ---- теплоёмкости PCM с гистерезисом ----
m1 = m_pcm * p.pcm_split;
m2 = m_pcm * (1 - p.pcm_split);
C1 = Ceff_pcm(T1, m1, p, dT1_sign);
C2 = Ceff_pcm(T2, m2, p, dT2_sign);

dydt = [(Qgen - Qcs)          / p.C_core;
        (Qcs  - Qsp)          / p.C_shell;
        (Qsp  - Q12)          / max(C1, 20);
        (Q12  - Qconv - Qrad) / max(C2, 20);
        dV_RC];
end

% -----------------------------------------------------------------------
% PATCH 3: динамическая мощность через аэродинамику
function [I, SOC] = mission_current_soc(t, p)

% фазовый множитель нагрузки
if t <= p.t_takeoff
    phase_mult = 2.20;
elseif t <= p.t_takeoff + p.t_climb
    phase_mult = 1.45;
elseif t <= p.t_takeoff + p.t_climb + p.t_cruise1
    phase_mult = 1.00;
elseif t <= p.t_takeoff + p.t_climb + p.t_cruise1 + p.t_maneuver
    phase_mult = 1.80;
elseif t <= p.t_takeoff + p.t_climb + p.t_cruise1 + p.t_maneuver + p.t_cruise2
    phase_mult = 0.95;
elseif t <= p.t_takeoff + p.t_climb + p.t_cruise1 + p.t_maneuver + p.t_cruise2 + p.t_descent
    phase_mult = 0.60;
else
    phase_mult = 0.45;
end

% аэродинамика
rho  = isa_density(p.H_flight, p.T_amb);
V    = p.V_cruise;
AR   = p.b_span^2 / p.S_ref;

% C_D с учётом лобового сопротивления радиатора
CD_rad   = (p.Cd_form * p.A_front) / p.S_ref;
CD_base  = p.CD0 + CD_rad;

% индуктивное сопротивление
m_eff = p.m_total * phase_mult;
CL    = (2 * m_eff * p.g) / max(rho * V^2 * p.S_ref, 1e-6);
CDi   = CL^2 / (pi * p.e_oswald * AR);

CD_total = CD_base + CDi;

% механическая мощность
P_aero    = 0.5 * rho * V^3 * p.S_ref * CD_total;
P_induced = (2*(m_eff*p.g)^2) / max(rho * V * pi * p.e_oswald * p.b_span^2, 1e-6);
P_mech    = P_aero + P_induced;
P_elec    = P_mech / (p.eta_prop * p.eta_motor);
P_elec    = max(50, min(P_elec, 800));   % клип: 50..800 Вт

I   = max(0.1, P_elec / p.U_nom);
Q_used = charge_used_piecewise(t, p);
SOC = max(0.05, p.SOC_0 - Q_used / p.Q_batt_cap);
end

% -----------------------------------------------------------------------
function Q = charge_used_piecewise(t, p)
d = [p.t_takeoff, p.t_climb, p.t_cruise1, p.t_maneuver, ...
     p.t_cruise2, p.t_descent, p.t_landing];
% для SOC используем те же phase_mult что и в mission_current_soc
mult = [2.20, 1.45, 1.00, 1.80, 0.95, 0.60, 0.45];

rho = isa_density(p.H_flight, p.T_amb);
V   = p.V_cruise;
AR  = p.b_span^2 / p.S_ref;
CD_rad  = (p.Cd_form * p.A_front) / p.S_ref;
CD_base = p.CD0 + CD_rad;

Q  = 0.0;
tt = t;
for k = 1:numel(d)
    if tt <= 0, break; end
    dt_k  = min(tt, d(k));
    m_eff = p.m_total * mult(k);
    CL    = (2*m_eff*p.g) / max(rho*V^2*p.S_ref, 1e-6);
    CDi   = CL^2 / (pi*p.e_oswald*AR);
    P_a   = 0.5*rho*V^3*p.S_ref*(CD_base+CDi);
    P_i   = (2*(m_eff*p.g)^2) / max(rho*V*pi*p.e_oswald*p.b_span^2, 1e-6);
    P_e   = max(50, min((P_a+P_i)/(p.eta_prop*p.eta_motor), 800));
    I_k   = max(0.1, P_e / p.U_nom);
    Q     = Q + I_k * dt_k;
    tt    = tt - dt_k;
end
end

% -----------------------------------------------------------------------
function [Rpack,dUdT_pack] = battery_maps(Tc, SOC, p)
Tg=[0 10 20 30 40 50]; Sg=[0.1 0.2 0.4 0.6 0.8 1.0];
Rcell=[0.028 0.024 0.020 0.018 0.017 0.016; ...
       0.022 0.019 0.016 0.015 0.014 0.013; ...
       0.018 0.016 0.013 0.012 0.011 0.0105; ...
       0.016 0.014 0.0115 0.0105 0.0098 0.0092; ...
       0.015 0.013 0.0108 0.0098 0.0092 0.0088; ...
       0.0145 0.0128 0.0105 0.0095 0.0090 0.0086];
dUdT_cell=[1.5e-4 1.2e-4 0.8e-4 0.2e-4 -0.2e-4 -0.4e-4; ...
           1.4e-4 1.1e-4 0.7e-4 0.2e-4 -0.2e-4 -0.5e-4; ...
           1.3e-4 1.0e-4 0.6e-4 0.1e-4 -0.3e-4 -0.6e-4; ...
           1.2e-4 0.9e-4 0.5e-4 0.1e-4 -0.3e-4 -0.6e-4; ...
           1.1e-4 0.8e-4 0.5e-4 0.1e-4 -0.3e-4 -0.6e-4; ...
           1.0e-4 0.8e-4 0.4e-4 0.1e-4 -0.3e-4 -0.6e-4];
Tq = min(max(Tc, Tg(1)), Tg(end));
Sq = min(max(SOC, Sg(1)), Sg(end));
Rc  = interp2(Sg, Tg, Rcell,     Sq, Tq, 'linear');
dU  = interp2(Sg, Tg, dUdT_cell, Sq, Tq, 'linear');
Rpack      = p.n_series * (Rc / p.n_parallel);
dUdT_pack  = p.n_series * dU;
end

% -----------------------------------------------------------------------
function h = h_external(Ts, Tamb, V, H, p)
Tfilm = 0.5*(Ts+Tamb) + 273.15;
rho   = isa_density(H, Tamb);
mu    = air_mu(Tfilm);
k     = air_k(Tfilm);
cp    = 1006;
Pr    = cp*mu/k;
Re    = rho*V*p.L_rad / max(mu, 1e-9);
if Re < 5e5
    Nu = 0.664*sqrt(Re)*Pr^(1/3);
else
    Nu = max((0.037*Re^0.8 - 871)*Pr^(1/3), 10);
end
h = max(5, min(Nu*k/p.L_rad, 250));
end

% -----------------------------------------------------------------------
function rho = isa_density(H, Tamb)
rho0=1.225; T0=288.15; g=9.80665; R=287.05; gamma=0.0065;
rho0c = rho0*(288.15/(Tamb+273.15));
base  = max(0.1, 1 - gamma*H/T0);
rho   = rho0c * base^(g/(R*gamma)-1);
end

function mu = air_mu(T)
mu0=1.716e-5; T0=273.15; S=110.4;
mu = mu0*(T/T0)^(3/2)*(T0+S)/(T+S);
end

function k = air_k(T)
k = max(0.020, min(0.0241 + 7.73e-5*(T-273.15), 0.040));
end

% -----------------------------------------------------------------------
function [Fdrag, Pdrag] = radiator_drag(Ts, Tamb, V, H, p)
rho  = isa_density(H, Tamb);
mu   = air_mu(0.5*(Ts+Tamb)+273.15);
Re   = rho*V*p.L_rad / max(mu, 1e-9);
if Re < 5e5
    Cf = 1.328/sqrt(max(Re,1));
else
    Cf = 0.074/Re^(1/5);
end
Fdrag = 0.5*rho*V^2*(Cf*p.A_wetted + p.Cd_form*p.A_front);
Pdrag = Fdrag*V;
end

% -----------------------------------------------------------------------
% PATCH 5+6: Ceff_pcm использует ту же f(T), что и пост-обработка,
% поэтому скрытая теплота всегда интегрируется ровно в m*L_pcm.
function C = Ceff_pcm(T, m, p, dTdt)
if nargin < 4, dTdt = 1; end
dTdt = branch_sign_local(dTdt);
a   = f_liquid(T, p, dTdt);
c   = p.c_solid + (p.c_liquid - p.c_solid)*a;
d = df_liquid_dT(T, p, dTdt);
C = max(20, m*(c + p.L_pcm*d));
end

% -----------------------------------------------------------------------
% PATCH 6: f_liquid/df_dT/Ceff используют одни и те же границы перехода.
function f = f_liquid(T, p, dTdt)
if nargin < 3, dTdt = 1; end
[Tsol, Tliq] = pcm_transition_bounds(p, dTdt);
dTm = max(Tliq - Tsol, 1e-6);
f = max(0, min(1, (T - Tsol) / dTm));
end

function df = df_liquid_dT(T, p, dTdt)
[Tsol, Tliq] = pcm_transition_bounds(p, dTdt);
dTm = max(Tliq - Tsol, 1e-6);
df = zeros(size(T));
mask = (T >= Tsol) & (T <= Tliq);
df(mask) = 1 / dTm;
end

function [Tsol, Tliq] = pcm_transition_bounds(p, dTdt)
if branch_sign_local(dTdt) >= 0
    Tsol = p.T_sol;
    Tliq = p.T_liq;
else
    % При кристаллизации сохраняем ту же ширину перехода, но сдвигаем
    % границы вниз, чтобы учесть переохлаждение согласованно во всех функциях.
    Tsol = p.T_sol - 2.0;
    Tliq = p.T_liq - 2.0;
end
end

function s = branch_sign_local(x)
if x >= 0
    s = 1;
else
    s = -1;
end
end

function s = branch_sign_from_samples(Tv, i)
N = length(Tv);
if N <= 1
    s = 1;
elseif i <= 1
    s = branch_sign_local(Tv(2) - Tv(1));
else
    s = branch_sign_local(Tv(i) - Tv(i-1));
end
end

function h = pcm_sensible_enthalpy(T, p, dTdt)
[Tsol, Tliq] = pcm_transition_bounds(p, dTdt);
dTm = max(Tliq - Tsol, 1e-6);
dc = p.c_liquid - p.c_solid;
if T <= Tsol
    h = p.c_solid * T;
elseif T >= Tliq
    h_mid = p.c_solid * Tliq + dc * dTm / 2;
    h = h_mid + p.c_liquid * (T - Tliq);
else
    h = p.c_solid * T + dc * (T - Tsol)^2 / (2 * dTm);
end
end

function E = pcm_stored_energy(T, Tref, m, p, dTdt)
f_now = f_liquid(T, p, dTdt);
f_ref = f_liquid(Tref, p, dTdt);
E = m * (pcm_sensible_enthalpy(T, p, dTdt) - pcm_sensible_enthalpy(Tref, p, dTdt) + ...
         p.L_pcm * (f_now - f_ref));
end

% -----------------------------------------------------------------------
function print_header(txt)
n=72;
fprintf('%s\n  %s\n%s\n\n', repmat('=',1,n), txt, repmat('=',1,n));
end

function s = center_text(txt, W)
txt = char(txt);
l   = max(0, floor((W-length(txt))/2));
s   = [repmat(' ',1,l) txt repmat(' ',1,max(0,W-length(txt)-l))];
end

function s = section_title(txt, W)
txt    = char(txt);
prefix = '  | ';
body   = [prefix txt];
if length(body) > W, body = body(1:W); end
s = [body repmat(' ',1,max(0,W-length(body)))];
end

% -----------------------------------------------------------------------
function print_variable_dictionary_full(p)
fprintf('====================================================================\n');
fprintf(' ДЕТАЛЬНАЯ РАСШИФРОВКА ПЕРЕМЕННЫХ\n');
fprintf('====================================================================\n');
fprintf('[МИССИЯ] t_end=%.1f s\n', p.t_end_sim);
fprintf('[МИССИЯ] takeoff:  %4.0fs @ %.0f W\n', p.t_takeoff,  p.P_takeoff);
fprintf('[МИССИЯ] climb:    %4.0fs @ %.0f W\n', p.t_climb,    p.P_climb);
fprintf('[МИССИЯ] cruise1:  %4.0fs @ %.0f W\n', p.t_cruise1,  p.P_cruise1);
fprintf('[МИССИЯ] maneuver: %4.0fs @ %.0f W\n', p.t_maneuver, p.P_maneuver);
fprintf('[МИССИЯ] cruise2:  %4.0fs @ %.0f W\n', p.t_cruise2,  p.P_cruise2);
fprintf('[МИССИЯ] descent:  %4.0fs @ %.0f W\n', p.t_descent,  p.P_descent);
fprintf('[МИССИЯ] landing:  %4.0fs @ %.0f W\n', p.t_landing,  p.P_landing);
fprintf('[МИССИЯ] V_cruise=%.2f m/s, H_flight=%.0f m\n', p.V_cruise, p.H_flight);
fprintf('[СРЕДА]  T_amb=%.1f C, T0=%.1f C\n', p.T_amb, p.T0);
fprintf('[АКБ]    %dS%dP, U=%.1fV, Q=%.1fAh\n', p.n_series, p.n_parallel, p.U_nom, p.Q_batt_cap/3600);
fprintf('[АКБ]    RC-модель: R1=%.4f Ohm, C1=%.0f F\n', p.R1, p.C1);
fprintf('[ТЕПЛО]  C_core=%.1f, C_shell=%.1f J/K\n', p.C_core, p.C_shell);
fprintf('[ТЕПЛО]  R_core_shell=%.4f, R_shell_pcm1=%.4f K/W\n', p.R_core_shell, p.R_shell_pcm1);
fprintf('[PCM]    m_pcm=%.3f kg, T_sol=%.1f, T_liq=%.1f C\n', p.m_pcm_design, p.T_sol, p.T_liq);
fprintf('[PCM]    L_pcm=%.0f J/kg (гистерезис: суперкулинг -2C)\n', p.L_pcm);
fprintf('[РАД]    A_rad=%.5f m2, epsilon=%.2f\n', p.A_rad, p.epsilon);
fprintf('[АЭРО]   L_rad=%.2f m, A_wetted=%.3f m2, A_front=%.4f m2, Cd=%.2f\n', ...
    p.L_rad, p.A_wetted, p.A_front, p.Cd_form);
fprintf('[ПЛАНЕР] S_ref=%.2f m2, b=%.2f m, e=%.2f, CD0=%.3f\n', ...
    p.S_ref, p.b_span, p.e_oswald, p.CD0);
fprintf('[ПЛАНЕР] eta_prop=%.2f, eta_motor=%.2f\n', p.eta_prop, p.eta_motor);
fprintf('[МАССА]  m_total=%.3f кг (empty=%.2f, payload=%.2f, batt=%.2f)\n', ...
    p.m_total, p.m_empty, p.m_payload, p.m_batt);
fprintf('[ЛИМИТ]  T_opt=%.1f, T_warn=%.1f, T_crit=%.1f C\n', ...
    p.T_opt_thresh, p.T_warn, p.T_crit);
fprintf('====================================================================\n\n');
end

% -----------------------------------------------------------------------
function patch_figure_readability(figHandle)
if nargin < 1 || isempty(figHandle), figHandle = gcf; end
set(figHandle, 'Color', [1 1 1]);
ax = findobj(figHandle, 'Type', 'axes');
for i = 1:numel(ax)
    set(ax(i), 'Color',[1 1 1], 'XColor',[0 0 0], 'YColor',[0 0 0]);
    grid(ax(i), 'on');
end
lgd = findobj(figHandle, 'Type', 'legend');
for k = 1:numel(lgd)
    set(lgd(k), 'Color',[1 1 1], 'TextColor',[0 0 0], 'EdgeColor',[0 0 0], 'Box','on');
end
end

% -----------------------------------------------------------------------
function compare_cooling_variants_4cases(p)
fprintf('\n================== СРАВНЕНИЕ 4 ВАРИАНТОВ ОХЛАЖДЕНИЯ ==================\n');

% A: No PCM + No Radiator
pA = p; mA = 0.0;
pA.A_rad=0.0; pA.epsilon=0.0; pA.A_wetted=0.0; pA.A_front=0.0;

% B: PCM-only
pB = p; mB = p.m_pcm_design;
pB.A_rad=1e-5; pB.epsilon=0.0; pB.A_wetted=0.0; pB.A_front=0.0;

% C: Radiator-only
pC = p; mC = 0.0;

% D: PCM + Radiator
pD = p; mD = p.m_pcm_design;

rA = run_simulation_4node(mA, pA);
rB = run_simulation_4node(mB, pB);
rC = run_simulation_4node(mC, pC);
rD = run_simulation_4node(mD, pD);

kA = calc_kpi_from_result_local(rA, mA, pA);
kB = calc_kpi_from_result_local(rB, mB, pB);
kC = calc_kpi_from_result_local(rC, mC, pC);
kD = calc_kpi_from_result_local(rD, mD, pD);

fprintf('-------------------------------------------------------------------------\n');
fprintf('%-18s | %-8s | %-8s | %-10s | %-11s | %-9s | %-6s\n', ...
    'Variant','TcoreMax','TshellMx','t>45(core)','t>40(shell)','Edrag J','PASS');
fprintf('-------------------------------------------------------------------------\n');
print_variant_line_local('A: NoPCM-NoRad', kA);
print_variant_line_local('B: PCM-only',    kB);
print_variant_line_local('C: Rad-only',    kC);
print_variant_line_local('D: PCM+Rad',     kD);
fprintf('-------------------------------------------------------------------------\n');

fprintf('\nСРАВНЕНИЕ ОТНОСИТЕЛЬНО БАЗЫ A:\n');
print_delta_local('B', kA, kB);
print_delta_local('C', kA, kC);
print_delta_local('D', kA, kD);

fid = fopen('compare_cooling_variants_4cases.csv','w');
fprintf(fid,'variant,Tcore_max_C,Tshell_max_C,t_above_45_core_s,t_above_40_shell_s,Egen_J,Econv_J,Erad_J,Elat_J,Estored_J,Edrag_J,residual_pct,pass\n');
write_row_local(fid, 'A_NoPCM_NoRad', kA);
write_row_local(fid, 'B_PCM_only',    kB);
write_row_local(fid, 'C_Rad_only',    kC);
write_row_local(fid, 'D_PCM_Rad',     kD);
fclose(fid);
fprintf('CSV: compare_cooling_variants_4cases.csv\n');
fprintf('=========================================================================\n\n');
end

% -----------------------------------------------------------------------
function k = calc_kpi_from_result_local(r, m_pcm, p)
t  = r.t;
Tc = r.T_core;
Ts = r.T_shell;
T1 = r.T_pcm1;
T2 = r.T_pcm2;
Vrc = r.V_RC;
N  = length(t);

Qgen=zeros(N,1); Qconv=zeros(N,1); Qrad=zeros(N,1);
Pdrag=zeros(N,1); f1=zeros(N,1); f2=zeros(N,1);

for i = 1:N
    [I, SOC]      = mission_current_soc(t(i), p);
    [Rpack, dUdT] = battery_maps(Tc(i), SOC, p);
    Qgen(i)  = I^2*Rpack + Vrc(i)*I + I*(Tc(i)+273.15)*dUdT;
    h        = h_external(T2(i), p.T_amb, p.V_cruise, p.H_flight, p);
    Qconv(i) = h*p.A_rad*(T2(i)-p.T_amb);
    Qrad(i)  = p.epsilon*p.sigma_SB*p.A_rad*((T2(i)+273.15)^4-(p.T_amb+273.15)^4);
    [~,Pdrag(i)] = radiator_drag(T2(i), p.T_amb, p.V_cruise, p.H_flight, p);
    f1(i) = f_liquid(T1(i), p, branch_sign_from_samples(T1, i));
    f2(i) = f_liquid(T2(i), p, branch_sign_from_samples(T2, i));
end

Egen  = trapz(t, Qgen);
Econv = trapz(t, Qconv);
Erad  = trapz(t, Qrad);
Edrag = trapz(t, Pdrag);

m1   = m_pcm * p.pcm_split;
m2   = m_pcm * (1-p.pcm_split);
dT1_sign_end = branch_sign_from_samples(T1, N);
dT2_sign_end = branch_sign_from_samples(T2, N);
Elat = m1*p.L_pcm*(f1(end)-f1(1)) + m2*p.L_pcm*(f2(end)-f2(1));
Epcm1 = pcm_stored_energy(T1(end), T1(1), m1, p, dT1_sign_end);
Epcm2 = pcm_stored_energy(T2(end), T2(1), m2, p, dT2_sign_end);
Estored = p.C_core*(Tc(end)-p.T0) + p.C_shell*(Ts(end)-p.T0) + Epcm1 + Epcm2;
resid = abs(Egen-(Econv+Erad+Estored)) / max(Egen,1) * 100;

dt = mean(diff(t));
k = struct();
k.Tcore_max        = max(Tc);
k.Tshell_max       = max(Ts);
k.t_above_45_core  = sum(Tc>45)*dt;
k.t_above_40_shell = sum(Ts>40)*dt;
k.Egen  = Egen;  k.Econv = Econv;
k.Erad  = Erad;  k.Elat  = Elat;
k.Estored = Estored; k.Edrag = Edrag; k.resid = resid;
end

function print_variant_line_local(name, k)
pf = (k.Tcore_max<=45) && (k.Tshell_max<=40);
fprintf('%-18s | %8.2f | %8.2f | %10.1f | %11.1f | %9.1f | %-6s\n', ...
    name, k.Tcore_max, k.Tshell_max, k.t_above_45_core, k.t_above_40_shell, ...
    k.Edrag, ternary_local(pf,'PASS','FAIL'));
end

function print_delta_local(tag, kRef, kX)
fprintf('%s: ΔTcore=%+7.2f°C | Δt>45=%+8.1f s | ΔEdrag=%+9.1f J\n', ...
    tag, kX.Tcore_max-kRef.Tcore_max, ...
    kX.t_above_45_core-kRef.t_above_45_core, ...
    kX.Edrag-kRef.Edrag);
end

function write_row_local(fid, name, k)
pf = (k.Tcore_max<=45) && (k.Tshell_max<=40);
fprintf(fid,'%s,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%s\n', ...
    name, k.Tcore_max, k.Tshell_max, k.t_above_45_core, k.t_above_40_shell, ...
    k.Egen, k.Econv, k.Erad, k.Elat, k.Estored, k.Edrag, k.resid, ternary_local(pf,'PASS','FAIL'));
end

function out = ternary_local(cond, a, b)
if cond, out = a; else, out = b; end
end