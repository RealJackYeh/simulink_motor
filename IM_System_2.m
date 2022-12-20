clear all; close all;
% 變頻器＆電機參數 --------------------------------------------------------
vdc = 300;
Pb = 2200; % Base power, 1Watt
Ibr = 8; % rated current, 0.1A
P = 4; % number of poles
Speed_rated = 1720; % rated motor speed, 1 RPM
w_rate = roundn(Speed_rated*2*pi/60, 0);  % motor rated speed, 0.1 rad/s
Tb = Pb/w_rate;   % Base torque, 0.1 Nm
wb = w_rate*P/2;   % Base speed, 0.1 rad/s
Vbr = Pb/(3*Ibr);   % Rated voltage, 0.1V
Vb = 1.414*Vbr;     % Base voltage, 0.1V
Ib = 1.414*Ibr;     % Base currrent, 0.1A 
zb = Vb/Ib;         % Base impedance, 0.001ohm      
lamb = Vb/wb;     % Base flux linkage, 0.001 V/rad/s
lb = lamb/Ib;       % Base inductance,10uH  
% motor parameters
Rs = 2.5/3;  
Rr = 1.83/3;
%Llkg = 4.34e-3;
Lm = 0.245/3;
Ls = 0.2557/3;
Lr = Ls;
Lsigma = Ls - Lm^2/Lr;
Sigma = 1 - Lm^2/(Ls*Lr);
D = Ls*Lr - Lm^2;
Tr = Lr/Rr;
% ws=(2*pi*1500/60)*P/2; % Stator frequency, rad/sec
B=0.00825;   % Friction coefficient of load and machine
J=0.013;   % Moment of inertia of load and machine
tln=0.5;         % load torque, p.u.
Omega = 0;
%-----------------------------------------------------------------------

% 馬達標么參數 -----------------------------------------------------------
 IbPUQ = 2^13;  % Q13
% lqPU = lq/lb;  lqPUQ = lqPU * 2^13;                % Q13
% ldPU = ld/lb;  ldPUQ = ldPU * 2^13;                % Q13
RsPU = Rs/zb;    RsPUQ = RsPU * 2^13;                % Q13
% lamafPU = lamaf/lamb;  lamafPUQ = lamafPU * 2^13;  % Q13
wr_refPU = 3600/wb; wr_refPUQ = wr_refPU * 2^13;   % Q13
%------------------------------------------------------------------------

% 控制器參數---------------------------------------------------------------
% deli = 0.1*Ib;
fc=8000;
Kpi=10;
Kp=10;
Ki=100;
Kp_id=1;
Ki_id=30;
Kp_iq=1;
Ki_iq=30;
%------------------------------------------------------------------------

% 模擬初始參數 ------------------------------------------------------------
wr_ref = 314.3;
theta_e = 0;
wr = 0;
wrm = 0;
t = 0;
dt = 1e-5;
tfinal = 0.5;
id_refPU = 0.5;
ids = 0;  iqs = 0; idse = 0; iqse = 0; idr = 0;  iqr = 0;
vqs = 0; vds = 0;
Tl= 0*Tb/10;
n = 1; x = 1; signe = 1; carrier = -1500;
ias=0; ibs=0; ics=0; t1=0;
vax1=0; vbx1=0; vcx1=0; vao1=0;
zia=0; zib=0; zic=0;
y = 0; w = 0; z = 0; i=0;
% -----------------------------------------------------------------------

% 模擬回路開始 ------------------------------------------------------------
while (t<tfinal)
    i = i+1;
    % if t > 0.02
    %     wr_ref = -314.3;
    % end
    % speed loop
    wrPU = (wr*10/wb) ;
    wr_errPU = wr_refPU - wrPU; 
    y = y + wr_errPU*dt;
    Te_refPU = Kp*wr_errPU + Ki*y;
    iq_refPU = Te_refPU;
     % limiter
    if iq_refPU > 2
        iq_refPU = 2;
    end
    if iq_refPU < -2
        iq_refPU = -2;
    end      
    is_refPU = sqrt(id_refPU^2 + iq_refPU^2);
    % current loop for iq and id
    iqsPU = (iqse*10/Ib) ;
    iq_errPU = iq_refPU - iqsPU;
    w = w + iq_errPU*dt;
    Vq_refPU = Kp_iq*iq_errPU + Ki_iq*w;
    
    idsPU = (idse*10/Ib);
    id_errPU = id_refPU - idsPU;
    z = z + id_errPU * dt;
    Vd_refPU = Kp_id * id_errPU + Ki_id*z;  
    theta_v = atan2(Vq_refPU, Vd_refPU);
    Vs_refPU = sqrt(Vd_refPU^2 + Vq_refPU^2);
    % voltage reference phase voltage
    Vas_ref = (cos(theta_e) * Vd_refPU - sin(theta_e) * Vq_refPU) * Vb;
    Vbs_ref = (cos(2*pi/3 - theta_e) * Vd_refPU + sin(2*pi/3 - theta_e) * Vq_refPU) * Vb;
    Vcs_ref = (cos(4*pi/3 - theta_e) * Vd_refPU + sin(4*pi/3 - theta_e) * Vq_refPU) * Vb;
    %SVPWM 變頻器-------------------------------------------------
    % calculate control voltages to be applied to PWM controller
    if Vas_ref > 150 %相電壓最大值不超過vdc/2
        Vas_ref = 150;
    end 
    if Vbs_ref > 150 %相電壓最大值不超過vdc/2
        Vbs_ref = 150;
    end 
    if Vcs_ref > 150 %相電壓最大值不超過vdc/2
        Vcs_ref = 150;
    end 
    %SVPWM 調變----------------------------------------------------
    Vx_max = max([Vas_ref,Vbs_ref,Vcs_ref]);
    Vx_min = min([Vas_ref,Vbs_ref,Vcs_ref]);
    vax = Vas_ref-(Vx_max+Vx_min)/2;
    vbx = Vbs_ref-(Vx_max+Vx_min)/2;
    vcx = Vcs_ref-(Vx_max+Vx_min)/2;
    %--------------------------------------------------------------
    % 取樣 ＆ 保持 
    if t1 > 1/fc
        vax1 = vax;
        vbx1 = vbx;
        vcx1 = vcx;
        t1 = 0;
    end
    % PWM 控制器---------------------------------------------------- 
    % o點為二電容間的中性點
    if vax1 >= carrier
        vao = vdc/2;  
    elseif vax1 < carrier
        vao = -vdc/2;
    end
    if vbx1 >= carrier
        vbo = vdc/2;
    elseif vbx1 < carrier
        vbo = -vdc/2;
    end
    if vcx1 >= carrier
        vco = vdc/2;
    elseif vcx1 < carrier
        vco = -vdc/2;
    end
    % 計算線電壓 vab, vbc, vca
    vab = vao - vbo;
    vbc = vbo - vco;
    vca = vco - vao;
    % 計算相電壓 vas, vbs, vcs
    vas = (vab - vca)/3;
    vbs = (vbc - vab)/3;
    vcs = (vca - vbc)/3;
    % q and d axes voltages
    vds = (2/3) * (vas - 0.5*vbs - 0.5*vcs);
    vqs = (2/3) * (sqrt(3)*vbs/2 - sqrt(3)*vcs/2);
    % machine equation
    d_ids = (-Rs*ids/Lsigma + (Omega + (1-Sigma)/Sigma*wr)*iqs + Lm/(Lsigma*Tr)*idr + Lm*wr/Lsigma*iqr + vds/Lsigma)*dt;
    ids = ids + d_ids;
    d_iqs = ((-Omega - (1-Sigma)/Sigma*wr)*ids - Rs/Lsigma*iqs - wr*Lm/Lsigma*idr + Lm/(Lsigma*Tr)*iqr + vqs/Lsigma)*dt;
    iqs = iqs + d_iqs;
    d_idr = (Rs*Lm/(Lsigma*Lr)*ids - wr*Lm/(Sigma*Lr)*iqs - 1/(Sigma*Tr)*idr + (Omega - wr/Sigma)*iqr - Lm*vds/(Lsigma*Lr))*dt;
    idr = idr + d_idr;
    d_iqr = (wr*Lm/(Sigma*Lr)*ids + Rs*Lm/(Lsigma*Lr)*iqs + (-Omega + wr/Sigma)*idr - iqr/(Sigma*Tr) - Lm*vqs/(Lsigma*Lr))*dt;
    iqr = iqr + d_iqr;
    % calculate state current and torque angle
    is = sqrt(iqs^2 + ids^2);
    %delta = atan(iqs/ids);
    % calculate torque 
    Te = (3/2)*(P/2)*Lm*(iqs*idr - ids*iqr);
    % calculate speed and position
    d_wrm = ((Te-Tl) - B*wrm) * dt/J;
    wrm = wrm + d_wrm;
    wr = (P/2)*wrm;
    % calculate slip frequency of rotor-oriented FOC
    wsl = Rr*iqse/(Lr*idse);
    we = wr + wsl;
    d_theta_e = we * dt;
    theta_e = theta_e + d_theta_e; 
    % 3 phase current feecback
    ias = ids;
    ibs = -0.5*ids + sqrt(3)*iqs/2;
    ics = -0.5*ids - sqrt(3)*iqs/2;
     % 2 phase current feecback
    idse = ids*cos(theta_e) + iqs*sin(theta_e);
    iqse = -ids*sin(theta_e) + iqs*cos(theta_e);
    % PWM ramp
    carrier =  1500 * signe * (2/(1/(2*fc)))*dt + carrier;
    if carrier > 1500
        signe = -1;
    end
    if carrier < -1500
        signe = 1;
    end
    t = t + dt;
    t1 = t1 + dt;
    carrier1(i) = carrier;
    % plot variables after normalization
    if x>16
        t
        tn(n) = t;
       % Te_refn(n) = Te_ref/Tb;
        iq_refn(n) = iq_refPU;
        id_refn(n) = id_refPU;
        is_refn(n) = is_refPU;
        iasn(n) = ias/Ib;
        ibsn(n) = ibs/Ib;
        icsn(n) = ics/Ib;
        Vd_refn(n) = Vd_refPU;
        Vq_refn(n) = Vq_refPU;
        vasn(n) = vas/Vb;
        vbsn(n) = vbs/Vb;
        vcsn(n) = vcs/Vb;
        iqsn(n) = iqs/Ib;   
        idsn(n) = ids/Ib;
        isn(n) = is/Ib;
        Ten(n) = Te/Tb;
        wrm1(n) = wrm;
        wrrefn(n) = wr_ref/wb;
        wr1(n) = wr;
       % lammn(n) = sqrt((1+ld*ids*10/(Ib*lb))^2 + (lq*iqs*10/(Ib*lb))^2);
        n = n + 1;
        x = 1;
    end
    x = x + 1;
end
figure(1); orient tall;
subplot(4,2,3)
plot(tn, Ten, 'k'); axis([0 tfinal -2.1 2.1]);
set(gca, 'xticklabel', []);
subplot(4,2,5)
plot(tn, is_refn, 'k--', tn, isn, 'k'); axis([0 tfinal 0 2]);
set(gca, 'xticklabel', []);
subplot(4,2,1)
plot(tn, wrm1, 'k'); axis([0 tfinal -200 200]);
set(gca, 'xticklabel', []);
subplot(4,2,7)
plot(tn, iq_refn, 'k--', tn, iqsn, 'k'); axis([0 tfinal -2 2]);
subplot(4,2,2)
plot(tn, id_refn, 'k--', tn, idsn, 'k'); axis([0 tfinal -1 1]);
set(gca, 'xticklabel', []); 
subplot(4,2,4)
plot(tn, Vd_refn, 'k', tn, Vq_refn, 'k--'); axis([0 tfinal -1.5 1.5]);
set(gca, 'xticklabel', []);
subplot(4,2,6)
plot(tn, iasn, 'k', tn, ibsn, 'k--', tn, icsn, 'k:'); axis([0 tfinal -4 4]);
set(gca, 'xticklabel', []);
subplot(4,2,8)
plot(tn, vasn, 'k', tn, vbsn, 'k--', tn, vcsn, 'k:'); axis([0 tfinal -4 4]);


