clear all; close all;
% 跑繵竟‘筿诀把计 --------------------------------------------------------
vdc=300;
Pb = 2200; % Base power, 1Watt
Ibr = 8; % rated current, 0.1A
P = 4; % number of poles
Speed_rated = 1720; % rated motor speed, 1 RPM
w_rate = roundn(Speed_rated*2*pi/60, 0);  % motor rated speed, 0.1 rad/s
% Tb = Pb/w_rate;   % Base torque, 0.1 Nm
% wb = w_rate*P/2;   % Base speed, 0.1 rad/s
% Vbr = Pb/(3*Ibr);   % Rated voltage, 0.1V
% Vb = 1.414*Vbr;     % Base voltage, 0.1V
% Ib = 1.414*Ibr;     % Base currrent, 0.1A 
% zb = Vb/Ib;         % Base impedance, 0.001ohm      
% lamb = Vb/wb;     % Base flux linkage, 0.001 V/rad/s
% lb = lamb/Ib;       % Base inductance,10uH  
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

% % 皑笷夹把计 -----------------------------------------------------------
%  IbPUQ = 2^13;  % Q13
% % lqPU = lq/lb;  lqPUQ = lqPU * 2^13;                % Q13
% % ldPU = ld/lb;  ldPUQ = ldPU * 2^13;                % Q13
% RsPU = Rs/zb;    RsPUQ = RsPU * 2^13;                % Q13
% % lamafPU = lamaf/lamb;  lamafPUQ = lamafPU * 2^13;  % Q13
% wr_refPU = 3600/wb; wr_refPUQ = wr_refPU * 2^13;   % Q13
% %------------------------------------------------------------------------

% 北竟把计---------------------------------------------------------------
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

% 家览﹍把计 ------------------------------------------------------------
wr_ref = 314.3;
theta_r = 0; theta_e = 0;
wr = 0;
wrm = 0;
t = 0;
dt = 1e-5;
tfinal = 0.5;
id_ref = 4;
ids = 0;  iqs = 0; idse = 0; iqse = 0; idr = 0;  iqr = 0;
vqs = 0; vds = 0;
Tl= 0;
n = 1; x = 1; signe = 1; carrier = -1500;
ias=0; ibs=0; ics=0; t1=0;
vax1=0; vbx1=0; vcx1=0; vao1=0;
zia=0; zib=0; zic=0;
y = 0; w = 0; z = 0; i=0;
% -----------------------------------------------------------------------

% 家览隔秨﹍ ------------------------------------------------------------
while (t<tfinal)
    i = i+1;
    % if t > 0.02
    %     wr_ref = -314.3;
    % end
    % speed loop
%     wrPU = (wr*10/wb) ;
    wr_err = wr_ref - wr; 
    y = y + wr_err*dt;
    Te_ref = Kp*wr_err + Ki*y;
    iq_ref = Te_ref;
     % limiter
    if iq_ref > 16
        iq_ref = 16;
    end
    if iq_ref < -16
        iq_ref = -16;
    end      
    is_ref = sqrt(id_ref^2 + iq_ref^2);
    % current loop for iq and id

    iq_err = iq_ref - iqse;
    w = w + iq_err*dt;
    Vq_ref = Kp_iq*iq_err + Ki_iq*w;
    
    id_err = id_ref - idse;
    z = z + id_err * dt;
    Vd_ref = Kp_id * id_err + Ki_id*z;  
    
    vds_cmd = Vd_ref*cos(theta_e) - Vq_ref*sin(theta_e);
    vqs_cmd = Vd_ref*sin(theta_e) + Vq_ref*cos(theta_e);    

%     theta_v = atan2(Vq_ref, Vd_ref);
%     Vs_refPU = sqrt(Vd_refPU^2 + Vq_refPU^2);
%     % voltage reference phase voltage
%     Vs_ref = roundn((Vs_refPU * Vb), 0);
%     Vas_ref = Vs_ref * sin(theta_r + theta_v);
%     Vbs_ref = Vs_ref * sin(theta_r + theta_v - 2*pi/3);
%     Vcs_ref = Vs_ref * sin(theta_r + theta_v + 2*pi/3);
%     % calculate control voltages to be applied to PWM controller
%     if Vas_ref > 1500
%         Vas_ref = 1500;
%     end 
%     if Vbs_ref > 1500
%         Vbs_ref = 1500;
%     end 
%     if Vcs_ref > 1500
%         Vcs_ref = 1500;
%     end 
%     vax = Vas_ref;
%     vbx = Vbs_ref;
%     vcx = Vcs_ref;
%     % sample and hold
%     if t1 > 1/fc
%         vax1 = vax;
%         vbx1 = vbx;
%         vcx1 = vcx;
%         t1 = 0;
%     end
%     % PWM controller
%     if vax1 >= carrier
%         vao = vdc/2;
%     elseif vax1 < carrier
%         vao = -vdc/2;
%     end
%     if vbx1 >= carrier
%         vbo = vdc/2;
%     elseif vbx1 < carrier
%         vbo = -vdc/2;
%     end
%     if vcx1 >= carrier
%         vco = vdc/2;
%     elseif vcx1 < carrier
%         vco = -vdc/2;
%     end
%     % compute line voltage
%     vab = vao - vbo;
%     vbc = vbo - vco;
%     vca = vco - vao;
%     % compute phase voltage
%     %vas = (vab - vca)/3;
%     %vbs = (vbc - vab)/3;
%     %vcs = (vca - vbc)/3;
%     vas = 150 * sin(377*t);
%     vbs = 150 * sin(377*t - 2*pi/3);
%     vcs = 150 * sin(377*t +  2*pi/3);
%     
%     % q and d axes voltages
%     %vqs = (2/3)*(cos(theta_r)*vas + cos(theta_r-2*pi/3)*vbs + cos(theta_r+2*pi/3)*vcs);
%     %vds = (2/3)*(sin(theta_r)*vas + sin(theta_r-2*pi/3)*vbs + sin(theta_r+2*pi/3)*vcs);
    vds = vds_cmd;
    vqs = vqs_cmd;
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
    d_theta_e = theta_e + d_theta_e; 
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

        iasn(n) = ias;
        ibsn(n) = ibs;
        icsn(n) = ics;
        Vd_refn(n) = Vd_ref;
        Vq_refn(n) = Vq_ref;
%         vasn(n) = vas;
%         vbsn(n) = vbs;
%         vcsn(n) = vcs;
        iqsn(n) = iqs;   
        idsn(n) = ids;
        isn(n) = is;
        Ten(n) = Te;
        wrm1(n) = wrm;
        wrrefn(n) = wr_ref;
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


