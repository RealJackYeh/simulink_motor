clear all; close all;
% 變頻器＆馬達參數 --------------------------------------------------------
% 馬達銘牌值 -------------------------------------------------------------
vdc=300;
Pb = 3700;            % 馬達銘牌額定功率, 3kW
Ibr = 14;             % 馬達銘銘額定電流, 23A
Vbr = Pb/(3*Ibr);     % 相電壓基準值 有效值, V
Poles = 6;            % 極數
Speed_rated = 1800;   % 馬達額定轉速 rpm, 1500 RPM
w_rated = Speed_rated*2*pi/60;  % 馬達額定轉速 rad/s, 0.1 rad/s
% 電氣基準值 -------------------------------------------------------------
Tb = Pb/w_rated;      % 馬達額定轉矩, Nm
wb = w_rated*Poles/2; % 馬達額定電轉速, rad/s
Vb = 1.414*Vbr;       % 相電壓基準值 峰值, V
Ib = 1.414*Ibr;       % 相電流基準值 峰值, A 
zb = Vbr/Ibr;         % 基準阻抗值, ohm      
lamb = Vb/wb;         % 磁通鏈基準值, V/rad/s
Lb = lamb/Ib;         % 電感基準值, H  
% 設定的馬達參數 ---------------------------------------------------------
Lq=0.0149;            % q軸電感, H 
Ld=0.006;             % d軸電感, 10uH 
ro=Lq/Ld;             % 凸極比，saliency ratio
lamaf=0.2739;         % 電達反電動勢常數, V/rad/s 
Rs=0.348;             % 馬達定子電阻, Ohm
B=0.0005;             % 摩擦系數 Nm/rad/s
J=0.001;               % 慣量 kg-m^2
tln=0.5;              % 負載轉矩, pu
%-------------------------------------------------------------------------
f_cmd = 90;          % 速度命令 Hz
wr_ref = 2*pi*f_cmd;  % 速度命令 rad/s
% 控制器參數---------------------------------------------------------------
fc=8000;
Kpi=10;
% ASR參數----------------------
Kp=2;
Ki=10;
%------------------------------
% ACR參數----------------------
Kp_id=20;
Ki_id=20;
Kp_iq=20;
Ki_iq=20;
%------------------------------
%-------------------------------------------------------------------------
% 模擬參數初始化 ----------------------------------------------------------
theta_r = 0;
wr = 0;
t = 0;
dt = 1e-6;
tfinal = 0.5;
if_ref = -1e-16;
iqs = 0;  ids = 0;
vqs = 0; vds = 0;
Tl= 0.3*Tb;   % 負載轉矩
n = 1;
x = 1;
signe = 1;
carrier = -1;
ias=0; ibs=0; ics=0; t1=0;
vax1=0; vbx1=0; vcx1=0;
vao1=0;
zia=0; zib=0; zic=0;
y = 0;
w = 0;
z = 0;
vax_o = 0;
id_ref = 0;         % d軸電電命令為零
% -----------------------------------------------------------------------
% 模擬回路開始 ------------------------------------------------------------
while (t<tfinal)
   % if t > 0.02
   %     wr_ref = -314.3;
   % end
    % speed loop
    wr_err = wr_ref - wr; 
    y = y + wr_err*dt;  % 速度誤差積分 
    Te_ref = Kp*wr_err + Ki*y;  % ASR PI控制器輸出
    iq_ref = Te_ref;    % 轉矩命令當作q軸電流命令

    % 電流限制器，不能超過二倍的馬達額定電流
    if iq_ref > 2*Ib
        iq_ref = 2*Ib;
    end
    if iq_ref < -2*Ib
        iq_ref = -2*Ib;
    end      
    % 電流回路 -------------------------------------------------
    iq_err = iq_ref - iqs;
    w = w + iq_err*dt;
    Vq_ref = Kp_iq*iq_err + Ki_iq*w;  % q軸電流PI輸出q軸電壓命令
    id_err = id_ref - ids;
    z = z + id_err*dt;
    Vd_ref = Kp_id*id_err + Ki_id*z;  % d軸電流PI輸出d軸電壓命令
    %----------------------------------------------------------
    % 產生三相靜止座標電壓命令
    Vas_ref = Vd_ref*cos(theta_r) - Vq_ref*sin(theta_r);
    Vbs_ref = Vd_ref*cos(2*pi/3 - theta_r) + Vq_ref*sin(2*pi/3 - theta_r);
    Vcs_ref = Vd_ref*cos(4*pi/3 - theta_r) + Vq_ref*sin(4*pi/3 - theta_r);
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
    if Vas_ref < -150 %相電壓最大值不超過vdc/2
        Vas_ref = -150;
    end 
    if Vbs_ref < -150 %相電壓最大值不超過vdc/2
        Vbs_ref = -150;
    end 
    if Vcs_ref < -150 %相電壓最大值不超過vdc/2
        Vcs_ref = -150;
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
    % 計算相應進入馬達的 d, q 軸電壓
    vds = (2/3)*(cos(theta_r)*vas + cos(2*pi/3 - theta_r)*vbs + cos(4*pi/3 - theta_r)*vcs);
    vqs = (2/3)*(-sin(theta_r)*vas + sin(2*pi/3 - theta_r)*vbs + sin(4*pi/3 - theta_r)*vcs);
    % 永磁馬達d, q軸模型----------------------------------------------------
    d_iqs = (vqs - Rs*iqs - wr*Ld*ids - wr*lamaf)*dt/Lq;
    iqs = iqs + d_iqs;
    d_ids = (vds + wr*Lq*iqs - Rs*ids)*dt/Ld;
    ids = ids + d_ids;
    %---------------------------------------------------------------------
    Te = (3/2)*(Poles/2)*iqs*((Ld-Lq)*ids + lamaf); % 計算電磁轉矩
    d_wr = ((Poles/2) * (Te-Tl) - B*wr) * dt/J; % 機械方程式，計算轉速
    wr = wr + d_wr;
    d_theta_r = wr * dt; 
    theta_r = theta_r + d_theta_r;  % 速度積分計算轉子角度
    % 計算三相輸出電流------------------------------------------------------
    ias = ids*cos(theta_r) - iqs*sin(theta_r);
    ibs = ids*cos(2*pi/3 - theta_r) + iqs*sin(2*pi/3 - theta_r);
    ics = ids*cos(4*pi/3 - theta_r) + iqs*sin(4*pi/3 - theta_r);
    %---------------------------------------------------------------------
    %產生 PWM 載波---------------------------------------------------------
    carrier =  150 * signe * (2/(1/(2*fc)))*dt + carrier;
    if carrier > 150
        signe = -1;
    end
    if carrier < -150
        signe = 1;
    end
    %---------------------------------------------------------------------
    t = t + dt;
    t1 = t1 + dt;
    % 將監看變數存入陣列,並標么化--------------------------------------------
    if x>16
        t  % 命令視窗輸出時間 
        tn(n) = t; 
       % Te_refn(n) = Te_ref/Tb;
        iq_refn(n) = iq_ref/Ib;
        id_refn(n) = id_ref/Ib;
        iasn(n) = ias/Ib;
        ibsn(n) = ibs/Ib;
        icsn(n) = ics/Ib;
        Vd_refn(n) = Vd_ref/Vb;
        Vq_refn(n) = Vq_ref/Vb;
        vasn(n) = vas/Vb;
        vbsn(n) = vbs/Vb;
        vcsn(n) = vcs/Vb;
        iqsn(n) = iqs/Ib;   
        idsn(n) = ids/Ib;
        Ten(n) = Te/Tb;
        wrn(n) = wr/wb;
        wrrefn(n) = wr_ref/wb;
        lammn(n) = sqrt((1+Ld*ids/(Ib*Lb))^2 + (Lq*iqs/(Ib*Lb))^2);
        vax_o(n) = vax/Vb; 
        n = n + 1;
        x = 1;
    end
    x = x + 1;
    %---------------------------------------------------------------------
end
%結束模擬
%-------------------------------------------------------------------------
%繪圖----------------------------------------------------------------------
figure(1); orient tall;
subplot(4,2,1)
plot(tn, wrrefn, 'k--', tn, wrn, 'k'); axis([0 tfinal -1.2 1.2]);
subplot(4,2,2)
plot(tn, id_refn, 'k--', tn, idsn, 'k'); axis([0 tfinal -1.2 1.2]);
subplot(4,2,3)
plot(tn, Ten, 'k'); axis([0 0.05 -2.1 2.1]);
subplot(4,2,4)
plot(tn, Vd_refn, 'k', tn, Vq_refn, 'k--'); axis([0 tfinal -1.5 1.5]);
subplot(4,2,5)
plot(tn, vax_o, 'k'); axis([0 tfinal -2 2]);
subplot(4,2,6)
plot(tn, iasn, 'k', tn, ibsn, 'k--', tn, icsn, 'k:'); axis([0 tfinal -1.5 1.5]);
subplot(4,2,7)
plot(tn, iq_refn, 'k--', tn, iqsn, 'k'); axis([0 tfinal -2 2]);
subplot(4,2,8)
plot(tn, lammn, 'k'); axis([0 tfinal 0 2]);
%-------------------------------------------------------------------------
        
        