from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
Rs = 2.5/3
Rr = 1.83/3
Ls = 0.2557/3
Lr = 0.2557/3
Lm = 0.245/3
pole = 4
J = 0.033
B = 0.00825
D = Ls*Lr - Lm**2
sigma = 1 - Lm**2/(Ls*Lr)
Lsigma = sigma*Ls
Tr = Lr/Rr
print('q-asix current pi control')
pi_q = signal.TransferFunction([100, 100*119], [1, 0])  #連續時間轉移函數
print(pi_q)
dt = 1/1000   # 採樣時間 1/1000Hz
dpi_q = pi_q.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(dpi_q) 
b = dpi_q.num  #分母係數
a = dpi_q.den #分子係數
print('filter coefficient b_i: ' + str(b))
print('filter coefficient a_i: ' + str(a[1:]))

print('\n speed loop pi control')
pi_s = signal.TransferFunction([10, 0.25*10], [1, 0])  #連續時間轉移函數
print(pi_s)
dt = 1/1000   # 採樣時間 1/1000Hz
dpi_s = pi_s.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(dpi_s) 
b = dpi_s.num  #分母係數
a = dpi_s.den #分子係數
print('filter coefficient b_i: ' + str(b))
print('filter coefficient a_i: ' + str(a[1:]))

print('\n phi loop pi control')
pi_phi = signal.TransferFunction([10, 7.15*10], [1, 0])  #連續時間轉移函數
print(pi_phi)
dt = 1/1000   # 採樣時間 1/1000Hz
dpi_phi = pi_phi.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(dpi_phi) 
b = dpi_phi.num  #分母係數
a = dpi_phi.den #分子係數
print('filter coefficient b_i: ' + str(b))
print('filter coefficient a_i: ' + str(a[1:]))

print('\n q-asix current plant')
iq_plant = signal.TransferFunction([1], [1, Rs/(sigma*Ls)])  #連續時間轉移函數
print(iq_plant)
dt = 1/1000   # 採樣時間 1/1000Hz
diq_plant = iq_plant.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(diq_plant) 

print('\nphi estimator')
phi_estimator = signal.TransferFunction([Lm], [Lr/Rr, 1])  #連續時間轉移函數
print(phi_estimator)
dt = 1/1000   # 採樣時間 1/1000Hz
dphi_estimator = phi_estimator.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(dphi_estimator) 
b = dphi_estimator.num  #分母係數
a = dphi_estimator.den #分子係數
print('filter coefficient b_i: ' + str(b))
print('filter coefficient a_i: ' + str(a[1:]))
print('\n 1/s integrator')
integrator = signal.TransferFunction([1], [1, 0])  #連續時間轉移函數
print(integrator)
dt = 1/1000   # 採樣時間 1/1000Hz
dintegrator = integrator.to_discrete(dt, method='gbt', alpha=0.5) #將連續時間轉移函數數位化，使用雙線性轉換
print(dintegrator) 
b = dintegrator.num  #分母係數
a = dintegrator.den #分子係數
print('filter coefficient b_i: ' + str(b))
print('filter coefficient a_i: ' + str(a[1:]))