% Calculo_estabilidad.m:
%
% Script que representa las respuestas a diferentes entradas para un
% controlador D|PID y analizar su estabilidad
%
%
% Fecha: 01/06/20     Version: 1.0
% Copyright: Alfonso Moreno Sanz

% clear all;
% close all;
%% Parametros configurables
k_p =1;
tau_D1 = 1;
tau_D2 = 64.986/2652.28;
%tau_D2 = 1;
tau_I = 0.1;
entrada= 1; %0 escalon, 1 rampa y 2 parabola
k = 2652.28;
p = 64.986;
n_figura = 1;
%% Representación controlador D|PID
num_control_d_pid = [(k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
den_control_d_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
H_d_pid = tf(num_control_d_pid,den_control_d_pid);% construir una funcción de transferencia
%impulse(H_p);
if (entrada == 1)
figure(n_figura); hold on
step(H_d_pid);
title("Controlador D|PID, respuesta a escalón") 
n_figura = n_figura +1;
end
if(entrada == 1)
 t=0:0.1:10;
 u = t;
 [y,x]=lsim(num_control_d_pid,den_control_d_pid,u,t);
 u=t;
 figure(n_figura); hold on
 plot(t,y,t,u);
 title("Controlador D|PID, respuesta a rampa") 
n_figura = n_figura +1;
end
if(entrada == 1)
 t=0:0.1:10;
 u=(t.^2)/100;
 [y,x]=lsim(num_control_d_pid,den_control_d_pid,u,t);
 figure(n_figura); hold on
 plot(t,y,t,u);
 title("Controlador D|PID, respuesta a parabola") 
n_figura = n_figura +1;
end
%rlocus(H_d_pid);