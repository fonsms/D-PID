% Funciones_transferencia.m:
%
% Script que representa las funciones de transferencia para los
% controladores P, PI, P-D, PD,PID, PI-D, PID-D y D|PID
%
% Fecha: 01/06/20     Version: 1.0
% Copyright: Alfonso Moreno Sanz

%clear all;
 %close all;
%% Parametros configurables
k_p = 5;
tau_D1 = 1;
tau_D2 = 1;
tau_I = 1
entrada= 0; %0 escalon, 1 rampa y 2 parabola
%% Inicialización o parametros fijos
k = 2652.28;
p = 64.986;
n_figura = 1;
%% Representación controlador P
num_control_p = [k*k_p];
den_control_p = [1 p k*k_p]
H_p = tf(num_control_p,den_control_p);% construir una funcción de transferencia
%impulse(H_p);
if (entrada == 0)
figure(n_figura); hold on
step(H_p);
title("Controlador P")
n_figura = n_figura +1;
xlabel('Tiempo') 
ylabel('Amplitud') 
end

% %% Representación controlador P-D
% num_control_p_d = [k*k_p];
% den_control_p_d = [1 (p+ k_p * k * tau_D1) k*k_p];
% H_p_d = tf(num_control_p_d,den_control_p_d);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura); hold on
% step(H_p_d);
% title("Controlador P-D")
% n_figura = n_figura +1;
% end
% %% Representación controlador PD
% num_control_pd = [(k*k_p* tau_D1) (k*k_p)];
% den_control_pd = [1 (p+ k_p * k * tau_D1) k*k_p];
% H_pd = tf(num_control_pd,den_control_pd);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura); hold on
% step(H_pd);
% title("Controlador PD")
% n_figura = n_figura +1;
% end
%% Representación controlador PI
num_control_pi = [(k*k_p) (k*k_p*1/tau_I)];
den_control_pi = [1 p (k_p * k) (k*k_p*1/tau_I)];
H_pi = tf(num_control_pi,den_control_pi);% construir una funcción de transferencia
%impulse(H_p);
if (entrada == 0)
figure(n_figura); hold on
step(H_pi);
title("Controlador PI")
n_figura = n_figura +1;
end
% %% Representación controlador PID
% num_control_pid = [(k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
% den_control_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
% H_pid = tf(num_control_pid,den_control_pid);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura)
% step(H_pid);
% title("Controlador PID")
% n_figura = n_figura +1;
% end
% %% Representación controlador PI-D
% num_control_pi_d = [(k*k_p) (k*k_p*1/tau_I)];
% den_control_pi_d = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
% H_pi_d = tf(num_control_pi_d,den_control_pi_d);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura)
% step(H_pi_d);
% title("Controlador PI-D")
% n_figura = n_figura +1;
% end
% %% Representación controlador PID-D
% num_control_pid_d = [(k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
% den_control_pid_d = [1 (p +k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
% H_pid_d = tf(num_control_pid_d,den_control_pid_d);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura)
% step(H_pid_d);
% title("Controlador PID-D") 
% n_figura = n_figura +1;
% end
% %% Representación controlador D|PID
% num_control_d_pid = [(k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
% den_control_d_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
% H_d_pid = tf(num_control_d_pid,den_control_d_pid);% construir una funcción de transferencia
% %impulse(H_p);
% if (entrada == 0)
% figure(n_figura)
% step(H_d_pid);
% title("Controlador D|PID") 
% n_figura = n_figura +1;
% end