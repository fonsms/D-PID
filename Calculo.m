% Calculo.m:
%
% Script que calcula los parametros kp,ki y kd para un controlador D|PID
% con ciertas restricciones
%
% Fecha: 01/06/20     Version: 1.0
% Copyright: Alfonso Moreno Sanz

clear all;
close all;
n_figura = 1;
%% Fijando B_2 calculamos los posibles valores de C(coeficiente de amortiguamiento) y B

% Parametros configurables
K = 2652.28;
r = 23;
k = K/r;
p = 64.986;
B_incremento= 0:0.1:70;
B_2 = 0.7;
C_incremento = [0.5 0.6 0.7 0.8];



%Se hacen dos for, el primero para cada C y el segundo para cada B
for i = 1:1:length(C_incremento)
    C = C_incremento(i);
    for  j = 1:1:length(B_incremento)
        B = B_incremento(j);
        %Transformamos con los parametros B,B_2 y C a  k_p tau_D1 tau_D2 tau_I
        k_p =((p.^2)*(2*B+1/(C.^2))) / (((B_2.^2)*k));
        tau_D1 =(B_2*(B-B_2+2)) / (p*(2*B+(1/(C.^2))));
        tau_D2 =p/(k*k_p);
        tau_I = (B_2*(C.^2)*(2*B+1/(C.^2)))/(B*p);
        
        %Generamos la funcion de transferencia de un controlador D|PID
        num_control_d_pid = [(k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
        den_control_d_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
        H_d_pid = tf(num_control_d_pid,den_control_d_pid);
        
        % obtenemos los valores la función ante la respuesta al impulso
        [y,t]=step(H_d_pid);
        
        %Almacenamos el valor M_p para cada interación
        M_p(i,j) = max(y);
    end
    %Representamos
    figure(n_figura); hold on
    plot(B_incremento,M_p(i,:))
end
ref_baja =ones(1,length(B_incremento))*1.06;
ref_alta =ones(1,length(B_incremento))*1.13;
plot (B_incremento,ref_baja,'--');
plot (B_incremento,ref_alta,'--');
legend("\zeta = 0.5","\zeta = 0.6","\zeta = 0.7","\zeta = 0.8","valor max M_P","valor min M_P");
title("Relación \beta y M_P")
xlabel('\beta')
ylabel('M_P')
n_figura = n_figura +1;
%Almacenamos los intervalos que cumplen lo deseado
[row,col] = find (M_p > 1.06 & M_p < 1.13);
C_05_i = find(row == 1);
C_05 = B_incremento(col(C_05_i));
C_06_i = find(row == 2);
C_06 = B_incremento( col(C_06_i));
C_07_i = find(row == 3);
C_07 = B_incremento( col(C_07_i));
C_08_i = find(row == 4);
C_08 = B_incremento( col(C_08_i));

%Metemos en un array el maximo y el minimo de cada intervalo (Esto quizas se tenga que tocar si la señal hace un rizado)
C_array = [C_05(1) C_05(end) C_06(1) C_06(end) C_07(1) C_07(end) C_08(1) C_08(end)];

%% Calculamos B_2 para el B minimo y B maximo de cada C que cumple el M_p 

C = 0.4;
%Se hacen dos for, el primero para cada C y el segundo para cada B
for i = 1:1:length(C_array)
    salvar = 1;
    salvar2 = 1;
    B = C_array(i);
    B_2_incremento = 0.1:0.1:100;
    if ~rem(i,2)==0
        C = C+ 0.1;
    end
    
    for  j = 1:1:length(B_2_incremento)
        B_2 = B_2_incremento(j);
       
        %Generamos la funcion de transferencia de un controlador D|PID
        k_p =((p.^2)*(2*B+1/(C.^2))) / (((B_2.^2)*k));
        tau_D1 =(B_2*(B-B_2+2)) / (p*(2*B+(1/(C.^2))));
        tau_D2 =p/(k*k_p);
        tau_I = (B_2*(C.^2)*(2*B+1/(C.^2)))/(B*p);
        num_control_d_pid = [(k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
        den_control_d_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
        H_d_pid = tf(num_control_d_pid,den_control_d_pid);
        [y_b2,t_b2]=step(H_d_pid,3);
        %Calculamos el máximo valor de B_2 que cumple la especificación de
        %ts
        l=length(t_b2);
        while (y_b2(l)>0.98)&(y_b2(l) < 1.02)
        l=l-1; 
        end
        ts(j)=t_b2(l); 
        %salvamos ese valor de B_2
         if( ts(j)>=0.4 )
         if(salvar== 1)
         B_2_max(i) = B_2;
         salvar = 0;
         end
         end
         
         %calculamos la limitación de tr
         indices_tr  =   find (y_b2> 1 );    
         tr(j) = t_b2(indices_tr(1));
         if( tr(j)>=0.25 | j == length(B_2_incremento))
         if(salvar2== 1)
         B_2_max2(i) = B_2;
         salvar2 = 0;
         end
         end
    end
        
    %Almaceno el último B_2 que culpe las restricciones.
    
    %Representamos
     figure(n_figura); hold on
     plot(B_2_incremento,ts)   
end
ref_ts =ones(1,length(B_2_incremento))*0.4;
plot (B_2_incremento,ref_ts,'--');
legend(sprintf('\\zeta = 0.5,\\beta =%.3f',C_array(1)),sprintf('\\zeta = 0.5,\\beta =%.3f',C_array(2)),sprintf('\\zeta = 0.6,\\beta =%.3f',C_array(3)),sprintf('\\zeta = 0.6,\\beta =%.3f',C_array(4)),sprintf('\\zeta = 0.7,\\beta =%.3f',C_array(5)),sprintf('\\zeta = 0.7,\\beta =%.3f',C_array(6)),sprintf('\\zeta = 0.8,\\beta =%.3f',C_array(7)),sprintf('\\zeta = 0.8,\\beta =%.3f',C_array(8)));
title("Especificación de ts")
xlabel('B_2')
ylabel('ts')
n_figura = n_figura +1;

%% Codigo para visualizar la simulación exacta de lo que va a introducir al motor real
%y calculo de los parametros que necesita el programa
calcular_prestaciones = 1;
T = 5*1e-3;
K = 2652.28;
r = 23;
k = K/r;
p = 64.986;
B= 30.5;
B_2 =15;
C = 0.5;
k_p =((p.^2)*(2*B+1/(C.^2))) / (((B_2.^2)*k))
tau_D1 =(B_2*(B-B_2+2)) / (p*(2*B+(1/(C.^2))));
K_d1 = k_p *  tau_D1/ T
tau_D2 =p/(k*k_p);
K_d2 = k_p *  tau_D2/ T
tau_I = (B_2*(C.^2)*(2*B+1/(C.^2)))/(B*p);
K_I = k_p* T/tau_I
num_control_d_pid = [(k*k_p* (tau_D1+tau_D2)) (k*k_p) (k*k_p*1/tau_I)];
den_control_d_pid = [1 (p +k*k_p* tau_D1) (k*k_p) (k*k_p*1/tau_I)];
H_d_pid = tf(num_control_d_pid,den_control_d_pid);
figure(n_figura); hold on
step(H_d_pid,1);
[y_prestacion,t_prestacion]=step(H_d_pid,3);
legend(sprintf('\\zeta = %.2f,\\beta =%.3f \\beta =%.3f',C,B,B_2));
n_figura = n_figura +1;
if calcular_prestaciones == 1
    %calculo de ts
    l_prestacion=length(t_prestacion);
    while (y_prestacion(l_prestacion)> 0.98) & (y_prestacion(l_prestacion) < 1.02)
        l_prestacion=l_prestacion-1;
    end
    
    ts_prestacion=t_prestacion(l_prestacion)
    %calculamos la limitación de tr
    indices_tr_prestacion  =   find (y_prestacion> 1 );
    tr_pres = t_prestacion(indices_tr_prestacion(1))
    % Caculamos el maximo
   Mp_prestacion = max(y_prestacion)
end

%figure (n_figura)
%nyquist(H_d_pid);