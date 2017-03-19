clear all; close all; clc
% Lectura de DATA
load datosm3.lvm % cargar data
t=datosm3(:,1); % tiempo
y=datosm3(:,3); % salida
u=datosm3(:,2); % entrada
figure
subplot(211), plot(y,'r'), title('Salida')
subplot(212), plot(u,'r'), title('Entrada')
ident
%% Fcn Ident MATLAB
T=0.001;
dato=iddata(y,u,T); % data objeto
%% ARX
% hold on
% % figure
% plot(dato)
% na=1; nb=2; nk=1;
% th=arx(dato,[na,nb,nk]); % estructura ARX
% Gd=tf(th.b,th.a,T); % modelo identificado discreto
% Pc2=d2c(Gd) % convertir model discreto a continuo
% step(Pc2)

%% Diseño de PID
% Parametros de la planta no controlada-Según Process Model
%ident
T1=0.28667;
T2=0.28709;
Kp=1.0014;
Gp=tf(Kp,[T1*T2 T1+T2 1]);
step(Gp)
%% Calculo de tiempo de establecimiento
% P=tf(1,1);
% S=series(Gp,P);
% F=feedback(S,1);
% ltiview('step',F); %para calcular td
%% Parametros de diseño
Mp=0.10;
alfa=0.8;
% tr=0.00412;
td=1;
%% Calculo de ecuacion característica
z=sqrt(log(Mp)^2/(pi^2+log(Mp)^2));
wn=4/(td*z); %Criterio al 2%
z1=sqrt(1-z^2);
% wn=(pi-atan(z1/z))/(tr*z1);
%% Calculo de Parametros de control
%Método de Ganancia de polos I
K=(T1*T2*wn^2*(1+2*alfa*z)-1)/Kp;
Ti=(T1*T2*wn^2*(1+2*alfa*z)-1)/(T1*T2*alfa*wn^3);
Td=(T1*T2*wn*(alfa+2*z)-T1-T2)/(T1*T2*wn^2*(1+2*alfa*z)-1);
%%Ganancias de PID
Kp=K;
Kd=K*Td;
Ki=K/Ti;
Gpid=tf([K*Ti*Td K*Ti K],[Ti 0]); %Ec. Carac. de control PID
%% Retroalimentando el sistema
S=series(Gp,Gpid);
Gpc=feedback(S,1);
%% Obteniendo Gráficas
figure
step(Gp);
hold on
step(Gpc); %respuesta controlada en tiempo continuo
% legend('G_{sin controlar}','G_{controlado continuo}')
% ltiview('step',Gp,Gpc)