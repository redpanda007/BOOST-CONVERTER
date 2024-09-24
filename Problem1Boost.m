clear all
% Plant Parameters
Vg = 12;
Vc = 30;
Il = 1.5;
R = 50;
L = 120e-6;
C = 50e-6;
D = 0.6;

% PLANT
a= Vg/((1-D)^2);
b= L/(R*((1-D)^2));
c= (L*C)/((1-D)^2);
Np = [0 a*(-b) a ];
Dp = [c b 1];

n = Il/c;
m =Vc*(1-D)/L*C;
o = ((1-D)^2)/L*C;
z= [-n m];
y= [1 1/R*C o];

Gp = tf(Np,Dp);
%Gp = tf(z,y);
subplot(3,1,1),rlocus(Gp);

%PI CONTROLLER
Kp = 0.01036;
Ki = 14;
Kd = 1.84e-6;
Nc = [Kp Ki];
Dc = [1 0];

Gc = tf(Nc,Dc);

% Closed loop TF

nc = [-a*b*Kd (a*Kd)-(a*b*Kp) ((a*Kp)-(a*b*Ki)) Ki*a];
dc = [c-(Kd*a*b) (b-(a*b*Kp)+(a*Kd)) (1+(a*Kp)-(a*b*Ki)) Ki*a];

%nc = [-n*Kd Kd*m-n*Kp m*Kp-n*Ki m*Ki];
%dc = [1-n*Kd Kd*m-n*Kp+(1/R*C) o+m*Kp-n*Ki m*Ki] 
Gscl = tf(nc,dc);
subplot(3,1,2),rlocus(Gscl);
subplot(3,1,3),step(Gscl);
