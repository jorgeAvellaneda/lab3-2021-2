% trayectora para un robot de 6 grados de libertad con 6 grados de libertad
% 19/1/2022
%% CREACION DEL ROBOT
close all
clear;clc;
L1=380;
L2=820;
L3=880;
L(1) = Link('revolute','alpha',0,'a',0,'d',L1,'offset',0, 'qlim',[-pi pi],'modified');
L(2) = Link('revolute','alpha',  -pi/2,  'a', 0,'d', 0,'offset',-pi/2, 'qlim',[-pi pi],'modified');
L(3) = Link('revolute','alpha',      0,  'a',L2,'d', 0,'offset',    0, 'qlim',[-67/180*pi 247*pi/180],'modified');
L(4) = Link('revolute','alpha',  -pi/2,  'a', 0,'d',L3,'offset',    0, 'qlim',[-210*pi/180 210*pi/180],'modified');
L(5) = Link('revolute','alpha',   pi/2,  'a', 0,'d', 0,'offset',    0, 'qlim',[-pi pi],'modified');
L(6) = Link('revolute','alpha',  -pi/2,  'a', 0,'d', 0,'offset',    0, 'qlim',[-210*pi/180 210*pi/180],'modified');
bot1=SerialLink(L,'name','Yaskawa HC20XP');

mth_tool = rt2tr(eye(3),[0 0 0])
bot1.tool = mth_tool
%% Creacion del tiempo
t1 = linspace(0,1,20);
t2 = linspace(1,1.5,20);
t3 = linspace(1.5,2.5,20);
t4 = linspace(2.5,4,41);
t=[t1 t2 t3 t4];
%% definicion del area de trabajo: plano 
Rb_S=rotz(-90,'deg')*roty(45,'deg');
Psorg= [0 -400 1100]';
S=rt2tr(Rb_S,Psorg);
%% definicion del path
%relaciones basicas
Length=680;
a=0.5;% factor de escalabilidad no mayor a uno ni menor de 0
c=Length*(1-a/2);
b=a*Length;
R=a*Length/2;

%Cordenadas iniciales y finales rectangulo en coordenadas de la base en
%matrices homogeneas
p1=rt2tr(eye(3),[-Length/2 b/2 0 ]');
p2=rt2tr(eye(3),[-Length/2 -b/2 0]');
p3=rt2tr(eye(3),[Length/2-R -b/2 0]');
p4=rt2tr(eye(3),[Length/2-R 0 0]');%centro del circulo
p5=rt2tr(eye(3),[Length/2-R b/2 0]');
%Traslado los puntos  a S y luego los llevo a la orientacion que debe tener
%el efector final respecto a S
T_p1=S*p1;
T_p2=S*p2;
T_p3=S*p3;
T_p4=S*p4;
T_p5=S*p5;
%% Interpolacion del circulo ENTRAN EN B y salen en S
theta = -pi/2:pi/40:pi/2;
[x,y] = pol2cart(theta,R*ones(size(theta)));

for i=1:length(x)
   Pcirculo(:,:,i) =S* rt2tr(eye(3),[x(i)+ 170 y(i) 0]');
end
%% SEMICIRCULO P3-P4-P5

%% SEGMENTO P5-P1
%cinematica inversa
T_traj = ctraj(T_p5,T_p1,20);
tra = cat(3,Pcirculo,T_traj);
%% SEGMENTO P1-P2
%cinematica inversa
T_traj = ctraj(T_p1,T_p2,20);
tra = cat(3,tra,T_traj);
%% SEGMENTO P2-P3
%cinematica inversa
T_traj = ctraj(T_p2,T_p3,20);
tra = cat(3,tra,T_traj);

q=bot1.ikine(tra(:,:,:));


%% UNION DEL MOVIMIENTO
qsize=size(q);
for i=1:qsize(1)
    if q(i,4)>pi
        q(i,4)=q(i,4)-2*pi;
    end
end

figure(1)
xyz = squeeze(tra(1:3,4,:));
plot3(xyz(1,:),xyz(2,:),xyz(3,:))
axis([-2000 2000 -2000 2000 0 4000])
hold on
bot1.plot(q,'noa','workspace',[-2000 2000 -2000 2000 0 4000]);
figure(2)
plot(t,rad2deg(q(:,:)));







