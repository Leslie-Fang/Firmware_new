clear all
clc
% %�������ݵ�Ŀ¼
% CSV%
% data = csvread('CSV\data2.csv',1,0);
data = csvread('CSV\data3.csv',1,0);
% data = csvread('CSV\data4.csv',1,0);
% data = csvread('CSV\data5.csv',1,0);
% data = csvread('CSV\data6.csv',1,0);

t=double((data(:,1)-data(1,1)))/1000000;%ÿһ�����ݵ�logʱ��%
roll=data(:,14); % phi%
pitch=data(:,15); % seta%
yaw=data(:,16);  % psi%

localsense_x=data(:,18);%N��ԭʼ��������%
localsense_y=data(:,19);%E��ԭʼ��������%
baro_z=data(:,17)-100;

x_est=data(:,20);%log�Ļ����˲�X�����λ�ù���%
y_est=data(:,21);%log�Ļ����˲�Y�����λ�ù���%
z_est=data(:,22);%log�Ļ����˲�Z�����λ�ù���%

xv_est=data(:,23);%log�Ļ����˲�X������ٶȹ���%
yv_est=data(:,24);%log�Ļ����˲�Y������ٶȹ���%
zv_est=data(:,25);%log�Ļ����˲�Z������ٶȹ���%

state=data(:,26);%��¼��ǰ�Ŀ���ģʽ%

b_ax=double(data(:,2));%log��������ϵX����ļ��ٶ�%
b_ay=double(data(:,3));%log��������ϵY����ļ��ٶ�%
b_az=double(data(:,4));%log��������ϵZ����ļ��ٶ�%

q_a=0.05;%���ٶȵĹ�������%
q_v=0.05;%�ٶȵĹ�������%
q_x=0.05;%λ�õĹ�������%
r_a=0.05;%���ٶȵĲ�������%
% r_x=0.05;%λ�õĲ�������%
r_x=0.5;

l=length(localsense_x);

xv_kalman=[];
yv_kalman=[];
x_kalman=[];
y_kalman=[];


for i=2:l
dt=t(i)-t(i-1);
%������ת����%
phi=roll(i);
theta=pitch(i);
psi=yaw(i);
R=[cos(psi)*cos(theta),cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
    sin(psi)*cos(theta),sin(psi)*sin(theta)*sin(phi)+cos(phi)*cos(psi),sin(psi)*sin(theta)*cos(phi)-sin(phi)*cos(psi);
    -sin(theta),cos(theta)*sin(phi),cos(phi)*cos(theta);
    ];
%����ٶ�ab�仯����������ϵ�¼��ٶ�ag%
%���ٶ���Ҫ�����ת���󣬱任����������ϵ��%
ab=[b_ax(i),b_ay(i),b_az(i)]';
ag=R*ab;
g_ax=ag(1);
g_ay=ag(2);
%zΪ�������Ĳ�������%
z=[g_ax,g_ay,localsense_x(i),localsense_y(i)]';

[xa_apo,Pa_apo]=MarkerEKF(dt,z,q_a,q_v,q_x,r_a,r_x);
%xa_apo���µ�״̬����%
% Pa_apo���µ�Э�������
xv_kalman(i)=xa_apo(3);
yv_kalman(i)=xa_apo(4);
x_kalman(i)=xa_apo(5);
y_kalman(i)=xa_apo(6);
end

xv_kalman(1)=xv_kalman(2);
yv_kalman(1)=yv_kalman(2);
x_kalman(1)=x_kalman(2);
y_kalman(1)=y_kalman(2);

figure(1)
plot(localsense_x)
hold on
plot(x_est,'g')
hold on
plot(x_kalman,'r')
xlabel('ʱ��(s)')
ylabel('Xλ��(m)')
legend('ԭʼ����','�˲�����','��������')

figure(2)
plot(localsense_y)
hold on
plot(y_est,'g')
hold on
plot(y_kalman,'r')
xlabel('ʱ��(s)')
ylabel('Yλ��(m)')
legend('ԭʼ����','�˲�����','��������')

figure(3)
plot(xv_est)
hold on
plot(xv_kalman,'g')
xlabel('ʱ��(s)')
ylabel('X�ٶ�(m/s)')
legend('�˲�����','��������')

figure(4)
plot(yv_est)
hold on
plot(yv_kalman,'g')
xlabel('ʱ��(s)')
ylabel('Y�ٶ�(m/s)')
legend('�˲�����','��������')


