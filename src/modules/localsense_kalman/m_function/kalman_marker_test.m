clear all
clc
% %所有数据的目录
% CSV%
% data = csvread('CSV\data2.csv',1,0);
data = csvread('CSV\data3.csv',1,0);
% data = csvread('CSV\data4.csv',1,0);
% data = csvread('CSV\data5.csv',1,0);
% data = csvread('CSV\data6.csv',1,0);

t=double((data(:,1)-data(1,1)))/1000000;%每一组数据的log时间%
roll=data(:,14); % phi%
pitch=data(:,15); % seta%
yaw=data(:,16);  % psi%

localsense_x=data(:,18);%N，原始测量数据%
localsense_y=data(:,19);%E，原始测量数据%
baro_z=data(:,17)-100;

x_est=data(:,20);%log的互补滤波X方向的位置估计%
y_est=data(:,21);%log的互补滤波Y方向的位置估计%
z_est=data(:,22);%log的互补滤波Z方向的位置估计%

xv_est=data(:,23);%log的互补滤波X方向的速度估计%
yv_est=data(:,24);%log的互补滤波Y方向的速度估计%
zv_est=data(:,25);%log的互补滤波Z方向的速度估计%

state=data(:,26);%记录当前的控制模式%

b_ax=double(data(:,2));%log的体坐标系X方向的加速度%
b_ay=double(data(:,3));%log的体坐标系Y方向的加速度%
b_az=double(data(:,4));%log的体坐标系Z方向的加速度%

q_a=0.05;%加速度的过程噪声%
q_v=0.05;%速度的过程噪声%
q_x=0.05;%位置的过程噪声%
r_a=0.05;%加速度的测量噪声%
% r_x=0.05;%位置的测量噪声%
r_x=0.5;

l=length(localsense_x);

xv_kalman=[];
yv_kalman=[];
x_kalman=[];
y_kalman=[];


for i=2:l
dt=t(i)-t(i-1);
%更新旋转矩阵%
phi=roll(i);
theta=pitch(i);
psi=yaw(i);
R=[cos(psi)*cos(theta),cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
    sin(psi)*cos(theta),sin(psi)*sin(theta)*sin(phi)+cos(phi)*cos(psi),sin(psi)*sin(theta)*cos(phi)-sin(phi)*cos(psi);
    -sin(theta),cos(theta)*sin(phi),cos(phi)*cos(theta);
    ];
%体加速度ab变化到惯性坐标系下加速度ag%
%加速度需要添加旋转矩阵，变换到惯性坐标系下%
ab=[b_ax(i),b_ay(i),b_az(i)]';
ag=R*ab;
g_ax=ag(1);
g_ay=ag(2);
%z为卡尔曼的测量向量%
z=[g_ax,g_ay,localsense_x(i),localsense_y(i)]';

[xa_apo,Pa_apo]=MarkerEKF(dt,z,q_a,q_v,q_x,r_a,r_x);
%xa_apo更新的状态估计%
% Pa_apo更新的协方差矩阵
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
xlabel('时间(s)')
ylabel('X位置(m)')
legend('原始数据','滤波数据','仿真数据')

figure(2)
plot(localsense_y)
hold on
plot(y_est,'g')
hold on
plot(y_kalman,'r')
xlabel('时间(s)')
ylabel('Y位置(m)')
legend('原始数据','滤波数据','仿真数据')

figure(3)
plot(xv_est)
hold on
plot(xv_kalman,'g')
xlabel('时间(s)')
ylabel('X速度(m/s)')
legend('滤波数据','仿真数据')

figure(4)
plot(yv_est)
hold on
plot(yv_kalman,'g')
xlabel('时间(s)')
ylabel('Y速度(m/s)')
legend('滤波数据','仿真数据')


