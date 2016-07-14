function [xa_apo,Pa_apo]= MarkerEKF(dt,z,q_a,q_v,q_x,r_a,r_x)
%输出
% %xa_apo状态变量最优估计，6维，分别为惯性坐标系下的X轴与Y轴的加速度，速度以及位置
% ax：惯性坐标系下的X轴加速度
% ay：惯性坐标系下的Y轴加速度
% vx：惯性坐标系下的X轴速度
% vy：惯性坐标系下的Y轴速度
% x：惯性坐标系下的X轴坐标
% y：惯性坐标系下的Y轴坐标
%%Pa_apo协方差矩阵最优估计

%输入
%persitent x_apo，上一时刻的状态的最优估计
%6维，分别为惯性坐标系下的X轴与Y轴的加速度，速度以及位置

%persitent p_apo，上一时刻的协方差的最优估计

%Z，测量变量，4维
% axz：惯性坐标系下的X轴加速度测量值
% ayz：惯性坐标系下的Y轴加速度测量值
% xz：惯性坐标系下的X轴位置测量值
% yz：惯性坐标系下的Y轴位置测量值
%dt: dt 积分的步长

%% init
persistent x_apo
if(isempty(x_apo))
    x_apo=[0,0,0,0,10,10]; 
end

%给定初始协方差矩阵，迭代过程会收敛%
persistent P_apo
if(isempty(P_apo))
    P_apo = single(ones(6));
end

ax=x_apo(1);% ax：惯性坐标系下的X轴加速度
ay=x_apo(2);% ay：惯性坐标系下的Y轴加速度
vx=x_apo(3);% vx：惯性坐标系下的X轴速度
vy=x_apo(4);% vy：惯性坐标系下的Y轴速度
x=x_apo(5);% x：惯性坐标系下的X轴坐标
y=x_apo(6);% y：惯性坐标系下的Y轴坐标

%% prediction section
%公式1
%X(k|k-1)=A X(k-1|k-1)+B U(k) ……….. (1)
%系统控制变量的输入为U=0
% ax(k|k-1)=ax(k-1|k-1)
% ay(k|k-1)=ay(k-1|k-1)
% vx(k|k-1)=vx(k-1|k-1)+ax(k-1|k-1)*dt;
% vy(k|k-1)=vy(k-1|k-1)+ay(k-1|k-1)*dt;
% x(k|k-1)=x(k-1|k-1)+vx(k-1|k-1)*dt+1/2*ax(k-1|k-1)*dt^2;
% y(k|k-1)=y(k-1|k-1)+vy(k-1|k-1)*dt+1/2*ay(k-1|k-1)*dt^2;
%A=[1 0 0 0 0 0;
% 0 1 0 0 0 0;
% dt 0 1 0 0 0;
% 0 dt 0 1 0 0;
% 0 0 dt 0 1 0;
% 0 0 0 dt 0 1;
% ]
ak =[ax;ay];
vk =[vx;vy] + dt*ak;
rk=[x;y]+[vx;vy]*dt+1/2*ak*dt^2;
% rk=[x;y]+dt*vk;
x_apr=[ak;vk;rk];%x_apr当前时刻的状态估计

%公式2 更新协方差矩阵的预测
% P(k|k-1)=A P(k-1|k-1) A’+Q
% A=[1 0 0 0 0 0;
%    0 1 0 0 0 0;
%    dt 0 1 0 0 0;
%    0 dt 0 1 0 0;
%    0 0 dt 0 1 0;
%    0 0 0 dt 0 1;];
A=[1 0 0 0 0 0;
   0 1 0 0 0 0;
   dt 0 1 0 0 0;
   0 dt 0 1 0 0;
   1/2*dt^2 0 dt 0 1 0;
   0 1/2*dt^2 0 dt 0 1;];

persistent Q
if (isempty(Q))
    Q=diag([ q_a,q_a,q_v,q_v,q_x,q_x]);
end

P_apr=A*P_apo*A'+Q;

%% update
%先公式4，再公式3 
% Kg(k)= P(k|k-1) H’ [ (H P(k|k-1) H’ + R)]^-1 
%H 为测量矩阵
H=[1,0,0,0,0,0;
    0,1,0,0,0,0;
    0,0,0,0,1,0;
    0,0,0,0,0,1];

persistent R
if (isempty(R))
    R=diag([ r_a,r_a,r_x,r_x]);
end
 S_k=H*P_apr*H';
 S_k = S_k+ R;
 K_k=(P_apr*H'/(S_k));
%  
%  公式3，更新最优状态估计
% X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ……… (3)
y_k=z-H*x_apr;
x_apo=x_apr+K_k*y_k;
%  
%  公式5，更新协方差矩阵估计
% P(k|k)=（I-Kg(k) H）P(k|k-1)
P_apo=(eye(6)-K_k*H)*P_apr;
xa_apo=x_apo;
Pa_apo=P_apo;
