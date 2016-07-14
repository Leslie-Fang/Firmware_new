function [xa_apo,Pa_apo]= MarkerEKF(dt,z,q_a,q_v,q_x,r_a,r_x)
%���
% %xa_apo״̬�������Ź��ƣ�6ά���ֱ�Ϊ��������ϵ�µ�X����Y��ļ��ٶȣ��ٶ��Լ�λ��
% ax����������ϵ�µ�X����ٶ�
% ay����������ϵ�µ�Y����ٶ�
% vx����������ϵ�µ�X���ٶ�
% vy����������ϵ�µ�Y���ٶ�
% x����������ϵ�µ�X������
% y����������ϵ�µ�Y������
%%Pa_apoЭ����������Ź���

%����
%persitent x_apo����һʱ�̵�״̬�����Ź���
%6ά���ֱ�Ϊ��������ϵ�µ�X����Y��ļ��ٶȣ��ٶ��Լ�λ��

%persitent p_apo����һʱ�̵�Э��������Ź���

%Z������������4ά
% axz����������ϵ�µ�X����ٶȲ���ֵ
% ayz����������ϵ�µ�Y����ٶȲ���ֵ
% xz����������ϵ�µ�X��λ�ò���ֵ
% yz����������ϵ�µ�Y��λ�ò���ֵ
%dt: dt ���ֵĲ���

%% init
persistent x_apo
if(isempty(x_apo))
    x_apo=[0,0,0,0,10,10]; 
end

%������ʼЭ������󣬵������̻�����%
persistent P_apo
if(isempty(P_apo))
    P_apo = single(ones(6));
end

ax=x_apo(1);% ax����������ϵ�µ�X����ٶ�
ay=x_apo(2);% ay����������ϵ�µ�Y����ٶ�
vx=x_apo(3);% vx����������ϵ�µ�X���ٶ�
vy=x_apo(4);% vy����������ϵ�µ�Y���ٶ�
x=x_apo(5);% x����������ϵ�µ�X������
y=x_apo(6);% y����������ϵ�µ�Y������

%% prediction section
%��ʽ1
%X(k|k-1)=A X(k-1|k-1)+B U(k) ������.. (1)
%ϵͳ���Ʊ���������ΪU=0
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
x_apr=[ak;vk;rk];%x_apr��ǰʱ�̵�״̬����

%��ʽ2 ����Э��������Ԥ��
% P(k|k-1)=A P(k-1|k-1) A��+Q
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
%�ȹ�ʽ4���ٹ�ʽ3 
% Kg(k)= P(k|k-1) H�� [ (H P(k|k-1) H�� + R)]^-1 
%H Ϊ��������
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
%  ��ʽ3����������״̬����
% X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ������ (3)
y_k=z-H*x_apr;
x_apo=x_apr+K_k*y_k;
%  
%  ��ʽ5������Э����������
% P(k|k)=��I-Kg(k) H��P(k|k-1)
P_apo=(eye(6)-K_k*H)*P_apr;
xa_apo=x_apo;
Pa_apo=P_apo;
