clear;
clc;
%% Sec.1
%first of all, we should define statespace matrix and after that define system%
A=[0 1 0 0;0 -15.14 -3.04 0;0 0 0 1; 0 37.23 31.61 0];
B=[0;3.39;0;-8.33];
C=[1 0 0 0;0 0 1 0];
system=ss(A,B,C,0,0);
C0 = ctrb(A,B) ;
Rc = rank(C0) 
Rb = obsv(A,C) ;
ROb = rank(Rb) 
%% Sec.2
%controlabillity and observability are checked%
Q=[100 0 0 0;0 1 0 0;0 0 32.65 0;0 0 0 1];
R=1;
K = lqr(A,B,Q,R);
A2 = (A-B*K);
B2 = B;
C2 = C;

%% Sec.3
%Now we want to simulate close-loop system with LQR
system2 = ss(A2,B2,C2,0,0);
t1=0:0.01:10;
u=-2*ones(size(t1));
[y,t1,x]=lsim(system2,u,t1);

%% Sec.4
figure
plot(t1,y(:,1),'g',t1,y(:,2),'r');
xlim([0 5]);
xlabel('Time (s)' );
ylim([-.05 .3]);
grid on ;
legend('Robot position in meter','Pendulum angle in radians');
title('respons of the system using LQR ');

%% Sec.5
%now add a observer
A2 = (A-B*K);
P=eig(A2);
L = (place(A,C',P)).';
hA=[(A-B*K) (B*K); zeros(size(A)) (A-L*C)];
hB=[B;zeros(size(B))];
hC=[C zeros(size(C))];
system3=ss(hA,hB,hC,0);
u3=stepfun(t1,1);
[hY,t1,hX]=lsim(system3,u3,t1);

%% Sec.6
figure
plot(t1,hY(:,1),'g',t1,hY(:,2),'r')
xlim([0 10]);
xlabel('Time (s)' );
ylim([-0.15 0.04]);
grid on ;
legend('Robot','Pendulum');
title('Time respons of the system using LQR and observer');
grid on
