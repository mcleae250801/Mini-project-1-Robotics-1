
%Mini Project 2
clear all;close all;
import Sls.*;
%load S_letter_path.m

Sls = [1.0000    1.0621    1.1242    1.1863    1.2483    1.3103    1.3723    1.4343    1.4962    1.5581    1.6200    1.6818    1.7436    1.8054    1.8671    1.9287    1.9903    2.0519    2.1134    2.1748    2.2362    2.2975    2.3584    2.4187    2.4782    2.5375    2.5978    2.6580    2.7130    2.7589    2.7950    2.8211    2.8368    2.8418    2.8359    2.8194    2.7935    2.7594    2.7183    2.6714    2.6198    2.5647    2.5074    2.4490    2.3901    2.3307    2.2708    2.2105    2.1497    2.0886    2.0272    1.9657    1.9040    1.8423    1.7806    1.7192    1.6581    1.5974    1.5373    1.4777    1.4189    1.3606    1.3028    1.2451    1.1881    1.1332    1.0819    1.0358    0.9964    0.9654    0.9443    0.9347    0.9381    0.9561    0.9897    1.0362    1.0915    1.1514    1.2119    1.2707    1.3292    1.3891    1.4498    1.5110    1.5721    1.6332    1.6942    1.7553    1.8164    1.8775    1.9387    2.0000    2.0614    2.1230    2.1847    2.2467    2.3088    2.3712    2.4338    2.4968    2.5600;
      -1.0000   -1.0056   -1.0106   -1.0148   -1.0183   -1.0211    -1.0232   -1.0246   -1.0252   -1.0250   -1.0241   -1.0224   -1.0199    -1.0166   -1.0125   -1.0076   -1.0019   -0.9953   -0.9879   -0.9797    -0.9706   -0.9605   -0.9490   -0.9354   -0.9193   -0.9019   -0.8854    -0.8683  -0.8421   -0.8016   -0.7493   -0.6886   -0.6229   -0.5557    -0.4901   -0.4282   -0.3702   -0.3162   -0.2664   -0.2210   -0.1802    -0.144 -0.1129   -0.0868   -0.0654   -0.0479   -0.0337   -0.0219    -0.0117   -0.0027    0.0057    0.0138    0.0222    0.0312    0.0409     0.0512    0.0623    0.0742    0.0869    0.1005    0.1155    0.1337     0.1575    0.1890    0.2293    0.2772    0.3310    0.3892    0.4502     0.5125   0.5744    0.6346    0.6913    0.7431    0.7886    0.8278     0.8615    0.8902    0.9145    0.9351    0.9525    0.9674    0.9803     0.9920 1.0031    1.0135    1.0232    1.0322    1.0403    1.0476     1.0541    1.0596    1.0641    1.0676    1.0701    1.0714    1.0716     1.0706    1.0684    1.0649    1.0600];

%% PART 1
%{
----------------Part 1--------------------
Robot is 3-link planar arm
    L1 = 1.5 m
    L2 = 1.5 m 
    L3 = 0.5 m
Goal: trace the motion of an S path (Sls) [2x101] matrix 
%}

%plots the s curve

figure(1)
plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2)

%xlabel(x);ylabel(y);
axis([0 3 -1.5 2]);
axis('square');grid;
hold on


%path length array, λ, set up
%makes the number of instances in path length array
N = length(Sls)-1;
%makes an array for the path one minus the length
l = zeros(1,N);

%makes a loop to path the ditance between the two points
%uses equation λ=norm(p_i+1-p_i)

yT = [diff(Sls')'./vecnorm(diff(Sls')'); zeros(1,N)];
zcross = [0 -1 0;1 0 0;0 0 0];
xT = zcross*yT;


for i = 1:(length(Sls)-1)
    l(:,i) = norm(Sls(:,i+1)-Sls(:,i)); 
    quiver(Sls(1,i),Sls(2,i),xT(1,i),xT(2,i),"r")
end

hold off 

%[xT,yT]=setR0T(Sls)

%unit vectors defined
zz=zeros(3,0);
ex = [1;0;0];ey = [0;1;0];ez = [0;0;1];

% robot parameters
l1 = 1.5; l2 = 1.5; l3 = 0.5;
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];

%{
robot.q=[0;0;0];
% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);
figure(10);hold on;show(robot_rb,robot.q,'Collision','on');
view(0,90);
axis([0,4,-2,2]);axis('square');
xlabel('x-axis');ylabel('y-axis');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')
hold off
%}

%% PART 3
%FORWARD 
P01 = [0;0;0];
P12 = [l1;0;0];
P23 = [l2;0;0];
P3T = [l3;0;0];

q1= (-pi+(pi-(-pi))*rand()) 
q2= (-pi+(pi-(-pi))*rand()) 
q3= (-pi+(pi-(-pi))*rand())
q = [q1 q2 q3];
%rotation matrix for 3 links
R01 = rot(q1); %first angle rotation
R02 = rot(q1+q2); %first and second rotation
R03 = rot(q1+q2+q3); %first, second, and third
R0T = R03; %R0T = R01*R12*R23

P0T = P01+(R01*P12)+(R02*P23)+(R03*P3T);
T0T = [R0T P0T; 0 0 0 1]; 

p1 = P01;
p2 = p1+R01*P12;
p3 = p2+R02*P23;
pT = p3+R03*P3T;
P123T = [p1 p2 p3 pT];
%{
q11=(-pi+(pi-(-pi))*rand()) 
q22= (-pi+(pi-(-pi))*rand()) 
q33=(-pi+(pi-(-pi))*rand())
q = [q11 q22 q33];
%rotation matrix for 3 links
R_01 = rot(q11); %first angle rotation
R_02 = rot(q11+q22); %first and second rotation
R_03 = rot(q11+q22+q33); %first, second, and third
R_0T = R_03 %R0T = R01*R12*R23

p_1 = P01;
p_2 = p_1+R_01*P12;
p_3 = p_2+R_02*P23;
p_T = p_3+R_03*P3T;

P_123T = [p_1 p_2 p_3 p_T]
%}

figure(2)
plot(P123T(1,:),P123T(2,:))
hold on
%plot(P_123T(1,:),P_123T(2,:))
axis([-3,4,-3,4])
grid on
hold on


%INVERSE KINEMATICS FOR ONE ARM
qT = q1+q2+q3
p = P0T-R0T*P3T;
q2_1s = pi - acos(((norm(P23))^2+(norm(P12))^2-(norm(pB))^2)/(2*norm(P12)*norm(P23)));
q2_s = [q2_1s;-q2_1s]

p1_1 = [l1;0;0]+rot(q2_s(1))*[l2;0;0];
p1_2 = [l1;0;0]+rot(q2_s(2))*[l2;0;0];

sgn_1 = (cross([p1_1],[pB])'*ez)/abs((cross([p1_1],[pB])'*ez));
sgn_2 = (cross([p1_2],[pB])'*ez)/abs((cross([p1_2],[pB])'*ez));

q1_1 = sgn_1*2*atan(norm(p1_1-pB)/norm(p1_1+pB));
q1_2 = sgn_2*2*atan(norm(p1_2-pB)/norm(p1_2+pB));
q1_s = [q1_1;q1_2];

q3_s = qT - q1_s - q2_s

plot(p1_1(1,1),p1_1(2,1),'o')
plot(p1_2(1,1),p1_2(2,1),'o')

%ALGEBRAIC EQUATION FOR q1
%{
xT = norm(P0T(1,1))
qT_s = acos(R0T(1,1))
syms q1_s q3_s real
eq1 = xT == l1*cos(q1_s)+l2*cos(q1_s+q2_s)+l3*cos(q1_s+q2_s+q3_s);
eq2 = q3_s == qT_s - q1_s - q2_s;

sol = vpasolve([eq1,eq2],[q1_s,q3_s],[0,inf;0,inf]);
q1_s=double(sol.q1_s)
q3_s=double(sol.q3_s)
%}

%% PART 4

%xT is a 3x100 vector
%yT is a 3
%position start of the P0 is (0,0)



N = length(xT)
q_T = zeros(1,N);

q_1 = zeros(2,100);
q_2 = zeros(2,100);

%loop to find the q1, q2, and q3 based on the vector q_T
  
for i = 1:N
    q_T(:,i) = atan(xT(2,i)/xT(1,i));
    P_0T = [Sls(1,i);Sls(2,i);0];
    R_0T = rot(q_T(:,i));
    p_B = P_0T - R_0T*P3T;
    q_2_1s= pi - acos(((norm(P23))^2+(norm(P12))^2-(norm(p_B))^2)/(2*norm(P12)*norm(P23)));
    q_2_s = [q_2_1s;-q_2_1s];
    q_2(:,i) = q_2_s;
    
    p_A_1 = [l1;0;0]+rot(q_2_s(1))*[l2;0;0];
    p_A_2 = [l1;0;0]+rot(q_2_s(2))*[l2;0;0];
    
    sgn_A = (cross([p_A_1],[p_B])'*ez)/abs((cross([p_A_1],[p_B])'*ez));
    sgn_B = (cross([p_A_2],[p_B])'*ez)/abs((cross([p_A_2],[p_B])'*ez));
    
    q_1_1 = sgn_A*2*atan(norm(p_A_1-p_B)/norm(p_A_1+p_B));
    q_1_2 = sgn_B*2*atan(norm(p_A_2-p_B)/norm(p_A_2+p_B));
    q_1_s = [q_1_1;q_1_2];
    
    q_1(:,i) = q_1_s;
    
    q_3_s = q_T - q_1_s - q_2_s;
    q_3 = q_3_s;
    
end 

P01 = [0;0;0];
P12 = [l1;0;0];
P23 = [l2;0;0];
P3T = [l3;0;0];

figure(4)
plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2)
hold on
hold on
%plot(P_123T(1,:),P_123T(2,:))
axis()
grid on
hold on

for i = 1:(length(Sls)-1)
    l(:,i) = norm(Sls(:,i+1)-Sls(:,i)); 
    quiver(Sls(1,i),Sls(2,i),.25*xT(1,i),.25*xT(2,i),"r")
end
hold on

for i = 1:N
    %rotation matrix for 3 links
    R01_s = rot(q_1(1,i)); %first angle rotation
    R02_s = rot(q_1(1,i)+q_2(1,i)); %first and second rotation
    R03_s = rot(q_1(1,i)+q_2(1,i)+q_3(1,i)); %first, second, and third
    R0T_s = R03_s; %R0T = R01*R12*R23

    P0T_s = P01+(R01_s*P12)+(R02_s*P23)+(R03_s*P3T);
    T0T_s = [R0T_s P0T_s; 0 0 0 1]; 

    p_1 = P01;
    p_2 = p_1+R01_s*P12;
    p_3 = p_2+R02_s*P23;
    p_T = p_3+R03_s*P3T;
    P_123T = [p_1 p_2 p_3 p_T];
    plot(P_123T(1,:),P_123T(2,:))
end 
hold off 

figure(5)
plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2)
hold on
hold on
%plot(P_123T(1,:),P_123T(2,:))
axis()
grid on
hold on

for i = 1:(length(Sls)-1)
    l(:,i) = norm(Sls(:,i+1)-Sls(:,i)); 
    quiver(Sls(1,i),Sls(2,i),.25*xT(1,i),.25*xT(2,i),"r")
end
hold on

for i = 1:N
    %rotation matrix for 3 links
    R01_s = rot(q_1(2,i)); %first angle rotation
    R02_s = rot(q_1(2,i)+q_2(2,i)); %first and second rotation
    R03_s = rot(q_1(2,i)+q_2(2,i)+q_3(2,i)); %first, second, and third
    R0T_s = R03_s; %R0T = R01*R12*R23

    P0T_s = P01+(R01_s*P12)+(R02_s*P23)+(R03_s*P3T);
    T0T_s = [R0T_s P0T_s; 0 0 0 1]; 

    p_1 = P01;
    p_2 = p_1+R01_s*P12;
    p_3 = p_2+R02_s*P23;
    p_T = p_3+R03_s*P3T;
    P_123T = [p_1 p_2 p_3 p_T];
    plot(P_123T(1,:),P_123T(2,:))
end 

%}





%% Functions

%make R matrix into a function
function R=rot(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s 0;s c 0; 0, 0, 1];
end
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s ;s  c ;];
end




%}


% defineRobot.m
% Define a robot as a collision body rigid body tree given the
% Product of Expoential (POE) description of a robot arm
%
% input
%       robdef: robot definition structure with
%               robdef.H: motion axis, a 3xn matrix
%               robdef.P: link vector, a 3x(n+1) matrix
%               robdef.joint_type: 1xn vector, 0 for revolute 1 for prismatic
%       rad: radius of the link cylinder (each link assumed to be a
%       cylinder of equal radius).  This is easily changed in the
%       code to have different radii or link shape (e.g., box)
%
% output      
%       robot: MATLAB rigid body tree object
%       colLink: MATLAB link collision body description (in zero
%       configuration) 
%{
function [robot,colLink]=defineRobot(robdef,rad)

% product of exponential description
H=robdef.H;
P=robdef.P;
type=robdef.joint_type;
n=numel(type);

% homogeneous transforms T0i stored in T0i{k}, k=1,..,n+1
T=eye(4,4);
for i=1:n
    q(i)=0;
    if type(i)==0
        Ti{i}=[rot1(H(:,i),q(i)) P(:,i);[0 0 0 1]];
        T=T*Ti{i};
    else
        Ti{i}=[eye(3,3),P(:,i)+q(i)*H(:,i);[0 0 0 1]];
        T=T*Ti{i};
    end
    T0i{i}=T;
end
Ti{n+1}=[eye(3,3) P(:,n+1);[0 0 0 1]];
T0i{n+1}=T*Ti{n+1};

% define MATLAB rigidbody tree

robot=rigidBodyTree('DataFormat','column');

for i=1:n
    ii=num2str(i);
    eval(['body',ii,' = rigidBody(''','body',ii,''');']);
    if type(i)==0
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''revolute''',');']);
    else
        eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
            ii,'''',',''prismatic''',');']);
    end
    eval(['jnt',ii,'.JointAxis=H(:,',ii,');']);
end
ii=num2str(n+1);
eval(['body',ii,' = rigidBody(''','body',ii,''');']);
eval(['jnt',ii,' = rigidBodyJoint(''','jnt',...
    ii,'''',',''fixed''',');']);

for i=1:n+1
    ii=num2str(i);
    eval(['setFixedTransform(jnt',ii,',Ti{',ii,'});']);
    eval(['body',ii,'.Joint = jnt',ii,';']);
end

addBody(robot,body1,'base')
for i=2:n+1
    ii=num2str(i);
    iim1=num2str(i-1);
    eval(['addBody(robot,body',ii,',','''body',iim1,''');']);
end

colBodyRadius=rad;

for i=1:n+1
    if norm(P(:,i))>0
        if (i<n+1) && (type(i)>0)
            colLink{i} = collisionBox(colBodyRadius,colBodyRadius,norm(P(:,i)));
        else
            colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
        end
        kvec=cross([0;0;1],P(:,i));
        if norm(kvec)<sqrt(eps)
            colLink{i}.Pose=trvec2tform(P(:,i)'/2);
        else
            th=subprob0(kvec,[0;0;1],P(:,i));
            colLink{i}.Pose=[rot1(kvec,th) P(:,i)/2;[0 0 0 1]];
        end
    else
        colLink{i} = collisionCylinder(colBodyRadius,norm(P(:,i)));
    end
    if i==1
        addCollision(robot.Base,colLink{i});    
    else
        addCollision(robot.Bodies{i-1},colLink{i});    
    end    
end

end

%
% rot.m 
%
% rotation matrix about vector k over angle theta
% 
% R=rot(k,theta)
%

function R=rot1(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);
  
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

%
% q=subprob0(k,p1,p2)
%
% solve for q subtended between p1 and p2
%    k determines the sign of q
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=subprob0(k,p1,p2)

if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
  error('k must be perpendicular to p and q');
end

p1=p1/norm(p1);
p2=p2/norm(p2);

q=2*atan2(norm(p1-p2),norm(p1+p2));

if k'*(cross(p1,p2))<0
  q=-q;
end

end
%}