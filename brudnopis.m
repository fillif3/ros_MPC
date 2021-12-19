n=1500;
ts=5;
x=zeros(1,n);
y=zeros(1,n);

v=1;
w=-0.1;
state=[0,0,0];
for i=1:n
    state=wheeled_robot_kinematics_model([v,w],state,ts);
    x(i)=state(1);
    y(i)=state(2);
end
plot(x,y)