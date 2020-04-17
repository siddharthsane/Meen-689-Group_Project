function [dx,dy] = Gps_to_bodyframe(Position)
x=Position.latitude*(pi/180);
y=Position.longitude*(pi/180);
%%
X=[x(1) x(20)];
Y=[y(1) y(20)];
%projection on path
for i=2:length(x)
ref=[X(2)-X(1) Y(2)-Y(1)];
vec(i,:)=([x(i)-X(1) y(i)-Y(1)]*ref')*ref;
norm_2=norm(ref)^2;
V(:,i)=(vec(i,:))'/norm_2;
res_vec(:,i)=(vec(i,:))'/norm_2+[X(1) Y(1)]';
end

Rg = 6371000;

x=V(1,:);
y=V(2,:);

for i = 1:length(x)-1
    a=(sin((x(i+1)-x(i))/2))^2+cos(x(i))*cos(x(i+1))*(sin((y(i+1)-y(i))/2))^2;
    c=2*atan2(sqrt(a),sqrt(1-a));
    dx(i)=Rg*c;
    dy(i) = sum(dx);
end
dy(i);
dy(i+1) = dy(i);% GPS position in y-axis
%%
x=Position.latitude*(pi/180);
y=Position.longitude*(pi/180);

X=[x(1) x(20)];
Y=[y(1) y(20)];

m=(Y(2)-Y(1))/(X(2)-X(1));
m=-1/m;

x3=(Y(1)-m*X(1))/(1-m);
y3=x3;

for i=2:length(x)
ref2=[x3-X(1) y3-Y(1)];
vec2(i,:)=([x(i)-X(1) y(i)-Y(1)]*ref2')*ref2;
norm_22=norm(ref2)^2;
V2(:,i)=(vec2(i,:))'/norm_22;
res_vec2(:,i)=(vec2(i,:))'/norm_22+[X(1) Y(1)]';
end

x=res_vec2(1,:);
y=res_vec2(2,:);

m=-1/m;
for i = 1:length(x)-1
    a=(sin((x(i+1)-X(1))/2))^2+cos(X(1))*cos(x(i+1))*(sin((y(i+1)-Y(1))/2))^2;
    c=2*atan2(sqrt(a),sqrt(1-a));
    dx(i)=Rg*c;
    if(res_vec2(2,i)-Y(1)-m*(res_vec2(1,i)-X(1))>0)
        dx(i)=Rg*c;
    else
        dx(i)=-Rg*c;
    end
end
dx(i+1) = dx(i); %GPS position in x-axis