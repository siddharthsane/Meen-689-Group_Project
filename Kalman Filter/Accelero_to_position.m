function [Px,Py,Vx,Vy] = Accelero_to_position(Acceleration)
scale = 0.980;
offset = [-0.1040; 0.14; 9.9138];
ya = Acceleration.Y;
xa = Acceleration.X;

for i = 2:length(ya)
    t(1) = 0;
    t(i) = t(i-1) + 1/50;
end
for i= 1:length(ya)
    Vy(1) = 0;
    Vx(1) = 0;
    bias_x = 0.1*log(1+0.15*t(i));
    bias_y = 0.61*log(1+0.9*t(i));
    Vx(i+1) = Vx(i) + (xa(i)*scale- offset(1)- bias_x)*1/50;
    Vy(i+1) = Vy(i) + (ya(i)*scale- offset(2)- bias_y)*1/50;
end
for i= 1:length(ya)
    Py(1) = 0;
    Px(1) = 0;
    Py(i+1) = Py(i) + Vy(i)*1/50; %Accelerometerposition in y-axis
    Px(i+1) = Px(i) + Vx(i)*1/50; %Accelerometerposition in x-axis
end