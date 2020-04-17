clear all
load('sensorlog_40.4m.mat')
data = xlsread('meen689.xlsx');

%Accelerometer
[Px,Py,Vx,Vy] = Accelero_to_position(Acceleration);
%GPS
[dx,dy]=Gps_to_bodyframe(Position); %GPS position
Vgy = Position.speed; %GPS speed in y-axis
%Ultrasonic
data(126:150) = 22.31; % Ultrasonic sensor data

%% settings
Na = length(Px); % number of time steps for acceleration

dta = 1/50; % time between time steps for acceleration
dtg = 1; % time between time steps for gps

sig_dyn = 0.1*[1; 1; 1; 1; 1; 1]; % user input of standard deviation of Model noice
sig_acc = 70*[1; 1; 1; 1; 1; 1]; % user input of standard deviation of accelerometer noise
sig_gps = 0.1*[1; 1; 1; 1; 1; 1]; % user input of standard deviation of GPS noise
sig_ult = 0.1*[1; 1; 1; 1; 1; 1]; % user input of standard deviation of Ultrasonic sensor noise

Q = diag(sig_dyn(1:6).^2); % process noise covariance matrix
Ra = diag(sig_acc(1:6).^2); % measurement noise covariance matrix Accelerometer
Rga = diag(sig_gps(1:6).^2);% measurement noise covariance matrix GPS
Ru = diag(sig_ult(1:6).^2);% measurement noise covariance matrix Ultrasonic

A = [1 0 0 0 0 0 ;0 1 0 0 1*dta 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0; 0 0 0 0 0 1]; % state matrix
B = eye(6)*dta; % control-input matrix
H = eye(6); % measurement matrix

%% Kalman filter simulation 
P_diag = zeros(6,Na); % diagonal term of error covariance matrix
% filtering
x_est(:,1) = [0;0;0;0;1;0]; %Initial state
P = eye(6)*10; %Initial Covariance matrix
P_diag(:,1) = diag(P);
K = eye(6);
K_diag(:,1) = diag(K);
    for k = 2:1:length(Px) 
        %%% Prediction
        u = [0;0;0;0;0;0]; %Input for constant velocity system
 
        % predicted state estimate
        x_est(:,k) = A*x_est(:,k-1) + B*u;
        
        % predicted error covariance
        P = A*P*A' + Q;
        
        %%% Update
        %obtain measurement
        if rem(k,50) ~= 0
            z = [Px(k); Py(k);0;Vx(k);Vy(k);0];
            y = z - H*x_est(:,k);
            % Kalman gain
            K = P\H'/(Ra+H*P\H');
            x_est(:,k) = x_est(:,k) + K*y;
            
        end
        if rem(k,50) == 0
            z =[dx(k/50);dy(k/50);0;0;Vgy(k/50);0];
           % measurement residual
            y = z - H*x_est(:,k);
           % Kalman gain
            K = P\H'/(Rga+H*P\H');
            x_est(:,k) = x_est(:,k) + K*y;
            
        end
        if rem(k,16) == 0
            if k/16 <15 && data(k/16) < 1
                z =[data(k/16)-0.2;5.8;0;0;1;0];
                % measurement residual
                y = z - H*x_est(:,k);
                % Kalman gain
                K = P\H'/(Ru+H*P\H');
                x_est(:,k) = x_est(:,k) + K*y;
   
            end
            if k/16 > 15 && data(k/16) < 1 && k/16 <45
                z =[data(k/16)-0.2;14.5;0;0;1;0];
                % measurement residual
                y = z - H*x_est(:,k);
                % Kalman gain
                K = P\H'/(Ru+H*P\H');
                x_est(:,k) = x_est(:,k) + K*y;
                
            end
            if k/16 > 45 && data(k/16) < 1 && k/16 < 85
                z =[data(k/16)-0.2;23.5;0;0;1;0];
                % measurement residual
                y = z - H*x_est(:,k);
                % Kalman gain
                K = P\H'/(Ru+H*P\H');
                x_est(:,k) = x_est(:,k) + K*y;
                
            end
            if k/16 > 85 && data(k/16) < 1 && k/16 <135
                z =[data(k/16)-0.2;35.1;0;0;1;0];
                % measurement residual
                y = z - H*x_est(:,k);
                % Kalman gain
                K = P\H'/(Ru+H*P\H');
                x_est(:,k) = x_est(:,k) + K*y;
             
            end

        end
        % updated error covariance
        P = (eye(6) - K*H)*P;
        P_diag(:,k) = diag(P);
        K_diag(:,k) = diag(K);
    end
    %Plots
    plot(x_est(1,:),'r')
    hold on
    plot(x_est(2,:),'b')
    