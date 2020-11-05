clear;clc;close all;format long
% ===================== Definition of Variables ===========================
% Jr : MOI of rotary
% Jp : MOI of propeller
% b# : Damping Coef. for bearing
% c  : Damping Coef due to stiffness
% T  : Magnitude of forcing function
% N# : Gear Ratio [1:n2/n1 & 2: n3/n2]
% k  : stiffness coef.
% t  : Time Vector for analysis and plotting
% t# : index of time vector that correspond to value #
% y0 : Initial conditions

% A,B,C,D  : Matrix Representation of State-Space System dotx = A*x + B*u
% sys      : State-Space System created using ss() function
% yimpulse : y(t) in response to impulse forcing function
% ystep    : y(t) in response to step forcubg function
% yfree    : free response of system due to initial conditions
% y#       : system response to scenario impulse[1] and step[2] inputs

% ------------------------ Declare Variables ----------------
Jr = 15; Jp = 80;
b1 = .2; b2 = .15; 
c = .08; T = 400;
N1 = 2.5; N2 = 4.2; N = N1*N2;
k = [5, 15, 50, 100, 500, 1000];
y0 = [0,0,0,0];

t = [0:.1:2000]';
t20 = find(t==20);
t1980 = find(t==1980);
t460 = find(t==460);
t480 = find(t==480);
t1500 = find(t==1500);
t1480 = find(t==1480);


% This for-loop creates a state-space system for each stiffness coef. and
% performs an impulse and step analysis with the given initial conditions.
% It also return the angular velocity of the propeller as a set of vectors
% for furhter analysis

for i = 1:length(k)
    % Create State Variable Matrix for A,B, C, D using standard for
    % xdot = A*x + B*u
    % y = C*x + D*u
    A = [0, 1, 0, 0; 
        -(k(i)/N^2/Jr), -(c/N^2 + b1)/Jr, k(i)/N/Jr, -c/N/Jr;
         0, 0, 0, 1;
         k(i)/N/Jp, c/N/Jp, -k(i)/Jp, -(b2+c)/Jp];
    B = [0 1/Jr 0 0]';
    C = [0 0 0 1];
    D = [0];


    % Use Control System Toolbox to create state-space system
    sys = ss(A,B,C,D);
    yimpulse = impulse(T*sys,t);
    yfree = initial(sys,y0,t);
    ystep = step(T*sys,t(1:t1500));
    impulseResponse(:,i) = yimpulse + yfree;
    stepResponse(:,i) = ystep + yfree(1:t1500);
    
    num = [N*c, N*k(i)];
    den = [N^2*Jr*Jp,...
        Jp*c+Jp*N^2*b1+Jr*N^2*b2+Jr*N^2*c,...
        Jp*k(i)+b2*c+Jr*N^2*k(i)+N^2*b1*b2+N^2*b1*c,...
        N^2*b1*k(i)+b2*k(i)];
    TransferFunction = tf(num,den)
    p = pole(TransferFunction)
    
    figure()
    subplot(2,2,[1 2])
    hold on;grid on; grid minor
    plot(t,impulseResponse(:,i))
    title(['Impulse Input : Stiffness Coefficient k = ' num2str(k(i))],'Interpreter','latex')
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    subplot(2,2,3)
    hold on;grid on; grid minor
    plot(t(1:200),impulseResponse(1:200,i))
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    subplot(2,2,4)
    hold on;grid on; grid minor
    plot(t(end-200:end),impulseResponse(end-200:end,i))
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    
    figure()
    subplot(2,2,[1 2])
    hold on;grid on; grid minor
    plot(t(1:t1500),stepResponse(:,i))
    title(['Step Input : Stiffness Coefficient k = ' num2str(k(i))],'Interpreter','latex')
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    subplot(2,2,3)
    hold on;grid on; grid minor
    plot(t(t460:t480),stepResponse(t460:t480,i))
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    subplot(2,2,4)
    hold on;grid on; grid minor
    plot(t(t1480:t1500),stepResponse(t1480:end,i))
    xlabel('time [s]','Interpreter','latex');
    ylabel('$\dot{\theta_p}$ [rad/s]','Interpreter','latex');
    
end
%%
for j = 1:length(k)
    fprintf('\nK = %i\n',k(j))    
    
    impulsePeaksStart = find(islocalmax(impulseResponse(1:t20,j)));
    if length(impulsePeaksStart) >= 2
        freqImpulseStart = getFrequency( t(impulsePeaksStart) )
        AmpImpulseStart = getAmp(impulseResponse(1:t20,j))
    end
    
    impulsePeaksEnd = find(islocalmax(impulseResponse(t1980:end,j)));
    if length(impulsePeaksEnd) >= 2
        freqImpulseEnd = getFrequency( t(impulsePeaksEnd) )
        AmpImpulseEnd  = getAmp(impulseResponse(t1980:end,j))
    end
    % ------------------------------- Step ----------------------
    
    stepPeaksStart = find(islocalmax(stepResponse(t460:t480,j)));
    if length(stepPeaksStart) >= 2
        freqStepStart = getFrequency( t(stepPeaksStart) )
        AmpStepStart  = getAmp(stepResponse(t460:t480,j))
    end
    
    stepPeaksEnd = find(islocalmax(stepResponse(t1480:t1500,j)));
    if length(stepPeaksEnd) >= 2
        freqStepEnd = getFrequency( t(stepPeaksEnd) )
        AmpStepEnd  = getAmp(stepResponse(t1480:t1500,j))
    end
end

