% Computes Optimization objective and constraints
%
% Inputs:
%  x       - design variales (prop/rotor radius [rProp], cruise speed [V],
%            battery mass [mBattery], motor mass [mMotors], max takeoff
%            mass [mtow])
%  vehicle - string indicating the vehicle type that is being designed
%  range   - design range [m]
%  payload - payload mass [kg]
%
% Outputs:
%  f - direct operating cost per flight [$]
%  c - nonlinear inequality constraints
%      - Available energy in the battery > energy required
%      - Motor power available > motor power required
%      - Takeoff mass > computed mass
%      - For a helicopter, kinetic energy in rotor enough to arrest
%        autorotation descent rate


function [f,c,ceq] = computePerformance(x,vehicle,range,payload)

% Unpack design variables
rProp = x(1);
V = x(2);
mBattery = x(3);
mMotors = x(4);
mtow = x(5);
S=x(7);
rpm=x(8);
eta_motor=x(9);
m_gb=x(10);


% Assumed values
batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
motorPowerDensity = 5; % kW/kg, including controller and other accessories in 3-5 years
dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission

% For the nominal mission compute the energy use, flight time, hover
% performance, and cruise performance
[ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);

% For the reserve mission compute energy use
[EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);

% Mass estimate
mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);

% Compute operating cost
C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);

%% Objective is operating cost per flight
 f = mtow;
f=C.costPerFlight;

%% Constraints





% Constraint on MTOW
c(1) = mass.W - mtow * 9.8;



% Battery sizing
[EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
c(2) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;

% %motor power density
% c(4) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;

% motor sizing based on torque
torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
lb2kg = 0.453592;
c(3) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;



% Constraint on tip speed
% tip_mach=0.65;
% omega=tip_mach*340.294/rProp;
% rpm_max=omega*60/(2*pi);
% c(4)=rpm-rpm_max;
% c(4)=0;

% Ereserve constraint
ceq(1)=abs (EReserve-x(6));



        rho = 1.225;
        % Specify stall conditions
        VStall = 35; % m/s
        CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        ceq(2)=x(7)-((x(5)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        
        
        
        
        ceq(3)= x(9)-motor_eta(rpm);
       
     
        

        
        
        
       ceq(4)= x(10)-mass_gb(rpm,rProp,hoverOutput.PMax);
        
        

if strcmpi(vehicle,'helicopter')
    % Auto-rotation energy constraint => kinetic energy in blades has to be
    % twice that of vehicle in autorotation descent to be able to arrest
    % the descent.
    % See "Helicopter Theory" section 7-5, assume rotor CLmax is twice
    % hover CL. Rotor inertia is approximated as a solid rod.
    c(4) = 0.5 * mass.m * hoverOutput.VAutoRotation^2 - 0.5 * 1/3 * mass.rotor * hoverOutput.Vtip^2;
end

end