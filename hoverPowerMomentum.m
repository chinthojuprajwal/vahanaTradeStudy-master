% Estimate hover performance
%
% Inputs:
%  vehicle      - Vehicle type ('tiltwing' or 'helicopter')
%  rProp        - Prop/rotor radius
%  W            - Weight
%  cruiseOutput - Cruise data
%
% Outputs:
%  hoverOutput - Structure with hover performance values
%


function [PMax, FOM, Ct, Cp, Vtip, sigma] = hoverPowerMomentum(rProp,cProp,omega, W)
%Actuator disk theory for Propeller
% Altitude, compute atmospheric properties
rho = 1.225;

% Blade parameters
Cd0 = 0.012; % Blade airfoil profile drag coefficient

nProp = 8; % Number of props / motors
ToverW = 1.7; % Max required T/W to handle rotor out w/ manuever margin,ie each motor is sized to generate 1.7 times the required thrust
k = 1.15; % Effective disk area factor (see "Helicopter Theory" Section 2-6.2)
etaMotor = 0.85; % Assumed electric motor efficiency

% Tip Mach number constraint for noise reasons at max thrust condition
MTip = 0.65;

% Tip speed limit
Vtip = 340.2940 *MTip / sqrt(ToverW); % Limit tip speed at max thrust, not hover
%omega = Vtip / rProp;
Vtip=rProp*omega;
% Thrust per prop / rotor at hover
THover = W / nProp;

%Rotor disc area
PropArea=pi*rProp^2;

%Solidity
sigma = nProp *  cProp*rProp /PropArea;
% Compute thrust coefficient
Ct = THover / (rho * PropArea * Vtip^2);

% Average blade CL (see "Helicopter Theory" section 2-6.3)
AvgCL = 6 * Ct / sigma;

%Cpi
Cpi=Ct^(3/2)/sqrt(2);
Cp0=1/8*sigma*Cd0;

Cp=k*Cpi+Cp0;
FOM=Cpi/Cp;

% Hover Power
PHover =nProp*rho*PropArea*Vtip^3*Cp;

% Battery power
PBattery = PHover / etaMotor;

% Maximum thrust per motor
TMax = THover * ToverW;

% Maximum shaft power required (for motor sizing)
% Note: Tilt-wing multirotor increases thrust by increasing RPM at constant collective
PMax = nProp * TMax * ...
    (k .* sqrt(TMax ./ (2 * rho * pi * rProp.^2)) + ...
    sigma * Cd0 / 8 * (Vtip * sqrt(ToverW))^3 ./ (TMax ./ (rho * pi * rProp.^2)));

% Max battery power
PMaxBattery = PMax / etaMotor;

% Maximum torque per motor
QMax = PMax / (omega * sqrt(ToverW));
end



