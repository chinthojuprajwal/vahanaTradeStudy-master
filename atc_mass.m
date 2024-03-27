function massTiltWing_atc=atc_mass(x,params)

rProp = x(1);
V = x(2);
mBattery = x(3);
mMotors = x(4);
mtow = x(5);
S=x(7);
rpm=x(8);
eta_motor=x(9);
m_gb=x(10);
vehicle='tiltwing';
        range=params.ranges;%m
        payload=params.load;%kg

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
massTiltWing_atc = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
save('mass_atc.mat','massTiltWing_atc');
end