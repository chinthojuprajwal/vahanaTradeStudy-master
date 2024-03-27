function objc=objct(x,i,params)


switch i
    case 1
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        vehicle='tiltWing';
        range=params.ranges;%m
        payload=params.load;%kg
        S=x(7);   
        eta_motor=x(9);
        m_gb=x(10);
        rpm=x(8);
        
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5; % kW/kg, including controller and other accessories in 3-5 years
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        
        
        
        %objc= 6.427334084674058*100*   (x(1)^2+x(3)+x(4)+exp(-x(5)));
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
%         [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
%         
%         
%         
%         % Mass estimate
%         mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        % Compute operating cost
        %C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);
        objc= mtow;
%     case 2
%         % For the reserve mission compute energy use
%         %[EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range);
%         
%         %objc=  abs((EReserve-x(6))/x(6));
%         %objc=  abs (EReserve-x(6));
%         objc=0;
    otherwise
        objc=0;
end
end