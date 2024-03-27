function [c,ceq]=cons(x,i,~,~,~,~,~,~,~,~,~,params)
       
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        eta_motor=x(9);
        m_gb=x(10);
        rpm=x(8);

        
        vehicle='tiltWing';
        range=params.ranges;%m
        payload=params.load;%kg
        
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5; % kW/kg, including controller and other accessories in 3-5 years
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        

switch i
    case 1
        % Constraint on available energy (E is in kW-hr)
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
        [~,~,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        
        % Constraint on available motor power (kW)
        tip_mach=0.65;
        omega=tip_mach*340.294/rProp;
        rpm_rotor=omega*60/(2*pi);
        torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
        lb2kg = 0.453592;
        c(1) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;
        % Constraint on MTOW
        c(2) = mass.W - mtow * 9.8;
        c(3) = x(6) - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;
        [EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        ceq(1)=abs (EReserve-x(6));
    case 2
        c = [];
        rho = 1.225;
        % Specify stall conditions
        VStall = 35; % m/s
        CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        ceq(1)=x(7)-((x(5)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        
    case 3
        rpm=x(8);
       ceq(1)= x(9)-motor_eta(rpm);
       c=[];
       
    case 4
        [~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        rpm=x(8);
        r_prop=x(1);
        ceq= x(10)-mass_gb(rpm,r_prop,hoverOutput.PMax);
        c=[];
end
end

