function [sOUT]=Psystem(x0, xU,xL, vU,vL,wU,wL,misc)

disp(' ')
disp('----Psystem Begins----')


 
lb=[0.01,10,50,20,100,1,1,2600,0.01,20];
ub=[10, 100, 999, 999, 9999,300,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon(x,xU,xL, vU,vL,wU,wL,misc);
func=@(x) myfun(x,xU,xL, vU,vL,wU,wL,misc);

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1500,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr') ;% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0,A,b,Aeq,beq,lb,ub,constraints,options);

[~,sOUT]=myfun(xopt,xU,xL, vU,vL,wU,wL,misc);
sOUT.xopt=xopt;
sOUT.funcCount = OUTPUT.funcCount;
disp('_____Psystem  ends_________')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [fATC,sOUT]=myfun(x,xU,xL, vU,vL,wU,wL,misc)
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        
        
        
       
        % For the nominal mission compute the energy use, flight time, hover
        % performance, and cruise performance
        [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        % Compute operating cost
        C = operatingCost(vehicle,rProp,flightTime,ENominal,mass,cruiseOutput);
        
        %% Objective is operating cost per flight
        f = mtow;
        f=C.costPerFlight;
        
        
        
        %get original objective
        foriginal =f;
           
        % xsL=[mtoww,Sw,rpmm,eta_motorm,rPropg,Vg,mtowg,Sg,rpmg,eta_motorg,m_gbg];
        %Assemble c: target - response      
        c(1) = diffc(xL(1),mtow);  %from w
        c(2) = diffc(xL(2),S); %r
        c(3) = diffc(xL(3),rpm);   %w
        c(4) = diffc(xL(4),eta_motor);  %w
        c(5) = diffc(xL(5),rProp);  %c
        c(6) = diffc(xL(6),V);    %w     
        c(7) = diffc(xL(7),mtow);  %w
        c(8) = diffc(xL(8),S);  %c
        c(9) = diffc(xL(9),rpm);    %w   
        c(10) = diffc(xL(10),eta_motor);    %w  
        c(11) = diffc(xL(11),m_gb);    %w  
        
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        v = vL; w=wL;
        phi= v.*c+(w.*c).^2;
        
        %ATC objective
        fATC=foriginal/1e6+ sum(phi);
        
        %Pack all data to a struct
	    
        sOUT.rProp=rProp;
        sOUT.V = x(2);
        sOUT.mBattery = x(3);
        sOUT.mMotors = x(4);
        sOUT.mtow = x(5);
        sOUT.Ereserve = x(6);
        sOUT.S=x(7);
        sOUT.rpm=x(8);
        sOUT.eta_motor=x(9);
        sOUT.m_gb=x(10);
        
        
        sOUT.x = x;
        sOUT.xU = xU;
        sOUT.xL = xL;
        sOUT.vU = vU;
        sOUT.vL = vL;
        sOUT.wU = wU;
        sOUT.wL = wL;
        sOUT.misc = misc;
        sOUT.v = v;
        sOUT.w = w;
        sOUT.c=c;
        sOUT.phi=phi;
        sOUT.foriginal=foriginal;
        sOUT.fATC = fATC;
        
        
    end
%Constraint
    function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
        rProp = x(1);
        V = x(2);
        mBattery = x(3);
        mMotors = x(4);
        mtow = x(5);
        Ereserve=x(6);
        S=x(7);
        rpm=x(8);
        eta_motor=x(9);
        m_gb=x(10);
        
        vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        
        % Assumed values
        batteryEnergyDensity = 230; % Expected pack energy density in 3-5 years [Wh/kg]
        motorPowerDensity = 5;
        dischargeDepthReserve = 0.95; % Can only use 95% of battery energy in reserve mission
        
       [~,~,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProp,mBattery,mMotors,mtow,hoverOutput,cruiseOutput,payload,m_gb);
        
        

        

        
        % Constraint on MTOW
        cn(1) = mass.W - mtow * 9.8;
        
        
        
        % Battery sizing
        [EReserve,~,~,~,~] = reserveMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        cn(2) = EReserve - mBattery * batteryEnergyDensity * dischargeDepthReserve / 1000;
        
%         % motor power density
%         cn(4) = hoverOutput.PMax / 1000 - mMotors * motorPowerDensity;
        
        
        %motor sizing based on torque
        torq=0.74*(hoverOutput.PMax/8)/(rpm*2*pi/60);
        lb2kg = 0.453592;   
        cn(3) = 0.3928*(torq^0.8587)*lb2kg*8 - mMotors;
        
        
%         % Constraint on tip speed
%         tip_mach=0.65;
%         omega=tip_mach*340.294/rProp;
%         rpm_max=omega*60/(2*pi);
%         cn(4)=rpm-rpm_max;
%         cn(4)=0;
        

        
        % Ereserve constraint
        ceqn(1)=abs (EReserve-x(6)); 
    end

    function cd= diffc(aT,aR)
        cd=-(aT-aR)/(aT) ; %target - response
%         c=abs(c);
    end


end
