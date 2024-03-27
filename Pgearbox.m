function [sOUT]=Pgearbox(x0, xU,xLf, vU,vLf,wU,wL,misc)
% clear
% close all

disp(' ')
disp('----Gearbox Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[0.01,10,100,1,2600,0.01,20];
ub=[10, 100, 9999,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon(x,xU,xLf, vU,vLf,wU,wL,misc);
func=@(x) myfun(x,xU,xLf, vU,vLf,wU,wL,misc);

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',9000,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0,A,b,Aeq,beq,lb,ub,constraints,options);

[~,sOUT]=myfun(xopt,xU,xLf,vU,vLf,wU,wL,misc);
sOUT.xopt=xopt;
sOUT.funcCount = OUTPUT.funcCount;
disp('______Wing ends_________')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [fATC,sOUT]=myfun(x,xU,xL, vU,vL,wU,wL,misc)
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
        
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        %get original objective
        foriginal =0 ;
        
        %Assemble c: target - response
        
        c(1) = diffc(xU(1),rProp);
        c(2) = diffc(xU(2),V);
        c(3) = diffc(xU(3),mtow);
        c(4) = diffc(xU(4),S) ;
        c(5) = diffc(xU(5),rpm);
        c(6) = diffc(xU(6),eta_motor);
        c(7) = diffc(xU(7),m_gb);%Assemble v and w.Make it consistently ordered
        
        %get all phi
        v = vU; w=wU;
        phi= v.*c+(w.*c).^2;
        
        %ATC objective
        fATC=foriginal+ sum(phi);
        
        %Pack all data to a struct
        sOUT.rProp=x(1);
        sOUT.V=x(2);
        sOUT.mtow=x(3);
        sOUT.S=x(4);
        sOUT.rpm=x(5);
        sOUT.eta_motor=x(6);
        sOUT.m_gb=x(7);
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
    function [cn,ceqn] = mycon(x,xU,xL, vU,vL,wU,wL,misc)
        
        
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        
        
        
        [~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        ceqn= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        cn=[];
    end


    function cd= diffc(aT,aR)
        cd=(aT-aR)/(aT) ; %target - response
        %                c=abs(c);
    end

end


