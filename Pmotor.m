function [sOUT]=Pmotor(x0, xU,xLf, vU,vLf,wU,wL,misc)
% clear
% close all

disp(' ')
disp('----Motor Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[2600,0.01];
ub= [8800,1];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@(x)  mycon(x,xU,xLf, vU,vLf,wU,wL,misc);
func=@(x) myfun(x,xU,xLf, vU,vLf,wU,wL,misc);

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
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
        
        rpm = x(1);
        eta_motor = x(2); %m2
        
        
        
        
        %get original objective
        foriginal =0 ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(xU(1),rpm);
        c(2) = diffc(xU(2),eta_motor);
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        v = vU; w=wU;
        phi= v.*c+(w.*c).^2;
        
        %ATC objective
        fATC=foriginal+ sum(phi);
        
        sOUT.rpm=rpm;
        sOUT.eta_motor=eta_motor;
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
        
        rpm=x(1);
        ceqn(1)= x(2)-motor_eta(rpm);
        cn=[];
        
    end


    function cd= diffc(aT,aR)
        cd=(aT-aR)/(aT) ; %target - response
        %                c=abs(c);
    end

end


