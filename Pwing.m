function [sOUT]=Pwing(x0, xU,xLf, vU,vLf,wU,wL,misc)
% clear
% close all

disp(' ')
disp('----Wing Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m
lb=[100,1];
ub= [9999,30];
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
        
        mtow = x(1);
        S = x(2); %m2
        
        
        %get original objective
        foriginal =0 ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(xU(1),mtow);
        c(2) = diffc(xU(2),S);
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        v = vU; w=wU;
        phi= v.*c+(w.*c).^2;
        
        %ATC objective
        fATC=foriginal+ sum(phi);
        
        sOUT.mtow=mtow;
        sOUT.S=S;
        
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
        
        cn = [];
        rho = 1.225;
        % Specify stall conditions
        VStall = 35; % m/s
        CLmax = 1.1; % Whole aircraft CL, section Clmax much higher
        ceqn(1)=x(2)-((x(1)*9.8) / (0.5 * rho * VStall^2 * CLmax));
        
    end


    function cd= diffc(aT,aR)
        cd=(aT-aR)/(aT) ; %target - response
        %                c=abs(c);
    end
end


