% This script attempts to minimize direct operating costs (as a proxy for
% ticket price) for different small electric vehicle configurations.
%
% Design variables include high level parameters and variables used to
% converge to a closed solution.
%
% Each vehicle has many assumptions that are described in the different
% model functions.
%

function [massTiltWing,xT] = sizingTradeStudy(range,payload)


% Constants
lb2kg = 0.453592;

% User inputs
plotOn = true;
ranges = range; % m
%payload = payload; % kg

% Reset random number generator for repeatability
rng('default');

% Optimization options
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',15000,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr') ;% ,'PlotFcn','optimplotfvalconstr');

% Number of random optimization restarts to try since there may be local minima
nRestart = 0;

% Flags indicating that previous range has converged for a given config
exitflagH = 1; % Helicopter
exitflagT = 1; % Tilt-wing

% Function / Objective calls
xLast = []; % Last place computeall was called
myf = []; % Use for objective at xLast
myc = []; % Use for nonlinear inequality constraint
myceq=[];

%% Loop over design ranges
for i = 1:length(ranges)
    
    disp(['Range = ',num2str(ranges(i)/1000),' km']);
    range = ranges(i);
    
    
    %% Tilt-Wing Multirotor Configuration
    % Design variables (rotor radius, speed, battery mass, motor mass, mtow)
    vehicle = 'tiltwing';
    disp('Tilt-Wing')
    
    % Initial guess for first iteration, start with previous solution for
    % future iterations
    if i == 1
        x0 = [8, 80, 290, 300, 650,1,1,5000,0.9,16];
    else
        x0 = xT;
    end
    
    % Design variable bounds
    lb = [0.001,10,50,20,100,1,1,2600,0.01,0.01]; % Min cruise at 1.3 * VStall
    ub = [10, 100, 999, 999, 9999,300,30,8800,1,200];
    
    if exitflagT > 0 % Don't run if it hasn't converged at the previous speed
        
        [xT,fval,exitflagT,output] = fmincon(...
            @(x) objfun(x,vehicle,range,payload),...
            x0,[],[],[],[],lb,ub,...
            @(x) confun(x,vehicle,range,payload),...
            options);
%         plot(fval,'bd');hold on;
        % Run Multistart cases to try again
        for k = 1:nRestart
            [xTmp,fvalTmp,exitflagTmp,outputTmp] = fmincon(...
                @(x) objfun(x,vehicle,range,payload),...
                lb + rand(size(x0)).*(ub-lb),[],[],[],[],lb,ub,...
                @(x) confun(x,vehicle,range,payload),...
                options);
            if exitflagTmp>0
                %plot(k+1,fvalTmp,'bd');hold on;
            else
                %plot(k+1,fvalTmp,'rd');hold on;
            end
            % Save if previous solution didn't converge, or if this is a
            % better solution
            if exitflagT < 1 || (fvalTmp < fval && exitflagTmp == 1)
                xT = xTmp;
                fval = fvalTmp;
                exitflagT = exitflagTmp;
                output = outputTmp;
            end
        end
        
        if exitflagT > 0
            
            disp('Min Cost Tilt-Wing Design');
            disp('=========================');
            disp(['Operating Cost [$]: ',num2str(fval)]);
            disp(['  Rotor Radius [m]: ',num2str(xT(1))]);
            disp(['Cruise Speed [m/s]: ',num2str(xT(2))]);
            disp([' Battery Mass [kg]: ',num2str(xT(3))]);
            disp(['   Motor Mass [kg]: ',num2str(xT(4))]);
            disp(['         MTOW [kg]: ',num2str(xT(5))]);
            
            rPropTiltWing(i) = xT(1); %#ok
            VTiltWing(i) = xT(2); %#ok
            mBattery = xT(3);
            mMotors = xT(4);
            mtow = xT(5);
            S=xT(7);
            rpm=xT(8);
            eta_motor=xT(9);
            m_gb=xT(10);
            
            % Compute energy used for simple mission
            [ETiltWing(i),flightTimeTiltWing(i),hoverOutputTiltWing(i),cruiseOutputTiltWing(i)] = simpleMission(vehicle,rPropTiltWing(i),VTiltWing(i),mtow*9.8,range,S,rpm,eta_motor,m_gb); %#ok
            
            % Compute energy for reserve missoin
            [EReserveTiltWing(i),~,~,~,loiterOutputTiltWing(i)] = reserveMission(vehicle,rPropTiltWing(i),VTiltWing(i),mtow*9.8,range,S,rpm,eta_motor,m_gb); %#ok
            
            % Weight estimate
            massTiltWing(i) = configWeight(vehicle,rPropTiltWing(i),mBattery,mMotors,mtow,hoverOutputTiltWing(i),cruiseOutputTiltWing(i),payload,m_gb); %#ok
            
            % Compute operating cost
            CTiltWing(i) = operatingCost(vehicle,rPropTiltWing(i),flightTimeTiltWing(i),ETiltWing(i),massTiltWing(i),cruiseOutputTiltWing(i)); %#ok
            
        else
            disp('Optimization failed for Tilt-Wing!');
            disp(output);
        end
    end
end

%save('tradeStudyResults.mat');

if plotOn
    %plotTradeResults();
end

% Compute objective and constraint values in the same function. Only
% re-run if the inputs have changed.
    function y = objfun(x,vehicle,range,payload)
        
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computePerformance(x,vehicle,range,payload);
            xLast = x;
        end
        
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = confun(x,vehicle,range,payload)
        
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computePerformance(x,vehicle,range,payload);
            xLast = x;
        end
        
        % Now compute constraint functions
        c = myc; % In this case, the computation is trivial
        ceq=myceq;
    end
end
