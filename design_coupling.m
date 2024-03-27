% design-coupling

function design_coupling()

    xLast = []; % Last place computeall was called
    myf = []; % Use for objective at xLast
    myc = []; % Use for nonlinear inequality constraint
    myceq = []; % Use for nonlinear equality constraint


    lb=[0,10,50,20,100,1,1,2600,0.01,0]';
    ub=[10, 100, 999, 999, 9999,200,30,8800,1,200]';
    x0=[1.59808217208494
        31.7019110328425
        214.074569514902
        45.5816806279735
        994.624899765219
        46.7752934390062
        13.7074444953533
        5048.44639263640
        0.987905292847000
        0];
    vehicle = 'tiltwing';
    payload=300;
    range=150000;
    labels=["rProp";
    "V";
    "mBattery";
    "mMotors";
    "mtow";
    "EReserve";
    "S";
    "rpm";
    "eta_motor";
    "m_gb"];

    [f, c, ~] = computePerformance(x0, vehicle, range, payload);

    opt = optimoptions('fmincon', 'Algorithm', 'sqp', 'Display', 'final-detailed', 'MaxIterations', 500, 'ScaleProblem', false, 'UseParallel', true, 'MaxFunctionEvaluations', 10000);
    [xopt, fopt, eopt, oopt] = fmincon(@(x) objfun(x, vehicle, range, payload), x0, [], [], [], [], lb, ub, @(x) constr(x, vehicle, range, payload), opt);
    
    format long
    [fopt, copt, ~] = computePerformance(xopt, vehicle, range, payload)
    
    data_fpdx = zeros(length(xopt), 1);
    data_fmdx = zeros(length(xopt), 1);
    data_cpdx = zeros(length(xopt), length(copt));
    data_cmdx = zeros(length(xopt), length(copt));
    data_dxoptdxpdx = zeros(length(xopt), length(xopt));
    data_dxoptdxmdx = zeros(length(xopt), length(xopt));

    EXITFLAGWRONG = [];
    for idx = 1:length(x0)
        xpdx = xopt;
        xmdx = xopt;
        dx = (ub(idx) - lb(idx))/100.0;
        xpdx(idx) = xopt(idx) + dx;
        xmdx(idx) = xopt(idx) - dx;
        [fpdx, cpdx, ~] = computePerformance(xpdx, vehicle, range, payload);
        [fmdx, cmdx, ~] = computePerformance(xmdx, vehicle, range, payload);
        data_fpdx(idx, 1) = fpdx;
        data_fmdx(idx, 1) = fmdx;
        data_cpdx(idx, :) = cpdx;
        data_cmdx(idx, :) = cmdx;
        [xptmp, fptmp, eptmp, optmp] = fmincon(@(x) objfun(x, vehicle, range, payload), x0, [], [], [], [], lb, ub, @(x) constr_fix(x, vehicle, range, payload, idx, xpdx(idx)), opt);
        [xmtmp, fmtmp, emtmp, omtmp] = fmincon(@(x) objfun(x, vehicle, range, payload), x0, [], [], [], [], lb, ub, @(x) constr_fix(x, vehicle, range, payload, idx, xmdx(idx)), opt);
        if (eptmp == 0) || (emtmp == 0) || (eptmp == -2) || (emtmp == -2)
            EXITFLAGWRONG = [EXITFLAGWRONG, idx];
        end
        for jdx = 1:length(x0)
            data_dxoptdxpdx(idx, jdx) = (xptmp(jdx) - xopt(jdx))/(xpdx(idx) - xopt(idx));
            data_dxoptdxmdx(idx, jdx) = (xmtmp(jdx) - xopt(jdx))/(xmdx(idx) - xopt(idx));
        end
    end

    save('data_dxoptdxpmdx.mat', 'data_dxoptdxpdx', 'data_dxoptdxmdx', '-v7.3');

    l10min = min([min(min(log10(abs(data_dxoptdxpdx)))), min(min(log10(abs(data_dxoptdxmdx))))]);
    l10max = max([max(max(log10(abs(data_dxoptdxpdx)))), max(max(log10(abs(data_dxoptdxmdx))))]);
    l10absmax = max([abs(l10min), abs(l10max)]);
    icm = linspace(-l10absmax, l10absmax, 101)';
    vcm = jet(101);
    figure('Color', [1,1,1]);
    for idx = 1:length(x0)
        for jdx = 1:length(x0)
            rectangle('Position', [idx-0.4, 11-(jdx+0.4), 0.8, 0.8], 'Curvature', 0.1, 'EdgeColor', 'none', 'FaceColor', interp1(icm, vcm, log10(abs(data_dxoptdxpdx(idx, jdx))))); hold on;
            if idx == jdx
                rectangle('Position', [idx-0.3, 11-(jdx+0.3), 0.6, 0.6], 'Curvature', 0.1, 'EdgeColor', 'none', 'FaceColor', [1 1 1]); hold on;
            end
        end
    end
    ybtm = 3;
    for idx = 1:length(icm)
        yi = ybtm + (idx-1)/length(icm)*length(x0)/2;
        xi = length(x0) + 1;
        rectangle('Position', [xi, yi, 0.6, length(x0)/length(icm)/2], 'EdgeColor', 'none', 'FaceColor', vcm(idx, :)); hold on;
        if icm(idx) == min(icm) || icm(idx) == max(icm) || icm(idx) == min(abs(icm))
            text(length(x0) + 1.8, yi, num2str(icm(idx), '%.2f'));
        end
    end
    text(length(x0) + 0.9, yi + 1, 'log$_{10}(\frac{\partial{x}^*}{\partial{x}})$', 'Interpreter','latex', 'FontWeight', 'bold', 'FontSize', 13);
    ax = gca;
    ax.PlotBoxAspectRatio = [1 1 1];
%     ax.XColor = 'none';
%     ax.YColor = 'none';
%      ax.XAxis.TickLabelColor = [0 0 0];
%      ax.YAxis.TickLabelColor = [0 0 0];
    ax.XAxis.TickValues = 1:10;
    ax.XAxis.TickLabels = labels;
    ax.YAxis.TickValues = 1:10;
    ax.YAxis.TickLabels = flip(labels,1);
    ax.XAxisLocation = 'top';
    ax.YLim = [0 11];
    xlabel('Perturbed design variable', 'Color', [0 0 0]);
    ylabel('Influenced design variable', 'Color', [0 0 0]);

    eval(['export_fig ', 'design_coupling', ' -pdf']);


    function y = objfun(x, vehicle, range, payload)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computePerformance(x, vehicle, range, payload);
            xLast = x;
        end
        % Now compute objective function
        y = myf;
    end

    function [c,ceq] = constr(x, vehicle, range, payload)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computePerformance(x, vehicle, range, payload);
            xLast = x;
        end
        % Now compute constraint function
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

    function [c,ceq] = constr_fix(x, vehicle, range, payload, ifix, xfix)
        if ~isequal(x,xLast) % Check if computation is necessary
            [myf,myc,myceq] = computePerformance(x, vehicle, range, payload);
            xLast = x;
        end
        myceq = x(ifix) - xfix;
        % Now compute constraint function
        c = myc; % In this case, the computation is trivial
        ceq = myceq;
    end

end
