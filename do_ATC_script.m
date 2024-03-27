function res = do_ATC_script(atc,params)
%DO_ATC Summary of this function goes here
figure(100);
ax=gca;
clf(ax);
atc=init_ATC(atc);
atc.fcon_start=10^50;
fcon_c=1;
%ATC While loop
while atc.f_con>atc.tol
    
    atc.cost_prev=atc.cost_current;
    atc.cost_current=0;
    atc.iter_main=atc.iter_main+1;
    
    if atc.iter_main>2 && atc.f_con<atc.fcon_start*(10^(-fcon_c))
        atc.beta=atc.beta*1;
        fcon_c=fcon_c+1;
    end
    atc.f_con=0;
    options = optimoptions('fmincon','Display','off','Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');
    for i=1:atc.bsize
        
        %optimise the obj and penality
        [atc.x_temp,atc.feval,atc.xflag]=fmincon(@fun,...
            atc.red_mat{i}*atc.x0(:,i),[],[],[],[],...
            atc.red_mat{i}*atc.lb,atc.red_mat{i}*atc.ub,@cons...
            ,options,i,atc.v,atc.w,atc.cp_vec{i}...
            ,atc.var,atc.cp_mat,atc.red_mat,atc.vc,atc.wc,atc,params);
        %update initial point
        atc.x0(:,i)=pinv(atc.red_mat{i})*atc.x_temp;
        %update local copy of design variables
        atc.var(:,i)=pinv(atc.red_mat{i})*atc.x_temp;
        
        atc.diff=zeros(length(atc.var(:,i)),length(atc.cp_vec{i}));
        
        % compute conisistancy diff vector for all p-c combinations
        for k=1:length(atc.cp_vec{i})
            atc.k=k;
            atc.diff(:,atc.k)=pinv(atc.red_mat{i})*...
                atc.red_mat{i}*   diffc(atc.var(:,atc.cp_vec{i}(atc.k)),atc.var(:,i));
        end
        if i==1
            atc.diff=-1*atc.diff;
        end
        
        
        atc.objc_pev(i)=obj(atc.x_temp,i,atc,params);
        
        % update v
        atc.v{i}=atc.v{i}+2*atc.w{i}.*atc.w{i}.*    (atc.diff);
        atc.vc(i)=atc.vc(i)+2*atc.wc(i)*atc.wc(i)*obj(atc.x_temp,i,atc,params);
        %diff=[];
        
        % update beta vec based on consistancy and constraint improvements
        if atc.iter_main ~=1
            for l=1:length(atc.cp_vec{i})
                
                atc.beta_vec=(abs(atc.diff(:,l))>atc.gamma*abs(atc.prev_diff{i}(:,l)))*atc.beta+...
                    ~(abs(atc.diff(:,l))>atc.gamma*abs(atc.prev_diff{i}(:,l)));
                atc.w{i}(:,l)=atc.w{i}(:,l).*atc.beta_vec;
            end
            atc.wc(i)=atc.wc(i)*((obj(atc.x_temp,i,atc,params)>atc.gamma*atc.objc_pev(i))*atc.beta+...
                ~(obj(atc.x_temp,i,atc,params)>atc.gamma*atc.objc_pev(i)));
        else
            atc.w{i}=atc.w{i}*atc.beta;
            atc.wc(i)=atc.wc(i)*atc.beta;
        end
        
        atc.cost_current=atc.cost_current+abs(atc.feval);
        % compute constraints with x_temp of system level and add to f_con
        
        if i==1
            atc.fobjval=obj(atc.x_temp,i,atc,params);
            for j=1:atc.bsize
                if atc.constr(j)>0
                    atc.f_con=abs(obj(atc.x_temp,j,atc,params))+atc.f_con;
                end
            end
        end
        
        %add diff to f_con
        
        if i==1
            atc.f_con=atc.f_con+sum(sum(abs(atc.diff)));
        end
        atc.prev_diff{i}=atc.diff;

    end
    %    p.f1= plot(p.iter_main,log10(p.f_con),'r*');hold on;
    %    pause(0.01);
    if atc.iter_main==1
        atc.fcon_start=atc.f_con;
    end
    atc.fObjNew(atc.iter_main)=atc.fobjval;
    atc.cObjNew(atc.iter_main)=atc.f_con;
    
    plot(ax,1:atc.iter_main,atc.fObjNew,'-sb','LineWidth',2,'MarkerFace','b');
    hold(ax,'on')
    plot(ax,1:atc.iter_main,(atc.cObjNew),'-sr','LineWidth',2,'MarkerFace','r');
    hold(ax,'off')
    legend(ax,{'objective','log(residual)'});
    %     xlabel(ax,'Iteration')
    %     ylabel(ax,'Objective/residual')
    title(ax,['Objective: ', num2str(round(atc.fObjNew(atc.iter_main),1)),...
        '   Residual: ', num2str(round(atc.cObjNew(atc.iter_main),1)),'   Gamma: ',num2str(round(atc.gamma,1)),...
        '   Beta: ',num2str(round(atc.beta,1))]);

    %     grid on
    %     set(ax,'XMinorTick','on','YMinorTick','on')
    drawnow
    
    atc.beta=1+0.2*atc.cObjNew(atc.iter_main)-0.1;
    
end
res=atc;
%save('final_result', 'atc');
x_opt=res.x0(:,1);
% mass_atc1=atc_mass(x_opt);
res.mass_struct=atc_mass(x_opt,params);

end


function c= diffc(aT,aR)
l=length(aT);

for i=1:l
    if isnan((aT(i)-aR(i))./(aT(i)))
        c(i)=0;
    else
        c(i)=(aT(i)-aR(i))./(aT(i)+10);
    end
end
%target - response
%c=-(aT-aR);
c=c';
end

