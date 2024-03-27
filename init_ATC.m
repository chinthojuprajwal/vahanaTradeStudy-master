function res= init_ATC(prob)
% associate child and parents
prob.blocks=prob.struct;
prob.bsize=size(prob.blocks,2);
prob.vsize=size(prob.sel_vec,2);
prob=cp(prob);x0=prob.x0;
prob.x0=5*ones(prob.bsize,prob.vsize)';
prob.var=5*ones(prob.bsize,prob.vsize)';

for k=1:prob.bsize
% creating one variable each for all opt blocks and initialising
prob.var(:,k)=x0;
prob.x0(:,k)=x0;
end
% x0 used as initial value for optimisation
%prob.x0=5*ones(prob.bsize,prob.vsize)';

% bounds for optimisation

% weights cell structure for variable consistancy
prob.v=cell([1,prob.bsize]);prob.w=cell([1,prob.bsize]);

% cell struct for size reduction matrix

prob.cost_current=10^5;
prob.cost_prev=10^6;

% weights for "pure" constraint objective functions
prob.vc=zeros(size(prob.constr));
prob.wc=zeros(size(prob.constr));

% create reduction matrix
prob=red_mat(prob);
% Assign weights
prob=weights_init(prob);
% Child-parent matrix
prob=cp_mat(prob);

prob.iter_main=0;
prob.prev_diff=cell([1,prob.bsize]);
prob.f_con=909;

res=prob;
end