function prob = weights_init(prob)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
prob.bsize=size(prob.struct,2);
cp_vec=cell([1,prob.bsize]);
v=cell([1,prob.bsize]);
w=cell([1,prob.bsize]);
vc=zeros(size(prob.constr));
wc=zeros(size(prob.constr));
% for i3=1:prob.bsize
%     vc(i3)=zeros(size(prob.constr));
%     wc{i3}=zeros(size(prob.constr));
% end

vsize=prob.vsize;

for i=1:prob.bsize
    cp_vec{i}=[prob.child{i},prob.parent{i}];
    v{i}=zeros(length(cp_vec{i}),vsize)';
    w{i}=ones(length(cp_vec{i}),vsize)'*0.01;
    for i4=1:prob.bsize
        if prob.constr(i4)==1
            vc(i4)=0;
            wc(i4)=0.01;
        else
            vc(i4)=0;
        end
    end
end
prob.cp_vec=cp_vec;
prob.v=v;
prob.vc=vc;
prob.w=w;
prob.wc=wc;
end

