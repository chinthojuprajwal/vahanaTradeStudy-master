function prob= cp_mat(prob)
%CP_MAT Summary of this function goes here
%   Detailed explanation goes here
for i=1:prob.bsize
    for k=1:length(prob.cp_vec{i})
        prob.selvecmatch=prob.sel_vec(i,:).*prob.sel_vec(prob.cp_vec{i}(k),:);
        data=eye(length(prob.selvecmatch)).*...
            (ones([length(prob.selvecmatch),1])*prob.selvecmatch);
        data( ~any(data,2), : ) = [];
        prob.cp_mat{i}{k}=data;
    end
end
end

