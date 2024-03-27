function prob = red_mat(prob)
%RED_MAT Summary of this function goes here
%   Detailed explanation goes here
prob.bsize=size(prob.struct,2);
prob.vsize=length(prob.x0);
for i=1:prob.bsize
    
    % finding number of total variable copies required based on selection
    % vector for each block
    m=0;
    for k=1:prob.vsize
        if prob.sel_vec(i,k)==1
            m=m+1;
        end
    end
    
    % maping te reduction matrix
    red_mat{i}=zeros([m,prob.vsize]);
    m=0;
    for k=1:prob.vsize
        if prob.sel_vec(i,k)==1
            m=m+1;
            red_mat{i}(m,k)=1;
        end
    end
end
prob.red_mat=red_mat;
end

