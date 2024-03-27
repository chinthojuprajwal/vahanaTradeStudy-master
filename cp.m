function p1 = cp(prob)
%   Assigns child and parents for all blocks of the heirarchy
%   
%   Creates a cell for each block which consists of array with all the
%   children and parents in result.child and result.parent respectively
p1=prob;
blocks=p1.struct;
for i=1:length(blocks)
    [c,p]=cp_k(blocks,i);
    p1.child{i}=c;
    p1.parent{i}=p;
end
    
end

function [c,p,f]=cp_k(blocks,k)
f2=0;o=1;c=[];p=[];f=0;
for i=1:length(blocks)
    if isa(blocks{i},'double')&& blocks{i}==k
        f=1;
        if i-1>0
            p=blocks{i-1};
        end
        if i==1
            f2=1;
        else
            p=blocks{1};
            f=0;
        end
    end
    if f2==1 && isa(blocks{i},'double') && i~=1
        c(o)=blocks{i};
        o=o+1;
    end
    if isa(blocks{i},'cell')
        if f2==1
            c(o)=blocks{i}{1};
            o=o+1;
        else
            [c,p1,fin]=cp_k(blocks{i},k);
            if p1>0
                p=p1;
            end
            if fin==1
                p=blocks{1};
            end
            
        end
        
    end
end


end
