function ob=fun(x,i,v,w,cp_vec_i,var,cp_mat,red_mat,vc,wc,p,params)
con=0;diff=zeros([length(var(:,1)),length(cp_vec_i)]);kn=1;
if i==1
    kn=-1;
end
% diff=zeros([length(cp_vec),length(selvec(1,:))]);
for k=1:length(cp_vec_i)
%     diff(:,k)=   -((var(:,cp_vec_i(k))-pinv(red_mat{i})*x));
diff(:,k)=   kn*diffc(var(:,cp_vec_i(k)),pinv(red_mat{i})*x);
    diff_red=cp_mat{i}{k}*diff(:,k);
    con=con+sum(((cp_mat{i}{k})*v{i}(:,k).*...
        diff_red)+((cp_mat{i}{k})*w{i}(:,k).*diff_red).^2);
end
if i==1
ob=(~wc(i))*obj(x,i,p,params)+con+vc(i)*(obj(x,i,p,params))+wc(i)*wc(i)*obj(x,i,p,params)*obj(x,i,p,params);
end
if i>1
ob=con;
end
end
%if constr=true, add alp instead of obj, else add obj instead of alp

    function c= diffc(aT,aR)
        c=(aT-aR)./(aT+10) ; %target - response   
%c=-(aT-aR);
    end