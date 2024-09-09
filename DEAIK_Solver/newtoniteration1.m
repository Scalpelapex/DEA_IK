function x=newtoniteration1(fun,x0,EPS) %�?单牛顿迭代法
%fun即迭代函数，dfun即迭代函数的�?阶导数，x0为迭代初值，EPS为精�?
%f=fcnchk(fun);
k=1;
f = fun;
df=diff(fun);
x1=x0-df(x0).\f(x0); %左除
pos = imag(x1)==0;
x1 = double(x1.*pos);
d = norm(x1-x0);
%d=max(max(abs(x1-x0)));
while (d>=EPS)&&(x1~=0)
%     if ~isreal(x1)
%         %x='wujie';
%         x=0;
%         k=9999;
%         break;
%     end
	x0=double(x1);
	x1=double(x0-df(x0).\f(x0)); %左除
    x1 = double(x1.*(imag(x1)==0));
	d=norm(x1-x0);
    %d=max(max(abs(x1-x0)));
	k=k+1;
    if k>=1000
        %x='fasan';
        x1=0;
        break;
    end
end
x=x1;