a=0:0.5:2*pi;
b=0:0.5:2*pi;
c= zeros(length(b)*length(a),2);
truth=zeros(length(b)*length(a),1);
for i=1:length(b)
    for j=1:length(a)
        c((i-1)*length(a)+j,:)=[a(j),b(i)];
        truth((i-1)*length(a)+j)=sin(a(j))+sin(b(i));
    end
end
plot3(c(:,1),c(:,2),truth,'*')

fis=createGauusianFIS([0,0;2*pi,2*pi],[5,5],[-1,1]);

out= evalfis(fis,c);
[fis,errors]=trainGaussFIS(fis,c,truth,0.01,20,10);
out2=evalfis(fis,c);
hold on
plot3(c(:,1),c(:,2),out,'g*')
plot3(c(:,1),c(:,2),out2,'r*')
legend('original signal','starting approximation','approximation after learning')
figure
plot(errors)
legend('error given epoch')