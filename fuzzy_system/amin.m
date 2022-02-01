rng('default')
input =0:0.05:2*pi;
sineWave=sin(input);%[input(1:100),100*input(101:end)-495];
plot(input,sineWave)

fis=createGauusianFIS([0;2*pi],6,[-1,1]);
writeFIS(fis,'example')
out= evalfis(fis,input);
[fis,errors]=trainGaussFIS(fis,input',sineWave,0.01,30,10);
out2=evalfis(fis,input);
hold on
plot(input,out)
plot(input,out2)
legend('original signal','starting approximation','approximation after learning')
figure
plot(errors)
legend('error given epoch')
