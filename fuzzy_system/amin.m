rng('default')
input =0:0.05:2*pi;
sineWave=sin(input');
plot(input,sineWave)
% dataA = cumsum(ones(20,3));  % some test data
% p = .7;      % proportion of rows to select for training
% N = size(dataA,1);  % total number of rows 
% tf = false(N,1);    % create logical index vector
% tf(1:round(p*N)) = true;     
% tf = tf(randperm(N));   % randomise order
% dataTraining = dataA(tf,:); 
% dataTesting = dataA(~tf,:);

fis=createGauusianFIS([0;2*pi],10,[-1,1]);

out= evalfis(fis,input);
[fis,errors]=trainGaussFIS(fis,input,sineWave,0.01,20,10);
out2=evalfis(fis,input);
hold on
plot(input,out)
plot(input,out2)
figure
plot(errors)
