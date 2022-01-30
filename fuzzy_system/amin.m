input =0:0.01:2*pi;
sineWave=sin(input);
plot(input,sineWave)
% dataA = cumsum(ones(20,3));  % some test data
% p = .7;      % proportion of rows to select for training
% N = size(dataA,1);  % total number of rows 
% tf = false(N,1);    % create logical index vector
% tf(1:round(p*N)) = true;     
% tf = tf(randperm(N));   % randomise order
% dataTraining = dataA(tf,:); 
% dataTesting = dataA(~tf,:);

fis=createGauusianFIS([0;2*pi],6,[-1,1]);
for i=1:1000
    
end
out = evalfis(fis,input);
hold on
plot(input,out)

