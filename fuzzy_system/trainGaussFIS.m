function fis = trainGaussFIS(fis,input,groundTruth,learningRate,iterations)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
numberOfOutputs=length(groundTruth);
inputMemberships
for i =1:iterations
    oldFis=fis;
    out = evalfis(fis,input);
    errors=abs(groundTruth-out);
    u = getInputMembershipsFromGauus(fis,input); %TODO
    fis=trainOutputs(fis,u,errors,learningRate);
    fis=trainVaraince(fis,oldFis,u,errors,learningRate);
    fis=trainMeans(fis,oldFis,u,errors,learningRate);
    

end