function [fis,meanSquareErrorPerIteration] = trainGaussFIS(fis,input,groundTruth,learningRate,iterations,numberOfInputsPerBatch)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
numberOfOutputs=length(groundTruth);
out=zeros(numberOfInputsPerBatch,1);
memberships=zeros(numberOfInputsPerBatch,length(fis.Rules));
numberOfBatches=floor(numberOfOutputs/numberOfInputsPerBatch);
meanSquareErrorPerIteration=zeros(1,iterations+1);
outputTester =evalfis(fis,input);
meanSquareErrorPerIteration(1)=norm(groundTruth-outputTester);
for i =1:iterations
    
    ix = randperm(numberOfOutputs);
    input = input(ix,:);
    groundTruth = groundTruth(ix);
    for b=1:numberOfBatches
        oldFis=fis;       
        for j=((b-1)*numberOfInputsPerBatch+1):b*numberOfInputsPerBatch
            [out(mod(j-1,10)+1,:),~,~,~,memberships(mod(j-1,10)+1,:)]=evalfis(fis,input(j,:));
        end
        try
        fis=trainOutputs(fis,memberships,groundTruth((b-1)*...
                numberOfInputsPerBatch+1:b*numberOfInputsPerBatch),out,learningRate);
        catch
            k=1;
        end
        try
        fis=trainVaraince(fis,oldFis,memberships,input((b-1)*...
               numberOfInputsPerBatch+1:b*numberOfInputsPerBatch,:),groundTruth((b-1)*...
               numberOfInputsPerBatch+1:b*numberOfInputsPerBatch),out,learningRate);
        catch
           k=1;
        end
         try
         fis=trainMeans(fis,oldFis,memberships,input((b-1)*...
                 numberOfInputsPerBatch+1:b*numberOfInputsPerBatch,:),groundTruth((b-1)*...
                 numberOfInputsPerBatch+1:b*numberOfInputsPerBatch),out,learningRate);
         catch
             k=1;
         end


    end
    
outputTester =evalfis(fis,input);
meanSquareErrorPerIteration(i+1)=norm(groundTruth-outputTester);
end