function fis = trainOutputs(fis,u,groundTruth,outputs,learningRate)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
errors=groundTruth-outputs';
numberOfOuputs=length(errors);
numberOfRules=length(fis.Rules);
for i=1:numberOfRules
    gradient=0;
    for j=1:numberOfOuputs
        gradient=gradient+errors(j)'*u(j,i)/sum(u(j,:));
    end
    fis.Outputs.MembershipFunctions(i).Parameters=...
        fis.Outputs.MembershipFunctions(i).Parameters+learningRate*gradient;
end
end