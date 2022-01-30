function fis = trainOutputs(fis,u,errors,learningRate)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
numberOfOuputs=length(fis.Outputs.MembershipFunctions);
for i=1:numberOfOuputs
    fis.Outputs.MembershipFunctions(i).Parameters=...
        fis.Outputs.MembershipFunctions(i).Parameters+learningRate*sum(errors*u(i,:)/sum(u(i,:)));
end
end