function fis=trainVaraince(fis,oldFis,u,errors,learningRate);
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
numberOfOuputs=length(fis.Inputs);
for i=1:numberOfInputs
    numberOfMF=length(fis.Inputs(i).MembershipFunctions);
    
    for j=1:numberOfOuputs
        fis.Outputs.MembershipFunctions(i).Parameters=...
            fis.Outputs.MembershipFunctions(i).Parameters+learningRate*sum(errors*u(i,:)/sum(u(i,:)));
    end
end