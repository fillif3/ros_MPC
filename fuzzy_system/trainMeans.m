function fis=trainMeans(fis,oldFis,u,inputs,groundTruth,fis_outputs,learningRate)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
errors=groundTruth-fis_outputs';
numberOfInputs=length(fis.Inputs);
numberOfOuputs=length(errors);
numberOfRules=length(fis.Rules);

for i=1:numberOfInputs
    numberOfMemberships=length(fis.Inputs(i).MembershipFunctions);
    for m=1:numberOfMemberships
        
        gradient=0;
        for j=1:numberOfOuputs
            memberships=u(j,:);
            insideHelper=0;
            for k=1:numberOfRules
                if fis.Rules(k).Antecedent(i)==m
                    insideHelper=insideHelper+(oldFis.Outputs(1).MembershipFunctions(k).Parameters-fis_outputs(j))*...
                    memberships(k)*(inputs(j)-oldFis.Inputs(i).MembershipFunctions(m).Parameters(2))/oldFis.Inputs(i).MembershipFunctions(m).Parameters(1);
                end
            end
            
            gradient=gradient+errors(j)/sum(memberships)*insideHelper*2;
        end
        %if abs(gradient)>(1e-10)
        %    disp(gradient)
        %end
        fis.Inputs(i).MembershipFunctions(m).Parameters(2)=...
            fis.Inputs(i).MembershipFunctions(m).Parameters(2)+learningRate*gradient/10;
    end
end