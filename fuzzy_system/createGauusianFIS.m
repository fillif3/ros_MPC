function fis = createGauusianFIS(InputUniversesRanges,numberOfMF,outputRange)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
fis = sugfis;
s = size(InputUniversesRanges);
numberOfUniverses=s(2);
for i=1:numberOfUniverses
    fis = addInput(fis,InputUniversesRanges(:,i)','NumMFs',numberOfMF(i),'MFType',"gaussmf");
end
ruleVector=ones(size(numberOfMF));
ruleVector(end+1)=1;
ruleVector(end+1)=1;
ruleVector(end+1)=1;
pointer=numberOfUniverses;
fis = addOutput(fis,outputRange,'NumMFs',prod(numberOfMF),'MFType',"constant");
while true
    fis = addRule(fis,ruleVector);
    
    ruleVector(pointer)=ruleVector(pointer)+1;
    ruleVector(pointer+1)=ruleVector(pointer+1)+1;
    
    while true
        if ruleVector(pointer)>numberOfMF(pointer)
            pointer=pointer-1;
            

        else
            pointer=s(2);
            break
        end
        ruleVector(pointer+1)=1;
        if pointer == 0
            return
        else
            ruleVector(pointer)=ruleVector(pointer)+1;
        end
    end
end