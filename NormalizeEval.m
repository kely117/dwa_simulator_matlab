function EvalDB=NormalizeEval(EvalDB)
%Function that normalizes the evaluation value
for i=3:6
    EvalDB(:,i)=(EvalDB(:,i) - min(EvalDB(:,i)))/( max(EvalDB(:,i)) - min(EvalDB(:,i)) + 1e-6 ) ;
end

