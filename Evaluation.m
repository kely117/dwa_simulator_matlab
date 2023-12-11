function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%A function that calculates the evaluation value for each path
evalDB=[];
trajDB=[];
vx_samples = 15;
vth_samples = 10 ;
for vt=linspace(Vr(1),Vr(2),vx_samples)
    for ot=linspace(Vr(3),Vr(4),vth_samples)
        %Trajectory estimation
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model); %evalparam(4) is simtime.
        %Calculation of each evaluation function
        heading=CalcHeadingEval(xt,goal);
        obscost=CalcDistEval(xt,ob,R); %obscost
        distance = CalcDistanceEval(xt,goal);
        vel= Vr(1) - vt;
        evalDB=[evalDB;[vt ot distance heading obscost vel]];
        trajDB=[trajDB;traj];     
    end
end