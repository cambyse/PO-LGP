function [evalS] = evalScene(evalName,sceneName)
  
evalS.PFC = evalRun(evalName,sceneName,'PFC');
evalS.MPC = evalRun(evalName,sceneName,'MPC');
evalS.DMP = evalRun(evalName,sceneName,'DMP');

end