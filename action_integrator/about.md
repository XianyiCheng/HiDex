The action ingegrator performs projected integration given an action policy. 

It is initialized with a setup file (object, environment)
It performs projected integration given: 
    start pose, 
    object velocity, 
    robot object contact points, 
    object environment contact points,
    object environment contact mode.

It outputs: 
    the final reachable pose, 
    object environment contact (in the object frame).

Code: 
Environment setup given the setup file
Initialize Task
Call Task::forward_integration

`
bin/action_integrator /home/xianyi/Research/MCTS/action_integrator/setup.yaml 1,0,0,0.05,0,1,0,0,0,0,1,0.5,0,0,0,1 1,1,0,0,0,0 -0.5,0,0,1,0,0 0.5,0.5,-0.5,0,0,1,0.5,-0.5,-0.5,0,0,1,-0.5,0.5,-0.5,0,0,1,-0.5,-0.5,-0.5,0,0,1 0,0,0,0  
`