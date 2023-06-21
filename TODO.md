### HiDex
[x] update the file path configuration so that they use whatever under the root folder
[x] environment blocks: more primitive shapes
[x] object primitive shapes
[x] object surface point sampling on mesh
[x] generate primitive shape contact surface files, 400 each, normal pointing inward
[x] allow full state in Level 1

- [ ] bugs in quasidynamic 
    - [ ]  Pushing (check isQuasidynamic function)
    - [ ]  Placedown / topple, why no solution found in level 2 search?

- [ ] Think about the RRT + MCTS issue, check my scratch book

- [ ] bugs in pushing along the wall: 
    - [ ] why two fingers are worse? 
    - [ ] why rrt of 1 finger, no full state cannot even find solution?
    - [ ] so random, how to guarantee the discovery of solution
    - [ ] the goal must be penetrating, otherwise cannot touch the wall
    - [ ] it seems that previous setup can influence whether a solution can be found in a different setup... 

[] test different scenarios (objects + environment)
[] write a robot template

[] One finger reorientation with multiple 90 degree flips in different axis

[] fix turn trajectory into dataset

[] Better API for reward and action probability design

### WholeHand
[] Surface Opt: cannot deal with scaling the mesh.
[] SDF improve
[] Calculate fast derivatives with robot Jacobian
[] test with different setups
