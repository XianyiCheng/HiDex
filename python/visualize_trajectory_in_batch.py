import subprocess
import os

path = os.getcwd()+"/data/pushing_all_dir"
setup_yaml = path + "/setup.yaml"
run_folder = path+ "/runs"

subfolders = [f.path for f in os.scandir(run_folder) if f.is_dir()]

for subfolder in subfolders:
    print("Visualizing trajectory in " + subfolder)
    traj_file = subfolder + "/trajectory.csv"
    subprocess.run(["/home/xianyi/Research/MCTS/build/bin/hidex_trajectory_visualizer", path, traj_file], capture_output=True)
    
