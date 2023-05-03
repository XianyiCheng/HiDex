import subprocess
import os

total_number_of_runs = 1000
path = "/home/xianyi/Research/MCTS/data/pushing"
setup_yaml = path + "/setup.yaml"
run_folder = path + "/runs"

subfolders = [f.path for f in os.scandir(run_folder) if f.is_dir()]

for subfolder in subfolders[5:10]:
    traj_file = subfolder + "/trajectory.csv"
    subprocess.run(["/home/xianyi/Research/MCTS/build/bin/hidex_trajectory_visualizer", setup_yaml, traj_file], capture_output=True)
    
