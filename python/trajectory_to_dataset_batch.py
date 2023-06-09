import subprocess
import os


path = "/data/pushing_x"
setup_yaml = path + "/setup.yaml"
run_folder = path + "/runs"

subfolders = [f.path for f in os.scandir(run_folder) if f.is_dir()]

for subfolder in subfolders:
    subsubfolders = [f.path for f in os.scandir(subfolder) if f.is_dir()]
    if (len(subsubfolders) > 0):
        # print(subsubfolders[0][-7:]=='dataset')
        print("Already processed: " + subfolder)
        continue
    traj_file = subfolder + "/trajectory.csv"
    output_dir = subfolder + "/dataset"
    print("Processing: " + subfolder)
    subprocess.run(["/home/xianyi/Research/MCTS/build/bin/turn_trajectory_into_dataset", setup_yaml, traj_file, output_dir, "sparse"], capture_output=True)
    