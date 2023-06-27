import subprocess
import os


total_number_of_runs = 100
path = "/data/pushing_all_dir"
continue_process = True

# Start the process and wait for it to complete
while (continue_process):
    print("Collecting data")
    result = subprocess.run(["/home/xianyi/Research/MCTS/build/bin/hidex_batch", path + "/batch.yaml"], capture_output=True)
    
    count = 0
    for _, dirnames, _ in os.walk(path):
        count += len(dirnames)
    if (count >= total_number_of_runs):
        continue_process = False
