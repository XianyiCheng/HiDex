import subprocess
import os


total_number_of_runs = 1000
path = "/home/xianyi/Research/MCTS/data/pushing"
continue_process = True

# Start the process and wait for it to complete
while (continue_process):
    print("Collecting pushing data")
    result = subprocess.run(["/home/xianyi/Research/MCTS/build/bin/hidex_batch", "/home/xianyi/Research/MCTS/data/pushing/batch.yaml"], capture_output=True)
    
    count = 0
    for _, dirnames, _ in os.walk(path):
        count += len(dirnames)
    if (count >= total_number_of_runs):
        continue_process = False
