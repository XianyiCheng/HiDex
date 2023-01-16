import subprocess
import yaml

number_of_runs = 150

with open("/home/xianyi/Research/MCTS/data/inhand_all/setup.yaml", "r") as f:
    data = yaml.safe_load(f)
    file_name = "/home/xianyi/Research/MCTS/data/inhand_all/results/" + data["hand_type"] + "_" + data["object"] + ".csv"

continue_process = True
# Start the process and wait for it to complete
while (continue_process):
    print("Running inhand_all")
    result = subprocess.run(["/home/xianyi/Research/MCTS/build/bin/inhand_all"], capture_output=True)
    with open(file_name, 'r') as f:
        csv_lines = sum(1 for line in f)
        print("Collected ", csv_lines, " runs")
        if csv_lines >= number_of_runs:
            continue_process = False
