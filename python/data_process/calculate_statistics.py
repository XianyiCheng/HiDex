import csv
import numpy as np
import os

def calculate_averages_and_standard_deviations(file_name):
    data = np.genfromtxt(file_name, delimiter=',', dtype=float)
    averages = np.mean(data, axis=0)
    standard_deviations = np.std(data, axis=0)
    return averages, standard_deviations


directory = "/home/xianyi/Research/MCTS/data/inhand_goal_finger_locations/results"


# iterate through all the files in the directory
for filename in os.listdir(directory):
    # check if the file is a csv file
    if filename.endswith(".csv"):
        file_path = os.path.join(directory, filename)
        print(filename)

        averages, standard_deviations = calculate_averages_and_standard_deviations(file_path)
        for i in range(len(averages)):
            if i in [0,1,2,4,5,6,7,11]:
                print("{0:.2f}".format(averages[i]), "$\pm$", "{0:.2f}".format(standard_deviations[i]))
