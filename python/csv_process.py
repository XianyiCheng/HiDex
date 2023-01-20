import csv
import os

# specify the directory path
directory = "/home/xianyi/Research/MCTS/data/inhand_all/results"

# iterate through all the files in the directory
for filename in os.listdir(directory):
    # check if the file is a csv file
    if filename.endswith(".csv"):
        file_path = os.path.join(directory, filename)
        # open the file and create a new file to write the modified data
        with open(file_path, "r") as file, open(os.path.join(directory, "cleaned_" + filename), "w") as new_file:
            csv_reader = csv.reader(file)
            csv_writer = csv.writer(new_file)

            # iterate through the rows in the file
            for row in csv_reader:
                # check if the row has no size
                if len(row) == 0:
                    continue
                # check if the first column starts with 0
                
                if not row[0]== '0':
                    csv_writer.writerow(row)