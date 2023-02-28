import csv
import numpy as np

# Number of points to generate
n = 10
# Coordinates of box corners
x1, y1, x2, y2 = (-1, -1, 1, 1)

# margin to prevent generating points around box corners
margin = 0.01

# Generate evenly spaced x and y values, taking the margin into account
x_values = np.linspace(x1 + margin, x2 - margin, num=n, endpoint=True)
y_values = np.linspace(y1 + margin, y2 - margin, num=n, endpoint=True)

# Generate all possible combinations of x and y values
points = [(x1, 0, y) for y in y_values]
points = points + [(x2, 0, y) for y in y_values]
points = points + [(x, 0, y1) for x in x_values]
points = points + [(x, 0, y2) for x in x_values]

# Generate normals for each point
normals = []
for p in points:
    if p[0] == x1:
        normals.append((1, 0, 0)) # normal pointing towards right
    elif p[0] == x2:
        normals.append((-1, 0, 0)) # normal pointing towards left
    elif p[2] == y1:
        normals.append((0, 0, 1)) # normal pointing upwards
    elif p[2] == y2:
        normals.append((0, 0, -1)) # normal pointing downwards

# Write points and normals to CSV file
with open('points.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    # writer.writerow(["x", "y", "normal_x", "normal_y"])
    for i in range(len(points)):
        # writer.writerow([points[i][0], points[i][1],points[i][2], normals[i][0], normals[i][1], normals[i][2]])
        writer.writerow([points[i][0], points[i][2],points[i][1], normals[i][0], normals[i][2], normals[i][1]])