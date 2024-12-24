#!/usr/bin/env python3

__authors__ = "David Ho"

import numpy as np
import csv

def get_angle_S_type(t):
    theta = 145/2 * np.sin(t/5 + np.pi/2) + 145/2
    return round(theta, 1)

def get_angle_B_type(t):
    theta = 145/2 * np.cos(t/15 + np.pi) + 145/2
    return round(theta, 1)

def main():
    time_values_s = np.arange(0, 2*np.pi/(1/5) + 1, 0.1)       # Step of 1
    time_values_b = np.arange(0, 2*np.pi/(1/15) + 1, 0.1)       # Step of 1

    # Calculate angles for S type and B type
    angles_S_type = [get_angle_S_type(t) for t in time_values_s]
    angles_B_type = [get_angle_B_type(t) for t in time_values_b]

    # Write angles for S type to CSV file
    with open('../csv/angles_S_type.csv', 'w', newline='') as csvfile_S_type:
        writer_S_type = csv.writer(csvfile_S_type)
        for angle_S in angles_S_type:
            writer_S_type.writerow([angle_S])

    print("CSV file 'angles_S_type.csv' has been generated.")

    # Write angles for B type to CSV file
    with open('../csv/angles_B_type.csv', 'w', newline='') as csvfile_B_type:
        writer_B_type = csv.writer(csvfile_B_type)
        for angle_B in angles_B_type:
            writer_B_type.writerow([angle_B])

    print("CSV file 'angles_B_type.csv' has been generated.")

if __name__ == '__main__':
    main()