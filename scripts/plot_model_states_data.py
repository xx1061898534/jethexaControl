#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

def plot_model_states_data(csv_file):
    # Load the CSV file
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found.")
        return

    # Plot position data
    plt.figure(figsize=(10, 6))
    plt.plot(data['Elapsed_Time'], data['Position_X'], label='Position X', color='r')
    plt.plot(data['Elapsed_Time'], data['Position_Y'], label='Position Y', color='g')
    plt.plot(data['Elapsed_Time'], data['Position_Z'], label='Position Z', color='b')
    plt.xlabel('Elapsed Time (s)')
    plt.ylabel('Position (m)')
    plt.title('Robot Position Over Time')
    plt.legend()
    plt.grid()

    # Plot orientation data
    plt.figure(figsize=(10, 6))
    plt.plot(data['Elapsed_Time'], data['Orientation_X'], label='Orientation X', color='c')
    plt.plot(data['Elapsed_Time'], data['Orientation_Y'], label='Orientation Y', color='m')
    plt.plot(data['Elapsed_Time'], data['Orientation_Z'], label='Orientation Z', color='y')
    plt.plot(data['Elapsed_Time'], data['Orientation_W'], label='Orientation W', color='k')
    plt.xlabel('Elapsed Time (s)')
    plt.ylabel('Orientation (quaternion)')
    plt.title('Robot Orientation Over Time')
    plt.legend()
    plt.grid()

    # Show the plots
    plt.show()

def plot_position_xy(csv_file):
    # Load the CSV file
    try:
        data = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found.")
        return

    # Plot Position_X vs Position_Y
    plt.figure(figsize=(10, 6))
    plt.plot(data['Position_X'], data['Position_Y'], label='Trajectory', color='b')
    plt.xlabel('Position X (m)')
    plt.ylabel('Position Y (m)')
    plt.title('Robot Trajectory (Position X vs Position Y)')
    plt.legend()
    plt.grid()
    plt.show()

if __name__ == "__main__":
    # Path to the CSV file
    csv_file = "/home/hiwonder/jethexa_vm/src/jethexa_planning/scripts/model_states_data_para.csv"
    #plot_model_states_data(csv_file)
    plot_position_xy(csv_file)
