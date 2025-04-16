import matplotlib.pyplot as plt
import csv

def draw_trajectory(csv_file):
    x_values = []
    y_values = []

    # Read the trajectory data from the CSV file
    try:
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                x_values.append(float(row['X1']))
                y_values.append(float(row['Y1']))
        print(f"Successfully read {len(x_values)} points from {csv_file}")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return

    # Check if data is empty
    if not x_values or not y_values:
        print("No data to plot. Ensure the CSV file is not empty.")
        return

    # Plot the trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_values, y_values, marker='o', label='Trajectory Path')
    plt.title('Trajectory Design')
    plt.xlabel('X1')
    plt.ylabel('Y1')
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == "__main__":
    csv_file = "/home/hiwonder/jethexa_vm/src/jethexa_planning/scripts/trajectory_data.csv"
    draw_trajectory(csv_file)
