import matplotlib.pyplot as plt
import csv

def plot_imu_data(csv_file):
    # Initialize lists to store data
    elapsed_time = []
    imu_x = []
    imu_y = []
    pitch = []
    roll = []
    pid_pitch_output = []
    pid_roll_output = []

    # Read data from CSV file
    with open(csv_file, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            elapsed_time.append(float(row['Elapsed_Time']))
            imu_x.append(float(row['IMU_X']))
            imu_y.append(float(row['IMU_Y']))
            pitch.append(float(row['Pitch']))
            roll.append(float(row['Roll']))
            pid_pitch_output.append(float(row['PID_Pitch_Output']))
            pid_roll_output.append(float(row['PID_Roll_Output']))

    # Create subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Plot IMU data
    axs[0].plot(elapsed_time, imu_x, label='IMU_X')
    axs[0].plot(elapsed_time, imu_y, label='IMU_Y')
    axs[0].set_title('IMU Data')
    axs[0].set_xlabel('Elapsed Time (s)')
    axs[0].set_ylabel('IMU Values')
    axs[0].legend()

    # Plot Pitch and Roll
    axs[1].plot(elapsed_time, pitch, label='Pitch')
    axs[1].plot(elapsed_time, roll, label='Roll')
    axs[1].set_title('Pitch and Roll')
    axs[1].set_xlabel('Elapsed Time (s)')
    axs[1].set_ylabel('Angle (degrees)')
    axs[1].legend()

    # Plot PID Outputs
    axs[2].plot(elapsed_time, pid_pitch_output, label='PID Pitch Output')
    axs[2].plot(elapsed_time, pid_roll_output, label='PID Roll Output')
    axs[2].set_title('PID Outputs')
    axs[2].set_xlabel('Elapsed Time (s)')
    axs[2].set_ylabel('PID Output')
    axs[2].legend()

    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_imu_data('imu_data.csv')