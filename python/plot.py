'''
Simple script to plot the Kalman filter results from the ESP32 drone project. It reads the data from a CSV file and creates plots for the pitch angle and the Kalman filter estimates.
'''

import pandas as pd
import matplotlib.pyplot as plt

import os
os.chdir(os.path.dirname(__file__))     # Change the working directory to the script's location

def plot_data(time_vec: pd.Series, sensor_data: list[pd.DataFrame], title: str):
    '''
    Function to plot the sensor data. The funciton takes the time vector and the sensor data as input. 
    The sensor data is expected to be a list of pd.DataFrame objects, each containing the data for one sensor
    setting. The funtion then creates a plot n subplots, where n is the number of sensor settings. Each subplot
    contains the data for one sensor setting. Each subplot contains the roll angle from the accelerometer, the roll
    angle from the gyroscope and the roll angle from the Kalman filter.
    '''

    plt.figure(figsize=(12, 8))
    for i, df in enumerate(sensor_data):
        plt.subplot(len(sensor_data), 1, i+1)
        plt.plot(time_vec, df["roll"], label="Accelerometer")
        plt.plot(time_vec, df["rollgyro"], label="Gyroscope")
        plt.plot(time_vec, df["rollKalman"], label="Kalman Filter")
        plt.title(f"{title} - Sensor Setting {i+1}")
        plt.xlabel("Time (ms)")
        plt.ylabel("Roll Angle (degrees)")
        plt.legend()



if __name__ == "__main__":
    FOLDER_NAME = r"SensorData"
    HIGH_Q = r"HighQ.json"

    high_q_path = f"{FOLDER_NAME}\\{HIGH_Q}"
    df_high_q = pd.read_csv(high_q_path, sep=",")

    df_high_q.sort_values("timestamp(ms)")

    df_high_q["roll"] = df_high_q["roll"].ffill()
    df_high_q["rollgyro"] = df_high_q["rollgyro"].ffill()
    df_high_q["rollKalman"] = df_high_q["rollKalman"].ffill()
    #print(df_high_q.head())

    good_q_path = f"{FOLDER_NAME}\\{'GoodQ.json'}"
    df_good_q = pd.read_csv(good_q_path, sep=",")
    df_good_q['timestamp(ms)'] = df_good_q['timestamp(ms)'].round(1)

    df_good_q = df_good_q.groupby('timestamp(ms)').first().reset_index()

    print(df_good_q.head(20))

    plot_data(df_good_q["timestamp(ms)"], [df_good_q], "Good Q Setting")

    plt.show()  # Show the plots after all data has been plotted