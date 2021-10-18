from wave_track_imu import*
import time
import matplotlib.pyplot as plt


if __name__ == '__main__':
    imu = WaveTrackIMU()
    imu.turn_all_sensor_led_off()
    imu.configure_imu()
    print(imu.software_version)
    print(imu.hardware_version)
    print(imu.firmware_version)
    print("Number of sensor: ", imu.n_sensors)
    # imu.disable_sensor(0)
    time.sleep(1)
    imu.enable_sensor(0, wait=10)

    imu.start_capturing(100)

    time.sleep(1)

    imu.stop_capturing()

    print(len(imu.data))
    plt.figure()
    plt.plot(imu.data[8].qx)
    plt.show()