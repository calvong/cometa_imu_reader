import sys
import clr
import os
import numpy as np
import time

current_script_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_script_path + "/Cometa_SDK/")

clr.AddReference("Waveplus.DaqSys")
clr.AddReference("Waveplus.DaqSysInterface")

from Waveplus.DaqSysInterface import *
from Waveplus.DaqSys import *
from WaveplusLab.Shared.Definitions import *

# global variables
imu_data = []


class WaveTrackIMU(DaqSystem):
    """

    Refer to Waveplus.Daq.Net software library Release 3.0.0.2

    """

    def __init__(self):
        self.daq = DaqSystem()
        self.software_version = self._get_software_version()
        self.hardware_version = self._get_hardware_version()
        self.firmware_version = self._get_firmware_version()
        self.n_sensors = self.installed_sensors()

        # create empty data array
        self.data = None

        self.buf = DataSyncBuffer()
        self.daq.StateChanged += daq_state_changed_handler
        self.daq.DataAvailable += data_available_handler

    def start_capturing(self, period=100):
        """Start capturing data

        :param period: DataAvailableEventPeriod = {100, 50, 25, 10}ms
        """

        if period == 100:
            self.daq.StartCapturing(self.DataAvailableEventPeriod.ms_100)
        elif period == 50:
            self.daq.StartCapturing(self.DataAvailableEventPeriod.ms_50)
        elif period == 25:
            self.daq.StartCapturing(self.DataAvailableEventPeriod.ms_25)
        elif period == 10:
            self.daq.StartCapturing(self.DataAvailableEventPeriod.ms_10)

        print("Started capturing Cometa IMU data")

    def stop_capturing(self):
        self.daq.StopCapturing()

        n_pop = len(imu_data) % self.n_sensors
        for i in range(n_pop):
            imu_data.pop()

        n_interval = len(imu_data)
        for i in range(n_interval):
            if i == 0:
                self.data = imu_data[0]
            else:
                self.data.ax.append(imu_data[i].ax)

        print("Stopped capturing Cometa IMU data")

    def configure_imu(self, acquisition_type=None, accel_fullscale=None, gyro_fullscale=None):
        """

        :param acquisition_type:
        :param accel_fullscale:
        :param gyro_fullscale:
        :return:
        """

        if acquisition_type is None:
            acquisition_type = self.ImuAcqType.Fused6xData_284Hz
        if accel_fullscale is None:
            accel_fullscale = self.AccelerometerFullScale.g_2
        if gyro_fullscale is None:
            gyro_fullscale = self.GyroscopeFullScale.dps_500

        config = self.daq.CaptureConfiguration()
        config.IMU_AcqType = acquisition_type
        self.daq.ConfigureCapture(config)

        # get configure data from IMU 9
        imu_conf = self.daq.SensorConfiguration(9)
        imu_conf.SensorType = self.SensorType.INERTIAL_SENSOR
        imu_conf.AccelerometerFullScale = accel_fullscale
        imu_conf.GyroscopeFullScale = gyro_fullscale

        # writing the config to all the IMUs
        for i in range(self.n_sensors):
            self.daq.ConfigureSensor(imu_conf, i+1)

    def get_state(self):
        """Get the current state of the Waveplus system

        :return: state of the system in string
        """

        state = ""
        if self.daq.State == DaqSystem.DeviceState.NotConnected:
            state = "NotConnected"
        elif self.daq.State == DaqSystem.DeviceState.Initializing:
            state = "Initializing"
        elif self.daq.State == DaqSystem.DeviceState.CommunicationError:
            state = "CommunicationError"
        elif self.daq.State == DaqSystem.DeviceState.InitializingError:
            state = "InitializingError"
        elif self.daq.State == DaqSystem.DeviceState.Idle:
            state = "Idle"
        elif self.daq.State == DaqSystem.DeviceState.Capturing:
            state = "Capturing"
        else:
            state = "ShouldNotHappen"

        return state

    def enable_sensor(self, sensor=0, wait=10):
        """Enable a specific sensor

        :param sensor: sensor number. Value 0 = all sensors
        """
        self.daq.EnableSensor(sensor)

        print("Wait for " + str(wait) + "s to ensure sensors are enabled")
        time.sleep(wait)

    def disable_sensor(self, sensor):
        """Disable a specific sensor

        :param sensor: sensor number. Value 0 = all sensors
        """
        self.daq.DisableSensor(sensor)

    def installed_sensors(self):
        """Get number of sensors installed"""
        return self.daq.InstalledSensors

    def turn_sensor_led_on(self, sensor):
        """Activates blinking on a specific sensor's LED

        :param sensor: sensor number. Value 0 = all sensors
        """
        self.daq.TurnSensorLedOn(sensor)

    def turn_all_sensor_led_on(self):
        """Activates blinking on all sensors' LED"""
        self.daq.TurnAllSensorLedsOn()

    def turn_all_sensor_led_off(self):
        """Disable blinking on all sensors' LED"""
        self.daq.TurnAllSensorLedsOff()

    def _get_software_version(self):
        """Get the Waveplus.Daq.NET library version

        :return library version in string
        """

        prefix = "Waveplus.Daq.NET library version: "

        major = self.daq.SoftwareVersion.Major
        minor = self.daq.SoftwareVersion.Minor
        build = self.daq.SoftwareVersion.Build
        revision = self.daq.SoftwareVersion.Revision

        return prefix + str(major) + '.' + str(minor) + '.' + str(build) + '.' + str(revision)

    def _get_hardware_version(self):
        """Get hardware version for the sensors

        :return: a list of version number
        """

        prefix = "Hardware version: "

        major = self.daq.HardwareVersion[0].Major
        minor = self.daq.HardwareVersion[0].Minor

        version = prefix + str(major) + '.' + str(minor)

        return version

    def _get_firmware_version(self):
        """Get firmware version for the sensors

        :return: a list of version number
        """

        prefix = "Firmware version: "

        major = self.daq.FirmwareVersion[0].Major
        minor = self.daq.FirmwareVersion[0].Minor

        version = prefix + str(major) + '.' + str(minor)

        return version

    class ImuAcqType:
        """IMU acquisition type enum for CaptureConfiguration
        """
        RawData = 0
        Fused9xData_142Hz = 1
        Fused6xData_284Hz = 2
        Fused9xData_71Hz = 3
        Fused6xData_142Hz = 4
        Mixed6xData_142Hz = 5

    class SensorType:
        """Sensor type enum for SensorConfiguration
        """

        EMG_SENSOR = 0
        INERTIAL_SENSOR = 1
        ANALOG_GP_SENSOR = 2
        FSW_SENSOR = 3

    class AccelerometerFullScale:
        """Accelerometer full scale enum for SensorConfiguration
        """
        g_2 = 0
        g_4 = 1
        g_8 = 2
        g_16 = 3

    class GyroscopeFullScale:
        """Gyro full scale enum for SensorConfiguration
        """
        dps_250 = 0
        dps_500 = 1
        dps_1000 = 2
        dps_2000 = 3

    class DataAvailableEventPeriod:
        """Enum for time interval between two consecutive DataAvailableEvents events
        """
        ms_100 = 0
        ms_50 = 1
        ms_25 = 2
        ms_10 = 3


class ImuDataType:
    """Data structure for IMU data"""

    n_sample = 0

    # acceleration
    ax = None
    ay = None
    az = None

    # gyro
    gx = None
    gy = None
    gz = None

    # quaternion
    qw = None
    qx = None
    qy = None
    qz = None


def daq_state_changed_handler(source, args):
    # print("State changed to " + args.State)
    pass

def data_available_handler(source, args):

    global imu_data

    n_sensors = 16  # TODO: find a better to get this number?
    raw_data = ImuDataType()
    raw_data.ax = []
    raw_data.ay = []
    raw_data.az = []
    raw_data.gx = []
    raw_data.gy = []
    raw_data.gz = []
    raw_data.qw = []
    raw_data.qx = []
    raw_data.qy = []
    raw_data.qz = []

    print("hi once")

    # using ScanNumber instead of SamplesNumber for some reasons...
    for n in range(n_sensors):
        for i in range(args.ScanNumber):
            raw_data.qw.append(args.ImuSamples[n, 0, i])
            raw_data.qx.append(args.ImuSamples[n, 1, i])
            raw_data.qy.append(args.ImuSamples[n, 2, i])
            raw_data.qz.append(args.ImuSamples[n, 3, i])

            raw_data.ax.append(args.AccelerometerSamples[n, 0, i])
            raw_data.ay.append(args.AccelerometerSamples[n, 1, i])
            raw_data.az.append(args.AccelerometerSamples[n, 2, i])

            raw_data.gx.append(args.GyroscopeSamples[n, 0, i])
            raw_data.gy.append(args.GyroscopeSamples[n, 1, i])
            raw_data.gz.append(args.GyroscopeSamples[n, 2, i])

            # print(args.ImuSamples[8, 1, i])
        imu_data.append(raw_data)


