
from datetime import datetime, timezone



if __name__ == "__main__":


    print("ours")
    left_time  = 1764000177563391520
    right_time = 1764000177543369665
    imu_time =   1764000177562688883
    os_sensor_time = 1751277578056862068

    dt_utc = datetime.fromtimestamp(left_time/1e9, tz=timezone.utc)
    print("left_time " + dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    dt_utc = datetime.fromtimestamp(right_time/1e9, tz=timezone.utc)
    print("right_time " + dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    imu_dt_utc = datetime.fromtimestamp(imu_time/1e9, tz=timezone.utc)
    print("imu_time " + imu_dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    dt_utc = datetime.fromtimestamp(os_sensor_time/1e9, tz=timezone.utc)
    print("os_sensor_time " + dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))


    print("botanic garden")
    left_time = 1666059838350278378
    right_time = 1666059838350278378
    imu_time =  1666059838349512100
    livox_sensor_time = 1666059838320170164
    velodyne_sensor_time = 1666059838309870000


    left_dt_utc = datetime.fromtimestamp(left_time/1e9, tz=timezone.utc)
    print("left_time " + left_dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    right_dt_utc = datetime.fromtimestamp(right_time/1e9, tz=timezone.utc)
    print("right_time " + right_dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    imu_dt_utc = datetime.fromtimestamp(imu_time/1e9, tz=timezone.utc)
    print("imu_time " + imu_dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    dt_utc = datetime.fromtimestamp(livox_sensor_time/1e9, tz=timezone.utc)
    print("livox_sensor_time " + dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))

    dt_utc = datetime.fromtimestamp(velodyne_sensor_time/1e9, tz=timezone.utc)
    print("velodyne_sensor_time " + dt_utc.strftime("%Y-%m-%d %H:%M:%S.%f %Z"))