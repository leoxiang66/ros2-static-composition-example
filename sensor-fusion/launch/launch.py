import launch
import launch_ros.actions
from pathlib import Path


################### user configure parameters for ros2 start ###################
camera_params = [
    {'exposure_time': 5000.0}, 
    {'frequency': 20},
    {'gamma': 0.3},
    {'gain': 20.0},
    {'sync_point': 10000000}, # wait until the next `sync_point`
]

xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'



user_config_path = Path(__file__).resolve().parent.parent.joinpath("config/MID360_config.json").resolve()
user_config_path = str(user_config_path)
################### user configure parameters for ros2 end #####################

lidar_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"cmdline_input_bd_code": cmdline_bd_code},
    {"user_config_path": user_config_path}
]

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='sensor-fusion',  
            executable='sensors',  
            name='multi_nodes',
            output='screen',
            parameters=camera_params + lidar_params
        )
    ])
