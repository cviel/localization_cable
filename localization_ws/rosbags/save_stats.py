import csv
import rclpy
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time

def calculate_stats(messages, field):
    data = np.array([getattr(msg, field) for msg in messages])
    mean = np.mean(data)
    std = np.std(data)
    return mean, std

def read_rosbag(bag_file, topics, start_time_sec, end_time_sec):
    storage_options = rosbag2_py.StorageOptions(uri=bag_file, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    
    messages_by_topic = {topic: [] for topic in topics}
    first_timestamp = None
    
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        
        if topic in topics:
            if first_timestamp is None:
                first_timestamp = timestamp

            elapsed_time_sec = (timestamp - first_timestamp) / 1e9

            if start_time_sec <= elapsed_time_sec <= end_time_sec:
                msg_type = get_message(type_map[topic])
                msg = deserialize_message(data, msg_type)
                messages_by_topic[topic].append(msg)
    
    return messages_by_topic

def main():
    bag_file = 'rosbags/L600z256xy'
    topics = ['/interval/x', '/interval/x_empty', '/interval/y', '/interval/y_empty', '/sensor/z']
    start_time_sec = 52.063  # Start time in seconds
    end_time_sec = 100   # End time in seconds (adjust as needed)
    
    messages_by_topic = read_rosbag(bag_file, topics, start_time_sec, end_time_sec)

    with open('error.csv', 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["/sensor/z", "/interval/x_empty_lb", "/interval/y_empty_lb", "/interval/x_diam", "/interval/y_diam", "target_x", "target_y", "L"])

        if f.tell() == 0:
            writer.writeheader()
    
        dict = {}
        for topic, messages in messages_by_topic.items():
            if not messages:
                print(f'No messages found for topic: {topic} in the given time range')
                exit(1)
            
            if topic.startswith('/sensor'):
                mean, std = calculate_stats(messages, 'data')
                print(f'Topic: {topic} Mean: {mean}, Std: {std}')

                dict[topic] = mean

            elif topic.startswith('/interval'):
                mean_lb, std_lb = calculate_stats(messages, 'lb')
                mean_ub, std_ub = calculate_stats(messages, 'ub')
                print(f'Topic: {topic} Mean lb: {mean_lb}, Std lb: {std_lb}')
                print(f'Topic: {topic} Mean ub: {mean_ub}, Std ub: {std_ub}')
                print(f'Mean diameter: {mean_ub - mean_lb}')

                dict[topic+'_lb'] = mean_lb
                dict[topic+'_ub'] = mean_ub
                dict[topic+'_diam'] = mean_ub - mean_lb
        
        filtered_row = {k: v for k, v in dict.items() if k in writer.fieldnames}
        writer.writerow(filtered_row)

        print(start_time_sec, end_time_sec)

        

if __name__ == '__main__':
    rclpy.init()
    main()
    rclpy.shutdown()
