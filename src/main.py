# Copyright 2022 iwatake2222
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import argparse
import subprocess
from datetime import datetime
import influxdb_client

import hz


class InfluxDbAccessor:
    def __init__(self, url: str, token: str, org: str, bucket_name: str):
        self.url = url
        self.token = token
        self.org = org
        self.bucket_name = bucket_name

    def create_bucket(self):
        with influxdb_client.InfluxDBClient(url=self.url, token=self.token) as client:
            buckets_api = client.buckets_api()
            buckets = buckets_api.find_buckets().buckets
            my_buckets = [bucket for bucket in buckets if bucket.name==self.bucket_name]
            _ = [buckets_api.delete_bucket(my_bucket) for my_bucket in my_buckets]
            _ = buckets_api.create_bucket(bucket_name=self.bucket_name, org=self.org)

    def write_point(self, measurement_datetime, topic_name: str, hz: float):
        with influxdb_client.InfluxDBClient(url=self.url, token=self.token, org=self.org) as client:
            point_settings = influxdb_client.client.write_api.PointSettings()
            write_api = client.write_api(write_options=influxdb_client.client.write_api.SYNCHRONOUS, point_settings=point_settings)

            p = influxdb_client.Point('ros2_topic')\
                .tag('topic_name', topic_name)\
                .time(measurement_datetime)

            write_api.write(bucket=self.bucket_name, record=p.field('topic_rate_hz', hz))


db: InfluxDbAccessor = None


def update_hz_cb(hz_dict: dict[str, float]):
    measurement_datetime = datetime.now()
    for topic, hz in hz_dict.items():
        # print(f'{topic}: {hz: .03f} [Hz]')
        db.write_point(measurement_datetime, topic, hz)


def subscribe_topic_hz(topic_list: list[str]):
    hz_verb = hz.HzVerb()
    parser = argparse.ArgumentParser()
    hz_verb.add_arguments(parser, "ros2_monitor_grafana")
    args = parser.parse_args()
    args.topic_list = topic_list
    args.window_size = 10   # do not calculate average for a long term
    hz.main(args, update_hz_cb)
    # not reached here


def make_topic_list(topic_list_file: str) -> list[str]:
    topic_list = []
    if topic_list_file is not None:
        with open(topic_list_file, 'r') as f:
            topic_list = f.read().splitlines()
    else:
        topic_list = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True,
                                    text=True)
        topic_list = topic_list.stdout.splitlines()
        topic_list.remove('/parameter_events')
        topic_list.remove('/rosout')
    return topic_list


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('topic_list_file', type=str, nargs='?', default=None)
    parser.add_argument('--url', type=str, default='http://localhost:8086',
                        help='InfluxDB URL')
    parser.add_argument('--token', type=str, default='my-super-secret-auth-token',
                        help='InfluxDB Token')
    parser.add_argument('--org', type=str, default='my-org',
                        help='InfluxDB Organization')
    parser.add_argument('--bucket_name', type=str, default='my-bucket',
                        help='InfluxDB Bucket Name')
    args = parser.parse_args()
    return args


def main():
    args = parse_args()

    global db
    db = InfluxDbAccessor(args.url, args.token, args.org, args.bucket_name)
    db.create_bucket()

    topic_list = make_topic_list(args.topic_list_file)
    subscribe_topic_hz(topic_list)


if __name__ == '__main__':
    main()
