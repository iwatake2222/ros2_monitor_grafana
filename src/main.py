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
"""
main
"""

import argparse
import subprocess


def make_topic_list(topic_list_file: str):
    """Make topic list from either file or ros2cli"""
    topic_list = []
    if topic_list_file is not None:
        with open(topic_list_file, 'r') as f:
            topic_list = f.read().splitlines()
    else:
        topic_list = subprocess.run(['ros2', 'topic', 'list'],
                                    capture_output=True,
                                    text=True)
        topic_list = topic_list.stdout.splitlines()
    return topic_list


def parse_args():
    """Parse arguments"""
    parser = argparse.ArgumentParser()
    parser.add_argument('topic_list_file', type=str, nargs='?', default=None)
    args = parser.parse_args()

    return args


def main():
    """Main function"""
    args = parse_args()
    topic_list = make_topic_list(args.topic_list_file)
    print(topic_list)
s

if __name__ == '__main__':
    main()
