![screenshot](./docs/screenshot.jpg)

# ROS 2 Monitor with Grafana

## About

- Monitor ROS 2 topic rate
- Store topic rate into InfluxDB
- Display dashboard in Grafana for the created database

![overview](./docs/overview.png)

## How to use

### 1. Install requirements

```sh
pip3 install influxdb_client
```

- You also need ROS 2

### 2. Prepare InfluxDB

- Create your InfluxDB account
- Or, run a local Docker container as described below

### 3. Prepare Grafana

- Create your Grafana account
- Or, run a local Docker container as described below

### 4. Run your ROS 2 application

- You can use my sample application
  - `python3 src/sample_ros_app.py`

### 5. Run ROS 2 Monitor with Grafana

```sh
url=http://localhost:8086
token=my-super-secret-auth-token
org=my-org
bucket_name=my-bucket
python3 src/main.py --url=$url --token=$token --org=$org --bucket_name=$bucket_name
```

- Please replace arguments such as url, token, etc. for your account

### 6. Setup Grafana

- Login to Grafana
- Configure datasource
  - `Configuration` -> `Data sources` -> `Add data source`
  - Select `InfluxDB`, and process the following settings, then click `Save & Test`
    - Query Language: Flux
    - URL: http://localhost:8086
    - User: my-user
    - Password: my-password
    - my-user: my-org
    - Token: my-super-secret-auth-token
    - (Please replace arguments such as url, token, etc. for your account)
- Configure dashboards
  - `Dashboards` -> `Browse` -> `New` -> `Import`
  - `Upload JSON file`
    - Select  `./dashboard/topic_monitor.json`
    - Click `Import`

## (optional) Docker containers

### InfluxDB

```sh
mkdir temp && cd temp
mkdir ./influxdb
mkdir ./influxdb/config
mkdir ./influxdb/data

docker run --rm -d -p 8086:8086 \
  -v $PWD/influxdb/data:/var/lib/influxdb2 \
  -v $PWD/influxdb/config:/etc/influxdb2 \
  -e DOCKER_INFLUXDB_INIT_MODE=setup \
  -e DOCKER_INFLUXDB_INIT_USERNAME=my-user \
  -e DOCKER_INFLUXDB_INIT_PASSWORD=my-password \
  -e DOCKER_INFLUXDB_INIT_ORG=my-org \
  -e DOCKER_INFLUXDB_INIT_BUCKET=my-bucket \
  -e DOCKER_INFLUXDB_INIT_RETENTION=1w \
  -e DOCKER_INFLUXDB_INIT_ADMIN_TOKEN=my-super-secret-auth-token \
  influxdb:2.4.0
```

- Access http://localhost:8086/signin to explorer your database
  - Username: my-user
  - Password: my-password

- Query to explorer data

```
from(bucket: "my-bucket")
  |> range(start: 0, stop: 1d)
  |> filter(fn: (r) => r["_measurement"] == "ros2_topic")
  |> filter(fn: (r) => r["_field"] == "topic_rate_hz")
//   |> filter(fn: (r) => r["topic_name"] == "/topic_1000_ms")
```

### Grafana

```sh
mkdir ./grafana

docker run --name grafana \
  --user `id -u` \
  -v $PWD/grafana:/var/lib/grafana \
  --net host \
  -p 3000:3000 \
  -d --rm grafana/grafana
```

- Access http://localhost:3000/ to show your dashboards
  - Username: admin
  - Password: admin

## Notice

- `./src/hz.py` is retrieved from https://raw.githubusercontent.com/ros2/ros2cli/humble/ros2topic/ros2topic/verb/hz.py and modified.