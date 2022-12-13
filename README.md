# ROS 2 Monitor with Grafana

## About

- Monitor ROS 2 topic rate
- Store topic rate into InfluxDB
- Display dashboard in Grafana for the created database

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
