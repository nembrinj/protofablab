# Raspberry Pi App

Directory containing all the necessary files and instructions required to run the Raspberry Pi part of the project.

## How to Run

The project uses docker, whenever possible (it is possible for all components but Grafana).

For instruction on how to run the services and the main_server outside docker (for testing purposes), please check their respective directories.


### Docker

We can bring up everything excluding the four services this app needs (including Grafana; it runs as a service) on docker using the command ```docker compose up```.

### Grafana
Grafana is not running as a docker image due to the lack of an existing image compatible with the Raspberry Pi Zero's architecture (armv6). Instead, it runs as a service. Given the limitations imposed by the pi's architecture we had to use an older version of Grafana, we are restricted to using InfluxDB v1. In other words, all queries used for the dashboard are written in InfluxQL, because InfluxDB v1 supports only InfluxQL; Flux is not supported.

Grafana runs on port 3000. Both the username and the password are *admin*.

The location where provisioning is stored for Grafana is */etc/grafana/provisioning*.


## Debugging Tips
Some debugging tips for checking that the tables are created by telegraf as expected/delete the dummy data generated during development.

```
docker exec -it influxdb sh
influx

show databases
show measurements on profab
select * from "profab".."VL53L4CX" limit 10

delete from "APDS9960_processed"
```

We also want to highlight a few useful commands for grafana (the service).
```
sudo systemctl status grafana-server
sudo systemctl start grafana-server
sudo systemctl restart grafana-server
sudo systemctl stop grafana-server
# List all services.
sudo systemctl list-units --type=service
```

The project uses MQTT with 4 different topics (see the main README for information on that). Commands for subscribing/publishing (useful for testing):
```
mosquitto_sub -h ampi.home -p 1883 -t esp32/VL53L4C
mosquitto_pub -h ampi.home -p 1883 -t door/event/locked_unlocked -m "locked_unlocked status=1"
```

