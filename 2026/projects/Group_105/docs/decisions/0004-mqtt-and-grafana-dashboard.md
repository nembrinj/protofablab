# 4. Session results go over MQTT to a Grafana dashboard

- Status: accepted
- Date: 2026-11-04

## Context

The physiotherapist needs to see results across sessions, not only live values. The course introduces MQTT and Docker Compose (Mosquitto broker, data recorder, small web server). ROS 2 topics are not a good interface for a dashboard outside the robot system.

## Decision

A bridge node publishes one JSON message per walk and one per session to an MQTT broker. A Docker Compose stack on the laptop stores and shows them:

- Eclipse Mosquitto as the MQTT broker
- Telegraf subscribes to the MQTT topics and writes to InfluxDB
- InfluxDB stores the time series
- Grafana shows the physiotherapist dashboard: cadence, stops and walking speed per walk, walks not measured, and trends across sessions

## Consequences

- The dashboard is decoupled from ROS 2: anything that speaks MQTT can feed it.
- Uses widely used monitoring tools instead of a custom web page.
- Four containers to configure; all run on the laptop, not on the robot.
- Messages carry an anonymised patient ID only, and the data stays on the local network.

## Alternatives considered

- **Node-RED dashboard** (taught in the course): faster to build, weaker for history and trends. Fallback if the Grafana stack takes too long.
- **Course pattern with a CSV recorder and Flask**: simple, but we would have to build charts by hand.
