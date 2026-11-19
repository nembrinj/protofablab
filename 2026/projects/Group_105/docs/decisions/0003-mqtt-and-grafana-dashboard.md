# 3. Results go over MQTT to a Grafana dashboard

- Status: accepted
- Date: 2026-11-04, updated 2026-11-19

## Context

The physiotherapist needs to see results across sessions, not only live values, and the clips of the moments that went wrong. The course introduces MQTT and Docker Compose (Mosquitto broker, data recorder, small web server). ROS 2 topics are not a good interface for a dashboard outside the robot system.

## Decision

A bridge node publishes one JSON message per walk, one per flagged moment, and one per session to an MQTT broker. A Docker Compose stack on the laptop stores and shows them:

- Eclipse Mosquitto as the MQTT broker
- Telegraf subscribes to the MQTT topics and writes to InfluxDB
- InfluxDB stores the results over time
- Grafana shows the physiotherapist dashboard: numbers per walk, trends across sessions, and the list of flagged moments
- A small web server serves the clip files; each flagged moment in Grafana links to its clip

## Consequences

- The dashboard is decoupled from ROS 2: anything that speaks MQTT can feed it.
- Uses widely used monitoring tools instead of a custom web page.
- Five containers to configure; all run on the laptop, not on the robot.
- Whether Grafana shows the clips well enough as links is checked when the dashboard is built (26.11).
- Messages carry an anonymised patient ID only, and the data stays on the local network.

## Alternatives considered

- **Node-RED dashboard** (taught in the course): faster to build, weaker for history and trends. Fallback if the Grafana stack takes too long.
- **Course pattern with a CSV recorder and Flask**: simple, but we would have to build charts by hand.
