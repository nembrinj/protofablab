# Dashboard

The physiotherapist's dashboard ([ADR 3](../docs/decisions/0003-mqtt-and-grafana-dashboard.md)). `session_reporter` sends the numbers of each walk over MQTT; Telegraf stores them in InfluxDB; Grafana shows them.

| Container | Role | Port |
| --- | --- | --- |
| `mosquitto` | MQTT broker | 1883, open to the network, for `session_reporter` |
| `telegraf` | Reads `rehab/+/walk` and writes each walk to InfluxDB | |
| `influxdb` | Stores the walks, database `rehab`, measurement `walk` | 8086, this computer only |
| `grafana` | The dashboard "Walking exercise" | 3000 |

## Run

Needs Docker with Compose. From this folder:

```bash
docker compose up -d
```

Open <http://localhost:3000>. Anyone on the network can look at the dashboard without logging in, but not change it. To change it, log in with Grafana's default `admin` / `admin`; Grafana asks for a new password the first time.

The ROS 2 computer sends to this computer's IP address, port 1883: `ros2 launch walk_session session.launch.py broker_host:=<IP>`. On macOS, Docker Desktop must be allowed to accept incoming connections.

Stop with `docker compose down`. The stored walks stay in the `influxdb-data` volume; `docker compose down -v` deletes them.

## What it shows

A walk is one way along the path; a session is one sitting, with many walks. Pick the patient and the session at the top of the page.

- **All sessions of the patient**, in the time range (by default the last 30 days): a table with one row per session (walks, measured walks, stops, steps per minute, speed), and one bar per session for steps per minute and for walking speed, oldest on the left, to show the trend from session to session.
- **One session**, picked in the Session menu (newest first): its walks, walks not measured, stops and steps per minute; one bar per walk for steps per minute and for walking speed, to show a session getting slower; and the table of its walks.

Steps per minute and speed are means over the measured walks, with stops left out. A walk appears a few seconds after it ends, and the page refreshes every 10 s. A new session appears in the Session menu after the page is reloaded.

Each walk is stored at the time it reaches the broker, on the clock of this computer, so a wrong clock on the robot does not move the walks in time.

## Send a test walk

```bash
docker compose exec mosquitto mosquitto_pub -q 1 -t rehab/test/walk \
  -m '{"session_id": "test", "walk": 1, "measured": true, "duration_s": 5.0, "steps": 8, "cadence_spm": 96.0, "speed_mps": 0.7, "stops": 0}'
```

Delete the test walk again:

```bash
curl -s 'http://127.0.0.1:8086/query?db=rehab' --data-urlencode "q=DROP SERIES FROM walk WHERE patient_id='test'"
```

## Privacy

For the local network only: the broker and the database have no logins, so the repository holds no passwords. The messages carry an anonymised patient ID, never a name. Grafana and InfluxDB send no usage reports.
