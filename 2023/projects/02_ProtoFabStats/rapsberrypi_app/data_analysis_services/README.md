# Data Analysis Services

## Overview

This folder contains 3 different folders, one for each service used to process the data.

### Data Preprocessing Service

This service is intended to run rather often, i.e. every 10 minutes, and it is used to compute the number of people in the laboratory at a given time (based on the data collected by the sensor). We want this to run often in order to be able to prvide fresh statistics for the users, 


### Days of Week Stats Service
Service that runs every day at 4am (we ran it every 5 mins for testing purposes) and computes occupancy average for each day of week. We do not need to run this as often as the previous service given that the numbers outputed by this service do not need to be that fresh.


### Hour Stats Service
Same as "Days of Week Stats Service", but it is indended to run every hour.


## How to Run

These instructions apply to all services. Just replace *day_of_week_stats* with another service's name. 

On Raspberry Pi: create the directory that will hold all the files required by the service.
```
mkdir day_of_week_stats_service
```

On PC: copy the files related to the service to the newly created directory on Raspberry Pi.
```
day_of_week_stats_service
scp ./day_of_week_stats.py ./day_of_week_stats.service ./day_of_week_stats.timer  pi@ampi.home:/home/pi/day_of_week_stats_service
```

On Raspberry Pi: Make sure the *.service* and *.timer* files are in */etc/systemd/system* directory.
```
sudo cp ./day_of_week_stats.service /etc/systemd/system/day_of_week_stats.service
sudo cp ./day_of_week_stats.timer /etc/systemd/system/day_of_week_stats.timer
```

Once all the files are in the right place, the timer can be enabled and started.
```
sudo systemctl daemon-reload
sudo systemctl enable day_of_week_stats.timer
sudo systemctl restart day_of_week_stats.timer
```

After following these steps, the service, together with its timer, should be up and running. One can check the status of the timer/service using the following commands:
```
systemctl status day_of_week_stats.timer
systemctl status day_of_week_stats.service
```

In order to see all the timers, run:
```
systemctl list-timers --all
```
