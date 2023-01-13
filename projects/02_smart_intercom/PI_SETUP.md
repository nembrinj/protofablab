# Raspberry Pi Setup

This tutorial describes how to deploy the project to the raspberry pi and make the web application available on `https://protofablab.ch`.

The following steps have to be exectuted in order. Some of them might take a while.

## Build and deploy on a different domain

To deploy the application on a different domain, some configuration files have to be changed and the UI application has to be rebuilt. Since the build of the UI application is very slow on the Pi, this should be done on a PC.

Replace any occurrence of `protofablab.ch` in the following files:

- docker-compose.yml
- nginx.conf
- smart-intercom-ui/src/environment/environment.prod.ts

Then build the ui application:

```sh
cd smart-intercom-ui

# build the ui application to the output folder smart-intercom-ui/ui-dist
npm run build
```

Push these changes to a dedicated git branch.

## Install Docker

To install docker, we use the [Linux Media Delivery System (LMDS)](https://github.com/GreenFrogSB/LMDS).

```sh
sudo apt-get update && sudo apt-get upgrade

# install git
sudo apt install git

# install docker and docker-compose using LMDS
git clone https://github.com/GreenFrogSB/LMDS.git ~/LMDS

cd ~/LMDS

./deploy.sh

# select first option
"Install Docker & Docker-compose"

# reboot the pi after

# reconnect to the pi

sudo systemctl enable docker
```

## Check out project code

By using [sparse checkout](https://git-scm.com/docs/git-sparse-checkout), we only get the files from the relevant project folder.

```sh
mkdir protofablab

cd protofablab

git init

git remote add -f origin https://github.com/nembrinj/protofablab

# only check out relevant folder
git config core.sparseCheckout true

echo "projects/02_smart_intercom" >> .git/info/sparse-checkout

# check out your dedicated git branch instead of main, if you want to deploy to a different domain.
git pull origin main
```

## Build containers and generate SSL certificate

The certbot command will access `protofablab.ch` on port 80 (http). Make sure that the DNS and port-forwarding are configured.

```sh
cd projects/02_smart_intercom/

# delete or comment out ngingx.conf server config for port 443
nano nginx.conf

docker-compose build

docker-compose up -d

# generate ssl certificate (change domain if needed)
docker-compose run --rm  certbot certonly --webroot --webroot-path /var/www/certbot/ -d protofablab.ch

# restore https config
git restore nginx.conf

# restart changed containers
docker-compose build && docker-compose up -d
```

## We are done

If everything worked perfectly, the web application should now be available from the internet on https://protofablab.ch (or your own domain) and the MQTT broker should be running on port 1883 of the Raspberry Pi in the local network.
