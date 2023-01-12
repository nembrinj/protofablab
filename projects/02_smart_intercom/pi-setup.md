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

# check out project code

mkdir protofablab

cd protofablab

git init

git remote add -f origin https://github.com/nembrinj/protofablab

# only check out relevant folder
git config core.sparseCheckout true

echo "projects/02_smart_intercom" >> .git/info/sparse-checkout

git pull origin main

cd projects/02_smart_intercom/

# delete or comment out ngingx.conf server config for port 443
nano nginx.conf

docker-compose build

docker-compose up -d

# generate ssl certificate
docker-compose run --rm  certbot certonly --webroot --webroot-path /var/www/certbot/ -d protofablab.ch

# restore https config
git restore nginx.conf

# restart changed containers
docker-compose build && docker-compose up -d
```
