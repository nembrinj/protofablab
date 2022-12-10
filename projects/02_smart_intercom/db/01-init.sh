#!/bin/bash
set -e
export PGPASSWORD=$POSTGRES_PASSWORD;
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" --dbname "$POSTGRES_DB" <<-EOSQL
  CREATE USER $APP_DB_USER WITH PASSWORD '$APP_DB_PASS';
  CREATE DATABASE $APP_DB_NAME;
  GRANT ALL PRIVILEGES ON DATABASE $APP_DB_NAME TO $APP_DB_USER;
  \connect $APP_DB_NAME $APP_DB_USER
  BEGIN;
    CREATE TABLE IF NOT EXISTS public.PUSH_SUBSCRIPTION (
      id bigint NOT NULL,
      subscription json NOT NULL,
      CONSTRAINT pushsubscription_pkey PRIMARY KEY (id)
    );

    CREATE TABLE IF NOT EXISTS public.doorbell (
      id serial4 NOT NULL,
      evt_time timestamp NULL,
      evt_data json NULL,
      CONSTRAINT doorbell_pkey PRIMARY KEY (id)
    );
  COMMIT;
EOSQL