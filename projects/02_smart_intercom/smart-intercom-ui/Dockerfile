FROM node:14-alpine3.15

WORKDIR /build
COPY . .

RUN npm i
RUN npm i -g @angular/cli@12.2.10
RUN ng build --output-path=/build/dist

VOLUME /build/dist

