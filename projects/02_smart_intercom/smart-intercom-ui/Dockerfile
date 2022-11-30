FROM node:14-alpine3.15
WORKDIR /app
COPY . .
RUN npm i
RUN npm run build

FROM nginx:alpine
COPY --from=node /app/dist/ /usr/share/nginx/html
