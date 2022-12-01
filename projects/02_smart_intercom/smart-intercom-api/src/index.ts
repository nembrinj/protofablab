import {ApplicationConfig, SmartIntercomApiApplication} from './application';
import { MqttService } from './services';

const http = require('http');

export * from './application';

export async function main(options: ApplicationConfig = {}) {
  const app = new SmartIntercomApiApplication(options);
  await app.boot();
  await app.migrateSchema();
  await app.start();

  app.service(MqttService)

  const url = app.restServer.url;
  console.log(`Server is running at ${url}`);
  console.log(`Try ${url}/ping`);

  // ping application to initialize MqttService
  http.get(`${url}/ping`, () => {
    console.log('pinged application')
  })

  return app;
}

if (require.main === module) {
  console.log('Starting application')
  // Run the application
  const config = {
    rest: {
      port: +(process.env.PORT ?? 3000),
      host: process.env.HOST,
      // The `gracePeriodForClose` provides a graceful close for http/https
      // servers with keep-alive clients. The default value is `Infinity`
      // (don't force-close). If you want to immediately destroy all sockets
      // upon stop, set its value to `0`.
      // See https://www.npmjs.com/package/stoppable
      gracePeriodForClose: 5000, // 5 seconds
      openApiSpec: {
        // useful when used with OpenAPI-to-GraphQL to locate your application
        setServersFromRequest: true,
      },
    },
  };
  main(config).catch(err => {
    console.error('Cannot start the application.', err);
    process.exit(1);
  });
}
