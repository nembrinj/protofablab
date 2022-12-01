import {inject, lifeCycleObserver, LifeCycleObserver} from '@loopback/core';
import {juggler} from '@loopback/repository';

const config = {
  name: 'db',
  connector: 'postgresql',
  url: '',
  host: 'database',
  port: 5432,
  user: 'docker',
  password: 'C4iELjeL8Nc7Kzr17tCw',
  database: 'smartintercom'
};

const configLocal = {
  name: 'db',
  connector: 'postgresql',
  url: '',
  host: 'localhost',
  port: 5432,
  user: 'postgres',
  password: 'Vb3XQ7ke1TvWDZvM8Lmj',
  database: 'smartintercom'
};

// Observe application's life cycle to disconnect the datasource when
// application is stopped. This allows the application to be shut down
// gracefully. The `stop()` method is inherited from `juggler.DataSource`.
// Learn more at https://loopback.io/doc/en/lb4/Life-cycle.html
@lifeCycleObserver('datasource')
export class DbDataSource extends juggler.DataSource
  implements LifeCycleObserver {
  static dataSourceName = 'db';
  static readonly defaultConfig = DbDataSource.isLocalExecution() ? configLocal : config;

  constructor(
    @inject('datasources.config.db', {optional: true})
    dsConfig: object = DbDataSource.isLocalExecution() ? configLocal : config,
  ) {
    super(dsConfig);
  }

  private static isLocalExecution() : boolean {
    let local = false
    if(typeof process.env.LOCAL_DATABASE !== 'undefined') {
      local = JSON.parse(process.env.LOCAL_DATABASE)
    }
    return local
  }
}
