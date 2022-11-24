import {inject} from '@loopback/core';
import {DefaultCrudRepository} from '@loopback/repository';
import {DbDataSource} from '../datasources';
import {Pushsubscription, PushsubscriptionRelations} from '../models';

export class PushsubscriptionRepository extends DefaultCrudRepository<
  Pushsubscription,
  typeof Pushsubscription.prototype.id,
  PushsubscriptionRelations
> {
  constructor(
    @inject('datasources.db') dataSource: DbDataSource,
  ) {
    super(Pushsubscription, dataSource);
  }
}
