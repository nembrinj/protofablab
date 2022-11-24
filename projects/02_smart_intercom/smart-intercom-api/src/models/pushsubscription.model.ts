import {Entity, model, property} from '@loopback/repository';

@model()
export class Pushsubscription extends Entity {

  @property({
    id: true,
    type: 'number', 
    required: false,
  })
  id: number;

  @property({
    type: 'string',
    required: true,
  })
  endpoint: string;

  @property({
    type: 'any',
    required: true,
  })
  expirationTime: any;

  @property({
    type: 'object',
    itemType: 'any',
    required: true,
  })
  keys: any;


  constructor(data?: Partial<Pushsubscription>) {
    super(data);
  }
}

export interface PushsubscriptionRelations {
  // describe navigational properties here
}

export type PushsubscriptionWithRelations = Pushsubscription & PushsubscriptionRelations;
