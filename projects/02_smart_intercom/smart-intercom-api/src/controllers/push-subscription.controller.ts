import {
  Count,
  CountSchema,
  Filter,
  FilterExcludingWhere,
  repository,
  Where,
} from '@loopback/repository';
import {
  post,
  param,
  get,
  getModelSchemaRef,
  patch,
  put,
  del,
  requestBody,
  response,
} from '@loopback/rest';
import {Pushsubscription} from '../models';
import {PushsubscriptionRepository} from '../repositories';

export class PushSubscriptionController {
  constructor(
    @repository(PushsubscriptionRepository)
    public pushsubscriptionRepository : PushsubscriptionRepository,
  ) {}

  @post('/pushsubscription')
  @response(200, {
    description: 'Pushsubscription model instance',
    content: {'application/json': {schema: getModelSchemaRef(Pushsubscription)}},
  })
  async create(
    @requestBody({
      content: {
        'application/json': {
          schema: getModelSchemaRef(Pushsubscription, {
            title: 'NewPushsubscription',
            exclude: ['id'],
          }),
        },
      },
    })
    pushsubscription: Omit<Pushsubscription, 'id'>,
  ): Promise<Pushsubscription> {
    return this.pushsubscriptionRepository.create(pushsubscription);
  }

  @get('/pushsubscription/count')
  @response(200, {
    description: 'Pushsubscription model count',
    content: {'application/json': {schema: CountSchema}},
  })
  async count(
    @param.where(Pushsubscription) where?: Where<Pushsubscription>,
  ): Promise<Count> {
    return this.pushsubscriptionRepository.count(where);
  }

  @get('/pushsubscription')
  @response(200, {
    description: 'Array of Pushsubscription model instances',
    content: {
      'application/json': {
        schema: {
          type: 'array',
          items: getModelSchemaRef(Pushsubscription, {includeRelations: true}),
        },
      },
    },
  })
  async find(
    @param.filter(Pushsubscription) filter?: Filter<Pushsubscription>,
  ): Promise<Pushsubscription[]> {
    return this.pushsubscriptionRepository.find(filter);
  }

  @patch('/pushsubscription')
  @response(200, {
    description: 'Pushsubscription PATCH success count',
    content: {'application/json': {schema: CountSchema}},
  })
  async updateAll(
    @requestBody({
      content: {
        'application/json': {
          schema: getModelSchemaRef(Pushsubscription, {partial: true}),
        },
      },
    })
    pushsubscription: Pushsubscription,
    @param.where(Pushsubscription) where?: Where<Pushsubscription>,
  ): Promise<Count> {
    return this.pushsubscriptionRepository.updateAll(pushsubscription, where);
  }

  @get('/pushsubscription/{id}')
  @response(200, {
    description: 'Pushsubscription model instance',
    content: {
      'application/json': {
        schema: getModelSchemaRef(Pushsubscription, {includeRelations: true}),
      },
    },
  })
  async findById(
    @param.path.number('id') id: number,
    @param.filter(Pushsubscription, {exclude: 'where'}) filter?: FilterExcludingWhere<Pushsubscription>
  ): Promise<Pushsubscription> {
    return this.pushsubscriptionRepository.findById(id, filter);
  }

  @patch('/pushsubscription/{id}')
  @response(204, {
    description: 'Pushsubscription PATCH success',
  })
  async updateById(
    @param.path.number('id') id: number,
    @requestBody({
      content: {
        'application/json': {
          schema: getModelSchemaRef(Pushsubscription, {partial: true}),
        },
      },
    })
    pushsubscription: Pushsubscription,
  ): Promise<void> {
    await this.pushsubscriptionRepository.updateById(id, pushsubscription);
  }

  @put('/pushsubscription/{id}')
  @response(204, {
    description: 'Pushsubscription PUT success',
  })
  async replaceById(
    @param.path.number('id') id: number,
    @requestBody() pushsubscription: Pushsubscription,
  ): Promise<void> {
    await this.pushsubscriptionRepository.replaceById(id, pushsubscription);
  }

  @del('/pushsubscription/{id}')
  @response(204, {
    description: 'Pushsubscription DELETE success',
  })
  async deleteById(@param.path.number('id') id: number): Promise<void> {
    await this.pushsubscriptionRepository.deleteById(id);
  }
}
