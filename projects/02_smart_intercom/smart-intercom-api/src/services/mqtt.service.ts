import {injectable, /* inject, */ BindingScope, BindingKey} from '@loopback/core';
import { MqttClient, IClientOptions, connect } from 'mqtt';
import { Observable, Subject } from 'rxjs';

@injectable({scope: BindingScope.SINGLETON})
export class MqttService {

  private client: MqttClient
  private isInitialized = false

  constructor() {
    const opts = {
      username: 'api',
      password: 'apipw'
    } as IClientOptions

    console.log('trying to connect to mqtt')
    this.client = connect('mqtt://mqtt/', opts)

    this.client.on('error', err => {
      console.error('mqtt error', err)
    })

    this.client.on('disconnect', res => {
      console.error('mqtt disconnected', res)
      this.isInitialized = false
    })

    this.client.on('connect', () => {
      console.log('mqtt connected!')
      this.isInitialized = true
    })

    this.client.on('message', (topic, message) => {
      console.log('mqtt', topic, message.toString())
    })
  }

  public sendMessage(topic: string, message: string) {
    if(this.isInitialized) {
      this.client.publish(topic, message)
    } else {
      console.error('unable to send message. mqtt not connected')
    }
  }

  public subscribe(topic: string) : Observable<string> {
    const subject = new Subject<string>()
    this.client.subscribe(topic, (err: Error) => {
      if(err) {
        console.log('mqtt: unable to subscribe to topic', err)
        return
      }
      this.client.on("message", (tp, payload) => {
        if(topic === tp) {
          subject.next(payload.toString())
        }
      })
    })
    return subject.asObservable()
  }
}
