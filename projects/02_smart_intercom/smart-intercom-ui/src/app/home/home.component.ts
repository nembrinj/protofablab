import { Component, OnInit } from '@angular/core';
import { environment } from 'src/environments/environment';

@Component({
  selector: 'app-home',
  templateUrl: './home.component.html',
  styleUrls: ['./home.component.sass']
})
export class HomeComponent implements OnInit {

  isPushSubscriptionSucess = false
  doorbellEvtTime : Date | undefined
  doorbellEvtData : any

  constructor() { }

  async ngOnInit(): Promise<void> {
    this.isPushSubscriptionSucess = await this.registerPushSubscription() || true // TODO
    await this.getLoadDoorbellEvent()
  }

  private async getLoadDoorbellEvent() {
    const resp = await fetch(environment.apiRoot + '/doorbell', {
      method:'GET'
    })

    if(!resp.ok) {
      console.error('unable to load doorbell event', resp)
    } else {
      const doorbellEvt = await resp.json()
      this.doorbellEvtTime = new Date(doorbellEvt.time)
      this.doorbellEvtData = doorbellEvt.data
    }
  }

  public async openDoor() {
    const resp = await fetch(environment.apiRoot + '/door', {
      method:'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({
        action: 'open'
      })
    })
    if(!resp.ok) {
      console.error('unable to open door', resp)
    }
  }

  private async registerPushSubscription() : Promise<boolean> {
    if (!('serviceWorker' in navigator)) {
      console.error('Service Worker isn\'t supported on this browser, disable or hide UI.')
      return false
    }
    if (!('PushManager' in window)) {
      console.error('Push isn\'t supported on this browser, disable or hide UI.')
      return false
    }

    const registration = await this.registerServiceWorker()
    if(!registration) {
      console.error('service worker registration is null')
      return false
    }

    var serviceWorker = registration.installing || registration.waiting || registration.active

    if (!serviceWorker) {
      console.error('service worker is null')
      return false
    }

    console.log('waiting for service worker to be active')
    await this.waitForServiceWorker(serviceWorker)
    console.log('service worker activated')

    const subscribeOptions = {
      userVisibleOnly: true,
      applicationServerKey: this.urlBase64ToUint8Array(
        environment.vapidPublic,
      ), // public vapid key. the private key will be used to send push notifications
    }
    try {
      const pushSubscription = await registration.pushManager.subscribe(subscribeOptions)
      if(pushSubscription) {
        console.log('push subscription registered')
        await this.sendSubscriptionToBackEnd(pushSubscription)
        return true
      } else {
        console.error("push subscription failed")
      }
    } catch(err) {
      console.error("push subscription failed", err)
    }
    return false
  }

  private waitForServiceWorker(serviceWorker : ServiceWorker) : Promise<void> {
    return new Promise<void>(resolve => {
      const interval = setInterval(() => {
        if(serviceWorker.state === 'activated') {
          clearInterval(interval)
          resolve()
        }
      }, 100)
    })
  }

  private async sendSubscriptionToBackEnd(subscription : PushSubscription) {
    const resp = await fetch(environment.apiRoot + '/pushsubscription', {
      method:'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(subscription)
    })

    if(!resp.ok) {
      console.error('unable to save push subscription', resp)
    } else {
      console.log('push subscription saved', resp.status)
    }
  }

  private urlBase64ToUint8Array(base64String : string) : Uint8Array {
    var padding = '='.repeat((4 - base64String.length % 4) % 4);
    var base64 = (base64String + padding)
        .replace(/\-/g, '+')
        .replace(/_/g, '/');

    var rawData = window.atob(base64);
    var outputArray = new Uint8Array(rawData.length);

    for (var i = 0; i < rawData.length; ++i) {
        outputArray[i] = rawData.charCodeAt(i);
    }
    return outputArray;
}

  private async registerServiceWorker() {
    return navigator.serviceWorker
      .register(`/assets/service-worker.js?url=${location.href}`)
      .then(function (registration) {
        console.log('Service worker successfully registered.');
        return registration;
      })
      .catch(function (err) {
        console.error('Unable to register service worker.', err);
        return null;
      });
  }

  public async requestNotificationPermission() {
    if(!Notification) {
      console.error('Browser does not support notificatons')
      return
    }
    if(Notification.permission !== 'granted') {
      const res = await Notification.requestPermission()
    }
  }

  public arePermissionsGranted() : boolean {
    if(!Notification) {
      return false
    }
    return Notification.permission === 'granted'
  }
}
