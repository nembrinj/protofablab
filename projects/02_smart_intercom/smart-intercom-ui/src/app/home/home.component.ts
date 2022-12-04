import { Component, OnInit } from '@angular/core';
import { environment } from 'src/environments/environment';

@Component({
  selector: 'app-home',
  templateUrl: './home.component.html',
  styleUrls: ['./home.component.sass']
})
export class HomeComponent implements OnInit {

  constructor() { }

  ngOnInit(): void {
    this.registerPushSubscription()
    this.requestNotificationPermission()
  }

  private async registerPushSubscription() {
    if (!('serviceWorker' in navigator)) {
      console.error('Service Worker isn\'t supported on this browser, disable or hide UI.')
      return
    }
    if (!('PushManager' in window)) {
      console.error('Push isn\'t supported on this browser, disable or hide UI.')
      return
    }

    const registration = await this.registerServiceWorker()
    if(!registration) {
      console.error('service worker registration is null')
      return
    }

    var serviceWorker = registration.installing || registration.waiting || registration.active

    if (!serviceWorker) {
      console.error('service worker is null')
      return
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
      } else {
        console.error("push subscription failed")
      }
    } catch(err) {
      console.error("push subscription failed", err)
    }
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
      .register('/assets/service-worker.js')
      .then(function (registration) {
        console.log('Service worker successfully registered.');
        return registration;
      })
      .catch(function (err) {
        console.error('Unable to register service worker.', err);
        return null;
      });
  }

  private async requestNotificationPermission() {
    if(!Notification) {
      console.error('Browser does not support notificatons')
      return
    }
    if(Notification.permission !== 'granted') {
      const res = await Notification.requestPermission()
    }
  }

  public getNotificationStatus() : string {
    if(!Notification) {
      return 'not possible'
    }
    return Notification.permission
  }
}
