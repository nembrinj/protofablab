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
      return
    }
    const subscribeOptions = {
      userVisibleOnly: true,
      applicationServerKey: this.urlBase64ToUint8Array(
        'BJJ2P64jPFBQ8jIc_l4SsHMMwMIzEY_6wMvGMSTnO3_rklTg2XtZM8OC3Sw8MlZ8--NsKXvkOTEQmqbW_rl08zc',
      ), // public vapid key. the private key will be used to send push notifications
    }
    const pushSubscription = await registration.pushManager.subscribe(subscribeOptions)
    if(pushSubscription) {
      console.log(JSON.stringify(pushSubscription))
      console.log(pushSubscription)
      this.sendSubscriptionToBackEnd(pushSubscription)
    }
  }

  private async sendSubscriptionToBackEnd(subscription : PushSubscription) {
    const resp = await fetch(environment.apiRoot + '/pushsubscription', { //TODO
      method:'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(subscription)
    })

    if(!resp.ok) {
      debugger
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