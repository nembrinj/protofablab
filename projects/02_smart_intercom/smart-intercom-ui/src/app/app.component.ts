import { Component } from '@angular/core';
import { Router } from '@angular/router';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.sass']
})
export class AppComponent {
  title = 'smart-intercom-ui';

  constructor(private router : Router) {

  }

  public getNotificationStatus() : string {
    if(!Notification) {
      return 'not possible'
    }
    return Notification.permission
  }

  public goHome() {
    this.router.navigate(['home']);
  }
}
