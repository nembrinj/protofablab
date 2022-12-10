import { Component } from '@angular/core';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.sass']
})
export class AppComponent {
  title = 'smart-intercom-ui';

  public getNotificationStatus() : string {
    if(!Notification) {
      return 'not possible'
    }
    return Notification.permission
  }
}
