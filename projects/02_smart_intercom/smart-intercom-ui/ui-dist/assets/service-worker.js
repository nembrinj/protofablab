self.addEventListener('push', async function(event) {
  console.log('Received notification: ', event.data.text())
  event.waitUntil(self.registration.showNotification(
    'SmartIntercom',
    {
      body: event.data.text()
    }
  ))
})

self.addEventListener('notificationclick', (event) => {
  let url = new URL(location).searchParams.get('url')
  event.notification.close()
  event.waitUntil(
    clients.openWindow(url)
);
});
