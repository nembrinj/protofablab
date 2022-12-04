self.addEventListener('push', async function(event) {
  console.log('Received notification: ', event.data.text())
  event.waitUntil(self.registration.showNotification(
    'SmartIntercom',
    {
      body: event.data.text()
    }
  ))
})
