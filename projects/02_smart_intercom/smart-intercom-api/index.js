const router = require('koa-router')();
const Koa = require('koa');
const bodyParser = require('koa-bodyparser');
const cors = require('@koa/cors');

const TOPIC_BELL_RINGING = 'smart_intercom/bell_ringing'
const TOPIC_DOOR_ACTION = 'smart_intercom/door_action'

const PUSH_SUBSCRIPTION_INSERT_STATEMENT = 'INSERT INTO PUSH_SUBSCRIPTION(id, subscription) VALUES($1, $2)'
const DOORBELL_INSERT_STATEMENT = 'INSERT INTO DOORBELL(evt_time, evt_data) VALUES (NOW(), $1)'

let maxSubscriptionId = 0
let pushSubscriptions = {}

// database
const { Client } = require('pg')
const pgClient = new Client({
    connectionString : process.env.DB_CONNECTION
})
initDB()

// webpush
const webpush = require('web-push');
webpush.setVapidDetails(
  process.env.VAPID_SUBJECT,
  process.env.VAPID_PUBLIC,
  process.env.VAPID_PRIVATE,
)

// mqtt
const mqtt = require('mqtt')
const mqttOpts = {
  username: 'api',
  password: 'apipw'
}
console.log('trying to connect to mqtt')
const mqttClient  = mqtt.connect('mqtt://mqtt/', mqttOpts)

router.get('/ping', ping)
  .post('/pushsubscription', savePushSubscription)
  .post('/notify', sendNotification)
  .get('/doorbell', getDoorbell)
  .post('/door', doorAction)

async function ping(ctx) {
  ctx.body = {
    message: "ping received"
  }
}

async function savePushSubscription(ctx) {
  const subscription = ctx.request.body

  for (const [key, value] of Object.entries(pushSubscriptions)) {
    if(JSON.stringify(value) === JSON.stringify(subscription)) {
      ctx.body = { 
        key,
        subscription
      }
      ctx.status = 200
      console.log('Push subscription already saved', key)
      return
    }
  }

  const id = ++maxSubscriptionId
  pushSubscriptions[id] = subscription
  ctx.body = { 
    id,
    subscription
  }
  ctx.status = 201
  
  await saveToDB()
  console.log('Push subscription saved', id)
}

async function sendNotification(ctx) {
  const body = ctx.request.body
  if (!body.message) {
    ctx.throw(400, {'error': '"message" is a required field'})
  }

  if (Object.entries(pushSubscriptions).length == 0) {
    ctx.throw(404, {'error': 'no subscriptions found'})
  }

  sendNotificationToAllSubcriptions(body.message)

  ctx.status = 204

  await saveToDB()
}

async function sendNotificationToAllSubcriptions(message) {
  for (const [id, subscription] of Object.entries(pushSubscriptions)) {
    try {
      const success = await webpush.sendNotification(subscription, message) // TODO: add action to open web application
      console.log('sent notification with status:', success.statusCode)
    } catch(err) {
      console.log('Subscription has expired or is no longer valid: ', err)
      delete pushSubscriptions[id]
    }
  }
}

async function getDoorbell(ctx) {
  try {
    const res = await pgClient.query('SELECT EVT_TIME, EVT_DATA FROM DOORBELL WHERE EVT_TIME = (SELECT MAX(EVT_TIME) FROM DOORBELL)')
    if(res.rows.length === 0) {
      ctx.throw(404, {'error': 'no doorbell events found'})
    }
    ctx.body = {
      time: res.rows[0].evt_time,
      data: res.rows[0].evt_data
    }
    ctx.status = 200
  } catch(err) {
    console.error(err)
    ctx.throw(500, {'error': 'database error'})
  }
}

async function doorAction(ctx) {
  const body = ctx.request.body
  if (!body.action) {
    ctx.throw(400, {'error': '"action" is a required field'})
  }

  console.log('sending action to door:', body.action)
  mqttClient.publish(TOPIC_DOOR_ACTION, body.action)
  ctx.status = 200
}

async function initDB() {
  await pgClient
.connect()
 
  const res = await pgClient
.query('SELECT $1::text as message', ['Database connected successfully!'])
  console.log(res.rows[0].message) // Hello world!

  pushSubscriptions = {}
  maxSubscriptionId = 0

  const subscriptions = await pgClient
.query('SELECT * from PUSH_SUBSCRIPTION')
  for(const row of subscriptions.rows) {
    let id = row.id
    let subscription = row.subscription

    maxSubscriptionId = Math.max(maxSubscriptionId, id)
    pushSubscriptions[id] = subscription
  }
}

async function saveToDB() {
  await pgClient
.query('DELETE FROM PUSH_SUBSCRIPTION')

  for(const id in pushSubscriptions) {
    const subscription = pushSubscriptions[id]
    await pgClient.query(PUSH_SUBSCRIPTION_INSERT_STATEMENT, [id, JSON.stringify(subscription)])
  }
}

// mqtt handling
mqttClient.on('connect', () => {
  console.log('mqtt connected')
  mqttClient.subscribe(TOPIC_BELL_RINGING, function (err) {
    if (!err) {
      console.log('mqtt subscribed to doorbell topic')
    }
  })
})
mqttClient.on('error', err => console.error('mqtt error', err))
mqttClient.on('message', async (topic, message) => {
  console.log('mqtt message received (topic, msg):', topic, message.toString())
  if(topic === TOPIC_BELL_RINGING && !!message) {
    const data = {
      source: 'mqtt'
    }
    await pgClient.query(DOORBELL_INSERT_STATEMENT, [JSON.stringify(data)])
    await sendNotificationToAllSubcriptions('Doorbell is ringing!')
  }
})

const app = new Koa();
app
  .use(bodyParser())
  .use(cors())
  .use(router.routes())
  .use(router.allowedMethods());

app.listen(process.env.PORT);