const router = require('koa-router')();
const Koa = require('koa');
const bodyParser = require('koa-bodyparser');
const cors = require('@koa/cors');

let maxSubscriptionId = 0
let pushSubscriptions = {}

// database
const { Client } = require('pg')
const client = new Client({
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

router.get('/ping', ping)
  .post('/pushsubscription', savePushSubscription)
  .post('/notify', sendNotification)

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

  dataToSend = body.message

  for (const [id, subscription] of Object.entries(pushSubscriptions)) {
    try {
      const success = await webpush.sendNotification(subscription, dataToSend)
      console.log('sent notification with status:', success.statusCode)
    } catch(err) {
      console.log('Subscription has expired or is no longer valid: ', err)
      delete pushSubscriptions[id]
    }
  }

  ctx.status = 204

  await saveToDB()
}

async function initDB() {
  await client.connect()
 
  const res = await client.query('SELECT $1::text as message', ['Database connected successfully!'])
  console.log(res.rows[0].message) // Hello world!

  pushSubscriptions = {}
  maxSubscriptionId = 0

  const subscriptions = await client.query('SELECT * from PUSH_SUBSCRIPTION')
  for(const row of subscriptions.rows) {
    let id = row.id
    let subscription = row.subscription

    maxSubscriptionId = Math.max(maxSubscriptionId, id)
    pushSubscriptions[id] = subscription
  }
}

async function saveToDB() {
  await client.query('DELETE FROM PUSH_SUBSCRIPTION')

  const insertStatement = 'INSERT INTO PUSH_SUBSCRIPTION(id, subscription) VALUES($1, $2)'
  for(const id in pushSubscriptions) {
    const subscription = pushSubscriptions[id]
    await client.query(insertStatement, [id, JSON.stringify(subscription)])
  }
}

const app = new Koa();
app
  .use(bodyParser())
  .use(cors())
  .use(router.routes())
  .use(router.allowedMethods());

app.listen(process.env.PORT);