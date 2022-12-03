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


router.get('/ping', ping)
  .post('/pushsubscription', savePushSubscription)

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
}

async function initDB() {
  await client.connect()
 
  const res = await client.query('SELECT $1::text as message', ['Database connected successfully!'])
  console.log(res.rows[0].message) // Hello world!

  pushSubscriptions = {}
  maxSubscriptionId = 0

  const subscriptions = await client.query('SELECT * from PUSH_SUBSCRIPTION')
  for(const row of subscriptions.rows) {
    let id = row[0]
    let subscription = JSON.parse(row[1])

    maxSubscriptionId = Math.max(maxSubscriptionId, id)
    pushSubscriptions.push(subscription)
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