db.createCollection('users');
db.users.insertOne(
  {
    name: 'admin',
    username: 'admin',
    password: '$2b$12$1sJtp2wm7I92oFAM4bQRLed8wsfQLXWI1MU5fJfb1hdpTVbiCjxIq'
  }
);
db.createCollection('qrs');