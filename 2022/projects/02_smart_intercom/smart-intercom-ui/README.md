# SmartIntercomUi

## Local development

Run `ng serve` for a dev server. Navigate to `http://localhost:4200/`. The app will automatically reload if you change any of the source files.

## Push Registration

The push registration uses a VAPID Key-Pair. It can be generated like this:

```
$ npm install -g web-push
$ web-push generate-vapid-keys
```

The private key should never be shared or checked into git.
