

# Using MQTT with a certificate


We need only one broker with the certificate configuration. Then all clients only need the CA certificate in order to connect. See the client example using pahomqtt below


## howto

http://www.steves-internet-guide.com/mosquitto-tls/

## example of mqtt configuration


following https://openest.io/en/2020/01/03/mqtts-how-to-use-mqtt-with-tls/ and 
from https://github.com/ustccw/MQTT-TLS/blob/master/mosquitto.conf

1. generate broker certificate, key and CA certificate using the howto above
2. configure the broker as follows 

```
# mosquitto.conf
#pid_file /var/run/mosquitto/mosquitto.pid

persistence false
persistence_location /mosquitto/data/

log_dest file /mosquitto/log/mosquitto.log
log_dest stdout

allow_anonymous false

listener 8883

# -----------------------------------------------------------------
# Certificate based SSL/TLS support
# -----------------------------------------------------------------
# The following options can be used to enable SSL/TLS support for 
# this listener. Note that the recommended port for MQTT over TLS
# is 8883, but this must be set manually.
#
# See also the mosquitto-tls man page.

# At least one of cafile or capath must be defined. They both 
# define methods of accessing the PEM encoded Certificate 
# Authority certificates that have signed your server certificate 
# and that you wish to trust.
# cafile defines the path to a file containing the CA certificates.
# capath defines a directory that will be searched for files
# containing the CA certificates. For capath to work correctly, the
# certificate files must have ".crt" as the file ending and you must run
# "c_rehash <path to capath>" each time you add/remove a certificate.

cafile /mosquitto/certs/ca.crt
#capath /home/openest/certs/ca

# Path to the PEM encoded server certificate.
certfile /mosquitto/certs/broker.crt

# Path to the PEM encoded keyfile.
keyfile /mosquitto/certs/broker.key

# This option defines the version of the TLS protocol to use for this listener.
# The default value allows v1.2, v1.1 and v1.0, if they are all supported by
# the version of openssl that the broker was compiled against. For openssl >=
# 1.0.1 the valid values are tlsv1.2 tlsv1.1 and tlsv1. For openssl < 1.0.1 the
# valid values are tlsv1.
#tls_version

# By default a TLS enabled listener will operate in a similar fashion to a
# https enabled web server, in that the server has a certificate signed by a CA
# and the client will verify that it is a trusted certificate. The overall aim
# is encryption of the network traffic. By setting require_certificate to true,
# the client must provide a valid certificate in order for the network
# connection to proceed. This allows access to the broker to be controlled
# outside of the mechanisms provided by MQTT.
#require_certificate true

# If require_certificate is true, you may set use_identity_as_username to true
# to use the CN value from the client certificate as a username. If this is
# true, the password_file option will not be used for this listener.
#use_identity_as_username true

# If you have require_certificate set to true, you can create a certificate
# revocation list file to revoke access to particular client certificates. If
# you have done this, use crlfile to point to the PEM encoded revocation file.
#crlfile

# If you wish to control which encryption ciphers are used, use the ciphers
# option. The list of available ciphers can be optained using the "openssl
# ciphers" command and should be provided in the same format as the output of
# that command.
# If unset defaults to DEFAULT:!aNULL:!eNULL:!LOW:!EXPORT:!SSLv2:@STRENGTH
#ciphers DEFAULT:!aNULL:!eNULL:!LOW:!EXPORT:!SSLv2:@STRENGTH
```




## example of client paho usage with a certificate


assuming the certificate is stored as ./certs/ca.crt


    import paho.mqtt.client as mqtt 
    import paho.mqtt.publish as publish


    ...
    
    remote_port = 8883
    broker_address = 'the_broker_ip'
    
    ...


    client = mqtt.Client("myclientID")
     
    client.tls_set('./certs/ca.crt',tls_version=2)   
    
    #bind call back functions
    client.on_connect = on_connect
    client.on_message = on_message

    # connect to server
    try:
        client.connect(broker_address, port=remote_port) #connect to broker
        client.loop_forever()
    except:
        print ("Connection Failed")

