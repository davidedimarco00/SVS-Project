import paho.mqtt.client as paho
from paho import mqtt

class MQTTclient:
    def __init__(self, broker, port, username, password, client_id="", qos=1):
        self.qos = qos
        self.client = paho.Client(client_id=client_id, userdata=None, protocol=paho.MQTTv5)
        self.client.on_connect = self.on_connect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(username, password)
        self.client.connect(broker, port)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc, properties=None):
        #print(f"Connected with result code {rc}")
        pass

    def on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        print(f"Subscribed: {mid}, QoS: {granted_qos}")

    def on_message(self, client, userdata, msg):
        print(f"Message received on {msg.topic}: {msg.payload.decode()}")

    def on_publish(self, client, userdata, mid, properties=None):
        #print(f"Message published with mid {mid}")
        pass

    def subscribe(self, topic):
        self.client.subscribe(topic, qos=self.qos)

    def publish(self, topic, payload):
        self.client.publish(topic, payload=payload, qos=self.qos)

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()