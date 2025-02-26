import json
import paho.mqtt.client as mqtt
from src.robot_control_comms import MqttTopicNames, ClientRobotControlInputsMsg, SetLedsMsg, get_pose_list_from_json_msg

class DJIRoboMasterEPMqttInterface:
    def __init__(self, robot_id, backend_server_ip=None):
        # constants
        self.MQTT_CLIENT_NAME = f"robot_{robot_id}_client"
        if backend_server_ip is None:
            self.MQTT_BROKER = "localhost"
        else:
            self.MQTT_BROKER = backend_server_ip
        self.MQTT_PORT = 1883
        self.MQTT_KEEPALIVE = 60

        # MQTT topics for controlling robot and getting pose
        mqtt_topic_names = MqttTopicNames(robot_id)
        self.MQTT_CLIENT_ROBOT_CONTROL_INPUTS = mqtt_topic_names.client_robot_inputs
        self.MQTT_DESIRED_LEDS = mqtt_topic_names.desired_leds
        self.MQTT_ROBOT_POSE_TOPIC_NAME = mqtt_topic_names.global_robot_pose

        # initialize variables for object poses
        self.robot_pose = []
        # initialize counters
        # self.last_time_get_poses = int(round(time.time()*1000))

        # connect to the mqtt network
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, self.MQTT_CLIENT_NAME)
        self.mqtt_client.on_connect = lambda client, userdata, flags, rc: self.__mqtt_on_connect(client, userdata, flags, rc)
        self.mqtt_client.on_message = lambda client, userdata, msg: self.__mqtt_on_message(client, userdata, msg)
        self.mqtt_client.connect(self.MQTT_BROKER, port=self.MQTT_PORT, keepalive=self.MQTT_KEEPALIVE)
        self.mqtt_client.loop_start()

    def __mqtt_on_connect(self, client, userdata, flags, rc):
        print("Connected to the MQTT network with result code " + str(rc))
        self.mqtt_client.subscribe(self.MQTT_ROBOT_POSE_TOPIC_NAME)
        print("Subscribed to the MQTT topics:")
        print("\t", self.MQTT_ROBOT_POSE_TOPIC_NAME)

    def __mqtt_on_message(self, client, userdata, msg):
        json_msg = json.loads(msg.payload)
        if msg.topic == self.MQTT_ROBOT_POSE_TOPIC_NAME:
            self.robot_pose = get_pose_list_from_json_msg(json_msg)

    def step(self, inputs):
        self.mqtt_client.publish(self.MQTT_CLIENT_ROBOT_CONTROL_INPUTS, ClientRobotControlInputsMsg(inputs).toJson())

    def set_mobile_base_speed_and_gripper_power(self, x : float, y : float, z : float, gripper_power : float):
        self.step([x, y, z, 0., 0., 0., gripper_power])
    
    def set_arm_pose(self, x, y, wait_s=2.6): # Not recommended to use a smaller wait time than this
        self.step([0., 0., 0., x, y, 1., 0.])
        if wait_s > 0.:
            time.sleep(wait_s)

    def set_leds(self, red_level : int, green_level : int, blue_level : int):
        self.mqtt_client.publish(self.MQTT_DESIRED_LEDS, SetLedsMsg(red_level, green_level, blue_level).toJson())
    
    def get_pose(self):
        return self.robot_pose