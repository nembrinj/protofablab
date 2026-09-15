#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>


// ----------------------------------------------------
// Wi-Fi / micro-ROS Agent configuration
// ----------------------------------------------------

char wifi_ssid[]     = "ProFab";
char wifi_password[] = "1700_UniFR.&";

// IP address of the COMPUTER running micro_ros_agent
char agent_ip[] = "192.168.1.2";
const size_t ROS_DOMAIN_ID = 101;

uint16_t agent_port = 8888;


// ----------------------------------------------------
// ROS objects
// ----------------------------------------------------

rcl_node_t node;
rcl_publisher_t publisher;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;

std_msgs__msg__Int32 msg;


void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Configure micro-ROS WiFi transport.
  set_microros_wifi_transports(
    wifi_ssid,
    wifi_password,
    agent_ip,
    agent_port
  );

  delay(2000);

  allocator = rcl_get_default_allocator();

  // Initialize ROS init options.
  init_options = rcl_get_zero_initialized_init_options();

  rcl_init_options_init(
    &init_options,
    allocator
  );


  // Explicitly set ROS_DOMAIN_ID.
  rcl_init_options_set_domain_id(
    &init_options,
    ROS_DOMAIN_ID
  );


  // Initialize micro-ROS support using those options.
  rclc_support_init_with_options(
    &support,
    0,
    NULL,
    &init_options,
    &allocator
  );

  // Create node.
  rclc_node_init_default(
    &node,
    "esp32_node",
    "",
    &support
  );


  // Create publisher.
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/esp32/counter"
  );


  msg.data = 0;

  Serial.println("micro-ROS initialized");
  Serial.print("ROS domain ID: ");
  Serial.println(ROS_DOMAIN_ID);

  Serial.print("micro-ROS Agent IP: ");
  Serial.println(agent_ip);

  Serial.print("micro-ROS Agent port: ");
  Serial.println(agent_port);
}


// ----------------------------------------------------
// Main loop
// ----------------------------------------------------

void loop()
{
    msg.data++;

    rcl_publish(
        &publisher,
        &msg,
        NULL
    );

    Serial.print("Published: ");
    Serial.println(msg.data);

    delay(1000);
}