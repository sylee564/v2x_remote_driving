cmake_minimum_required(VERSION 2.8.3)
project(tcpip_msgs)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation)

# Add messages.
add_message_files(
  FILES
  endpoint.msg
  status.msg
  tcp_client.msg
  tcp_packet.msg
  tcp_server.msg
  tcp_socket.msg
  udp_packet.msg
  udp_socket.msg
  LinkMetrics.msg)

# Add services.
add_service_files(
  FILES
  close_socket.srv
  open_udp_socket.srv
  resolve_ip.srv
  send_tcp.srv
  start_tcp_client.srv
  start_tcp_server.srv
  stop_tcp_client.srv
  stop_tcp_server.srv)

# Generate messages and services.
generate_messages()

# Create catkin package.
catkin_package(CATKIN_DEPENDS message_runtime)