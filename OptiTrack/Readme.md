# OptiTrack V120:Trio Network Setup with NatNet SDK (Windows + Ubuntu)

This document describes how we connected the OptiTrack V120:Trio motion capture system over the same network on Windows and Ubuntu machines using the NatNet SDK.

## Overview
- The OptiTrack V120:Trio is connected to a Windows PC running **Motive** software.
- Both Windows and Ubuntu machines are on the **same local network**.
- We use **NatNet SDK** on Ubuntu to receive motion capture data streamed from Motive on Windows.

## Setup Steps

### On Windows (Motive)
1. Connect the V120:Trio device to the Windows PC via USB.
2. Launch **Motive** and ensure the camera system is tracking markers correctly.
3. Enable **NatNet streaming**:
   - Go to **Streaming Settings** in Motive.
   - Enable **Multicast** or **Unicast** streaming.
   - Note the Windows PC’s IP address (the streaming server IP).

### On Ubuntu (NatNet SDK Client)
1. Install required dependencies and NatNet SDK for Linux.
2. Clone or download the NatNet SDK repository:
   ```bash
   git clone https://github.com/OptiTrack/NatNetSDK.git
   ```
3. Build the NatNet client example or use Python wrapper to receive data.

4. Configure the client to connect to the Windows PC’s IP address (server IP).

5. Run the NatNet client to receive real-time motion capture data streamed from Motive.

Notes

    1. Both machines must be on the same subnet.

    2. Firewalls should allow UDP traffic on ports used by NatNet (typically 1510, 1511).
               **Disabled Firewall**
         ```
            sudo ufw disable
         ```

    3. The NatNet SDK supports multiple languages including C++, C#, and Python.

# OptiTrack V120:Trio NatNet ROS2 Setup

## ROS2 Node Configuration (`optitrack_wrapper_node`)

Save this configuration in your ROS2 config YAML file (e.g., `config/optitrack_wrapper.yaml`):

```yaml
optitrack_wrapper_node:
  ros__parameters:
    # NatNet server address (Windows PC running Motive)
    server_address: "172.31.1.146"

    # Local IP address (Ubuntu machine running ROS2 node)
    local_address: "172.31.1.149"

    # Connection type: "Multicast" or "Unicast"
    connection_type: "Multicast"

    # Multicast group address (default)
    multicast_address: "239.255.42.99"

    # NatNet server command port (default: 1510)
    server_command_port: 1510

    # NatNet server data port (default: 1511)
    server_data_port: 1511

    # Enable verbose output for data description (optional)
    verbose_data_description: false

    # Enable verbose frame output (optional)
    verbose_frame: false

    # ROS topic name to publish the frame data
    topic_frame_data: "frame_data"
```

#Running the OptiTrack NatNet ROS2 Node

1. Make sure your ROS2 workspace environment is sourced:
   
   ```source install/setup.bash```

2. Run the launch file from the optritrack_ws directory with your NatNet server IP address:
```ros2 launch optitrack_wrapper_ros2 optitrack_wrapper.launch.py server_address:=172.31.1.146```


## Network Setup Notes

- Both Ubuntu and Windows machines should be connected to the same local network and subnet.
- **Windows (NatNet server) IP:** `172.31.1.146`
- **Ubuntu (ROS2 client) IP:** `172.31.1.149`
- Ensure **Windows Firewall allows UDP traffic** on ports **1510** and **1511**.
- Set **Motive streaming to "Multicast"** to match the ROS2 node configuration.
- **Verify connectivity** by pinging between the Ubuntu and Windows machines.

## NatNet Diagram 

![NatNet Image](../images/pic1.png)

## References

• [Motive Software](https://optitrack.com/products/motive/)  
• [optitrack_packages_ros2 GitHub Repository](https://github.com/lis-epfl/optitrack_packages_ros2)

