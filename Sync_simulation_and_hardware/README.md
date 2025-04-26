#MoveIt via RViz - Hardware 

    #Client side configurations:

            Configure the `client_command_mode to position in lbr_system_config.yaml

Set the   ```bash
update_rate to 100 in lbr_controllers.yaml

Remote side configurations:

        Select

                FRI send period: 10 ms

                IP address: your configuration

                FRI control mode: POSITION_CONTROL or JOINT_IMPEDANCE_CONTROL

                FRI client command mode: POSITION

Proceed with steps 1 and 2 from MoveIt via RViz - Simulation but with ros2 launch lbr_bringup hardware.launch.py in step 1.
