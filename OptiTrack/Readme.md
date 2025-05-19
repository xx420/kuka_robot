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
