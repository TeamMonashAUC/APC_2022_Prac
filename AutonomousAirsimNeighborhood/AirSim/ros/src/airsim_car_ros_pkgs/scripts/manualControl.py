#!/usr/bin/env python3

import airsim

client = airsim.CarClient()
client.confirmConnection()
client.enableApiControl(False)