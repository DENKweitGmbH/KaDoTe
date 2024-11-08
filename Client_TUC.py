# Copyright (c) 2024 DENKweit GmbH <denkweit.com> All rights reserved.
from opcua import Client
from opcua.ua import NodeId
from opcua.crypto import uacrypto
import time
from datetime import datetime

import ids_peak.ids_peak as ids_peak
import ids_peak_ipl.ids_peak_ipl as ids_ipl
import numpy as np
from matplotlib import pyplot as plt


# Function to execute when myvariable is set to "1"
def acquire_ids_image():
    print("Acquiring image.")
    remote_device_nodemap.FindNode("AcquisitionStart").Execute()
    remote_device_nodemap.FindNode("AcquisitionStart").WaitUntilDone()
    
    # Enable Auto Exposure
    #exposure_auto_node = remote_device_nodemap.FindNode("ExposureAuto")
    #exposure_auto_node.SetValue("Continuous")
    
    remote_device_nodemap.FindNode("ExposureTime").SetValue(5000) # in microseconds

    # trigger image
    remote_device_nodemap.FindNode("TriggerSoftware").Execute()
    buffer = datastream.WaitForFinishedBuffer(1000)

    # convert to RGB
    raw_image = ids_ipl.Image.CreateFromSizeAndBuffer(buffer.PixelFormat(), buffer.BasePtr(), buffer.Size(), buffer.Width(), buffer.Height())
    color_image = raw_image.ConvertTo(ids_ipl.PixelFormatName_RGB8)
    datastream.QueueBuffer(buffer)

    picture = color_image.get_numpy_3D()

    plt.figure(figsize = (15,15))
    plt.imshow(picture)
    current_time = datetime.now()
    plt.savefig(current_time.strftime("%Y-%m-%d_%H-%M-%S"))



###Initialize IDS camera###########################
ids_peak.Library.Initialize()
device_manager = ids_peak.DeviceManager.Instance()
device_manager.Update()
device_descriptors = device_manager.Devices()

print("Found Devices: " + str(len(device_descriptors)))
for device_descriptor in device_descriptors:
    print(device_descriptor.DisplayName())


try:
    device = device_descriptors[0].OpenDevice(ids_peak.DeviceAccessType_Exclusive)
    print("Opened Device: " + device.DisplayName())
    remote_device_nodemap = device.RemoteDevice().NodeMaps()[0]

    remote_device_nodemap.FindNode("TriggerSelector").SetCurrentEntry("ExposureStart")
    remote_device_nodemap.FindNode("TriggerSource").SetCurrentEntry("Software")
    remote_device_nodemap.FindNode("TriggerMode").SetCurrentEntry("On")

    datastream = device.DataStreams()[0].OpenDataStream()
    payload_size = remote_device_nodemap.FindNode("PayloadSize").Value()
    for i in range(datastream.NumBuffersAnnouncedMinRequired()):
        buffer = datastream.AllocAndAnnounceBuffer(payload_size)
        datastream.QueueBuffer(buffer)
    datastream.StartAcquisition()
    camera_detected = True
except Exception as e:
    print("No camera detected.")
    camera_detected = False
###################################################


# connect to OPCUA  server#########################
# OPC UA server endpoint URL

client = Client("opc.tcp://134.109.8.245:4840") #OpcUaClient:PASSWORT@134.109.8.238:4840")
client.set_user("OpcUaClient")
client.set_password("PASSWORT")
# client.set_security(ua.SecurityPolicyType.Basic256Sha256_Sign, ua.MessageSecurityMode.SignAndEncrypt)
client.connect()


root = client.get_root_node()
print("Root is", root)
print("childs of root are: ", root.get_children())

objects = client.get_objects_node()
print("Objects node is: ", objects)

# Recursively browse nodes
#def browse_nodes(node, level=0):
    # Print the current node
#    print("  " * level + str(node))
    # Browse child nodes
#   for child in node.get_children():
#        browse_nodes(child, level + 1)

# Start browsing from the objects node
#browse_nodes(objects)

###################################################

try:
    while True:
        # Read the value of the variable
        acquireImage = client.get_node("ns = 2; s=/Channel/Parameter/R[1]").get_value()
        imageAcquired = client.get_node("ns = 2; s=/Channel/Parameter/R[2]").get_value()
        print("Nodes acquireImage:" + str(acquireImage))
        print("Nodes imageAcquired:" + str(imageAcquired))

        # Check if the variable value is "1"
        if acquireImage == 1 and imageAcquired == 0:
            # Execute the function
            if camera_detected == True:
                acquire_ids_image()
                node = client.get_node("ns = 2; s=/Channel/Parameter/R[1]")
                node.set_value(0.0)
            else:
                print("Cannot acquire image. No camera connected.")

        # Wait for 1 second before checking again
        time.sleep(1)

except KeyboardInterrupt:
    print("Script terminated by user.")

finally:
    # Disconnect from the OPC UA server
    client.disconnect()