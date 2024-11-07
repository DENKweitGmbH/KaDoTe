import datetime as dt
import time

import keyboard
from opcua import Server

# ruff: noqa: T201

# Initialize the server
server = Server()
server.set_endpoint("opc.tcp://localhost:4840/freeopcua/server/")


# Register a namespace
uri = "http://example.org"
idx = server.register_namespace(uri)

# Create a new node
node = server.nodes.objects.add_object(idx, "MyObject")

# Add a variable to the node
acquireImage = node.add_variable(idx, "AquireImage", 0)
imageAcquired = node.add_variable(idx, "ImageAcquired", 1)
testVariable = node.add_variable(idx, "Test", 2)
imageAcquired.set_writable()

# Start the server
server.start()
print(f"Server started at {server.endpoint}")

delay_passed = False
last_update_time = dt.datetime.now()


try:
    while True:
        # Check the user input and execute the corresponding function
        time.sleep(0.1)
        if (dt.datetime.now() - last_update_time).total_seconds() > 5:
            delay_passed = True
            # Send Acquire image once every x seconds
            acquireImage.set_value(1)
            print("Acquire image!")

            last_update_time = dt.datetime.now()
        # Check if client acquired image
        if imageAcquired.get_value() == 1:
            acquireImage.set_value(0)
            imageAcquired.set_value(0)
            print("ImageAcquired: " + str(imageAcquired.get_value()))

        if keyboard.is_pressed("i") and acquireImage.get_value() == 0 and delay_passed:
            print(time.time())
            last_update_time = dt.datetime.now()
            delay_passed = False
            value = acquireImage.get_value()
            print("Acquire image:" + str(value))
            print("Setting AquireImage to 1")
            acquireImage.set_value(1)
            value = acquireImage.get_value()
            print("Acquire image:" + str(value))

        elif keyboard.is_pressed("q"):
            print("Exiting program...")
            break
finally:
    # Stop the server
    server.stop()
