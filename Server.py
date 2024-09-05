from opcua import Server
import keyboard
import time

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
print("Server started at {}".format(server.endpoint))

delay_passed = False
time_since_last_button_press = 0
time_since_last_update = 0

try:
    while True:
        # Check the user input and execute the corresponding function

        if time.time() - time_since_last_button_press > 1:
            delay_passed = True

        if time.time() - time_since_last_update > 1:
            if imageAcquired.get_value() == 1:
                acquireImage.set_value(0)
                imageAcquired.set_value(0)
            print("Acquire image: " + str(acquireImage.get_value()))
            print("ImageAcquired: " + str(imageAcquired.get_value()))
            
            time_since_last_update = time.time()

        if keyboard.is_pressed('i') and acquireImage.get_value() == 0 and delay_passed == True:
            print(time.time())
            time_since_last_button_press = time.time()
            delay_passed = False
            value = acquireImage.get_value()
            print ("Acquire image:" + str(value))
            print("Setting AquireImage to 1")
            acquireImage.set_value(1)
            value = acquireImage.get_value()
            print ("Acquire image:" + str(value))

        elif keyboard.is_pressed('q'):
            print("Exiting program...")
            break


finally:
    # Stop the server
    server.stop()