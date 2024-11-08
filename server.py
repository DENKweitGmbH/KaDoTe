# Copyright (c) 2024 DENKweit GmbH <denkweit.com> All rights reserved.
"""Test OPCUA server implementation."""

import datetime as dt
import sys
import time
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

import keyboard
from opcua import Server

# ruff: noqa: T201, D103


def main(args_: list[str]) -> None:
    parser = ArgumentParser(
        description="Test OPCUA server implementation",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--interval", type=float, default=1.0, help="Interval in which analysis is requested."
    )
    # The keyboard functionality requires root privileges on linux so we disable it by default.
    parser.add_argument("--disable-keyboard", action="store_true", default=sys.platform == "linux")
    args = parser.parse_args(args_)

    # Initialize the server
    server = Server()
    server.set_endpoint("opc.tcp://localhost:4840/freeopcua/server/")
    # Register a namespace
    uri = "http://denkweit.de"
    idx = server.register_namespace(uri)
    # Create a new node
    node = server.nodes.objects.add_object(idx, "MyObject")
    # Add a variable to the node
    acquire_image = node.add_variable(idx, "AquireImage", 0)
    image_acquired = node.add_variable(idx, "ImageAcquired", 1)
    image_acquired.set_writable()

    # Start the server
    server.start()
    print(f"Server started at {server.endpoint}")
    last_update_time = dt.datetime.now()

    try:
        while True:
            # Check the user input and execute the corresponding function
            time.sleep(0.1)
            if (dt.datetime.now() - last_update_time).total_seconds() > args.interval:
                # Send Acquire image once every x seconds
                acquire_image.set_value(1)
                print("Acquire image!")

                last_update_time = dt.datetime.now()
            # Check if client acquired image
            if image_acquired.get_value() == 1:
                acquire_image.set_value(0)
                image_acquired.set_value(0)
                print("ImageAcquired: " + str(image_acquired.get_value()))

            if args.disable_keyboard:
                continue

            if keyboard.is_pressed("i") and acquire_image.get_value() == 0:
                print(time.time())
                last_update_time = dt.datetime.now()
                value = acquire_image.get_value()
                print("Acquire image:" + str(value))
                print("Setting AquireImage to 1")
                acquire_image.set_value(1)
                value = acquire_image.get_value()
                print("Acquire image:" + str(value))
            elif keyboard.is_pressed("q"):
                print("Exiting program...")
                break
    finally:
        # Stop the server
        server.stop()


if __name__ == "__main__":
    main(sys.argv[1:])
