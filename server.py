# Copyright (c) 2024 DENKweit GmbH <denkweit.com> All rights reserved.
"""OPCUA test server implementation."""

import datetime as dt
import json
import logging
import sys
import time
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser

try:
    import keyboard

    has_keyboard = True
except ImportError:
    has_keyboard = False

from opcua import Server
from opcua.ua import StringNodeId

# ruff: noqa: T201, D103


def main(args_: list[str]) -> None:
    log = logging.getLogger()
    stream_h = logging.StreamHandler()
    stream_fmter = logging.Formatter(
        fmt="%(asctime)s.%(msecs)03d %(name)-15s %(levelname)-8s %(message)s",
        style="%",
        datefmt="%H:%M:%S",
    )
    stream_h.setFormatter(stream_fmter)
    log.addHandler(stream_h)
    log.setLevel(logging.DEBUG)
    log.getChild("opcua").setLevel(logging.WARNING)
    parser = ArgumentParser(
        description="OPCUA test server implementation",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Interval in which analysis is requested (in sec).",
    )
    # The keyboard functionality requires root privileges on linux so we disable it by default.
    parser.add_argument(
        "--disable-keyboard",
        action="store_true",
        default=not has_keyboard or sys.platform == "linux",
    )
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
    image_acquired = node.add_variable(idx, "ImageAcquired", 0)
    image_acquired.set_writable()
    results = node.add_variable(idx, "Results", "", datatype=StringNodeId("String", idx))
    results.set_writable()

    # Start the server
    server.start()
    log.info("Server started at %s", server.endpoint)
    last_update_time = dt.datetime.now()

    try:
        while True:
            # Check the user input and execute the corresponding function
            time.sleep(0.1)
            if (
                dt.datetime.now() - last_update_time
            ).total_seconds() > args.interval and acquire_image.get_value() == 0:
                # Send Acquire image once every x seconds
                acquire_image.set_value(1)
                last_update_time = dt.datetime.now()
                log.info("Set 'AcquireImage' to 1")
            # Check if client acquired image
            if image_acquired.get_value() == 1:
                acquire_image.set_value(0)
                image_acquired.set_value(0)
                log.info("Analysis done")
                res = results.get_value()
                log.info("Results are: %s (raw: %r)", json.loads(res), res)

            if args.disable_keyboard:
                continue

            if keyboard.is_pressed("i") and acquire_image.get_value() == 0:
                log.info(time.time())
                last_update_time = dt.datetime.now()
                acquire_image.set_value(1)
                log.info("Set 'AcquireImage' to 1")
            elif keyboard.is_pressed("q"):
                log.info("Exiting program...")
                break
    finally:
        # Stop the server
        server.stop()


if __name__ == "__main__":
    main(sys.argv[1:])
