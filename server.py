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
from opcua.ua import DataValue, Variant, VariantType

# ruff: noqa: T201, D103


def main(args_: list[str]) -> None:  # noqa: PLR0915
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
    ns = server.register_namespace(uri)
    # Create a new node
    node = server.nodes.objects.add_object(ns, "CameraControl")
    # Add a variable to the node
    capture_image_1 = node.add_variable(ns, "CaptureImage1", val=False)
    capture_image_1.set_writable()
    capture_image_2 = node.add_variable(ns, "CaptureImage2", val=False)
    capture_image_2.set_writable()
    # TODO: Should be array of strings
    results = node.add_variable(ns, "Results", ["{}"])
    results.set_writable()

    # Start the server
    server.start()
    log.info("Server started at %s", server.endpoint)
    last_update_time = dt.datetime.now()
    next_stage = 1
    key_pressed = False

    try:
        while True:
            # Check the user input and execute the corresponding function
            time.sleep(0.1)
            if (
                (
                    (dt.datetime.now() - last_update_time).total_seconds() > args.interval
                    or key_pressed
                )
                and not capture_image_1.get_value()
                and not capture_image_2.get_value()
            ):
                # Send capture image once every x seconds
                if next_stage == 1:
                    capture_image_1.set_value(True)
                    log.info("Set 'CaptureImage1'")
                elif next_stage == 2:  # noqa: PLR2004
                    capture_image_2.set_value(True)
                    log.info("Set 'CaptureImage2'")
                next_stage += 1
                if next_stage > 2:  # noqa: PLR2004
                    next_stage = 1
                last_update_time = dt.datetime.now()
            key_pressed = False
            # Check if client acquired image
            if next_stage == 1 and (res := results.get_value()):
                log.info("Analysis done")
                log.info("Results are: %s (raw: %r)", [json.loads(obj) for obj in res], res)
                results.set_value(DataValue(Variant([], VariantType.String)))
            if args.disable_keyboard:
                continue

            if keyboard.is_pressed("i"):
                key_pressed = True
            elif keyboard.is_pressed("q"):
                log.info("Exiting program...")
                break
    finally:
        # Stop the server
        server.stop()


if __name__ == "__main__":
    main(sys.argv[1:])
