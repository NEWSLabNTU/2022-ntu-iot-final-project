import argparse
import logging
from .loop import game_loop


def main():
    argparser = argparse.ArgumentParser(description="CARLA Manual Control Client")
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )
    argparser.add_argument(
        "--host",
        metavar="H",
        default="127.0.0.1",
        help="IP of the host server (default: 127.0.0.1)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    # argparser.add_argument(
    #     "-a", "--autopilot", action="store_true", help="enable autopilot"
    # )
    argparser.add_argument(
        "--res",
        metavar="WIDTHxHEIGHT",
        default="1280x768",
        help="window resolution (default: 1280x720)",
    )
    # argparser.add_argument(
    #     "--actor-filter",
    #     metavar="PATTERN",
    #     default="vehicle.tesla.model3",
    #     help='actor filter (default: "vehicle.*")',
    # )
    # argparser.add_argument(
    #     "--actor-generation",
    #     metavar="G",
    #     default="2",
    #     help='restrict to certain actor generation (values: "1","2","All" - default: "2")',
    # )
    # argparser.add_argument(
    #     "--rolename",
    #     metavar="NAME",
    #     default="hero",
    #     help='actor role name (default: "hero")',
    # )
    # argparser.add_argument(
    #     "--gamma",
    #     default=2.2,
    #     type=float,
    #     help="Gamma correction of the camera (default: 2.2)",
    # )
    # argparser.add_argument(
    #     "--sync", action="store_true", help="Activate synchronous mode execution"
    # )
    args = argparser.parse_args()
    args.actor_filter = "vehicle.tesla.model3"
    args.actor_generation = "2"
    args.role_name = "hero"
    args.gamma = 2.2
    args.autopilot = True
    args.sync = True
    args.width, args.height = [int(x) for x in args.res.split("x")]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("listening to server %s:%s", args.host, args.port)

    print(__doc__)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
