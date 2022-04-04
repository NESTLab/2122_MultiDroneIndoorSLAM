#! /usr/bin/env python3
import sys
import argparse
import signal
from typing import Dict
import rospy
from coms.lite import Lite_Simulator
from subprocess import check_output
from coms.utils import get_ip_list
from coms.constants import ENCODING, STATIC_LISTENER_PORT
import trio

CATKIN_WS = "/root/catkin_ws"


# Handle command line arguments
parser = argparse.ArgumentParser(
    description='Start network communications in ROS.')
parser.add_argument('-ip',
                    type=str,
                    help='unique static IP for ROS node')
parser.add_argument('-env',
                    type=str,
                    choices=['pi', 'sim'],
                    help='deployment environment [ pi | sim ]')
parser.add_argument('-name',
                    type=str,
                    help='unique ROS node name')
parser.add_argument('-launch',
                    type=str,
                    help='path to net-sim launch file')
parser.add_argument('-config',
                    type=str,
                    help='path to net-sim config file')
parser.add_argument('-role',
                    type=str,
                    help='role of the robot [ relay | explorer ]')


def get_run_args() -> Dict:
    NODE_NAMESPACE = rospy.get_namespace()
    NODE_NAME = rospy.get_name()
    LAUNCH_FILE = None

    args, unknown = parser.parse_known_args()
    if len(unknown) > 0:
        # Fetch arguments from launch file
        ip = rospy.search_param('ip')
        env = rospy.search_param('environment')
        config = rospy.search_param('config')
        launch = rospy.search_param('launch')
        role = rospy.search_param('role')
        NODE_IP = rospy.get_param(ip, "")
        NODE_ENVIRONMENT = rospy.get_param(env, "")
        CONFIG_FILE = rospy.get_param(config, "")
        LAUNCH_FILE = rospy.get_param(launch, None)
        ROLE = rospy.get_param(role, "")
    else:
        NODE_IP = args.ip
        NODE_ENVIRONMENT = args.env
        NODE_NAME = args.name
        CONFIG_FILE = args.config
        ROLE = args.role
        if args.launch:
            LAUNCH_FILE = args.launch

    print(NODE_NAME)

    # Validate all runtime arguments
    if (NODE_NAMESPACE == "" or NODE_NAME == "" or CONFIG_FILE == "" or NODE_ENVIRONMENT == "" or NODE_IP == ""
            or (NODE_ENVIRONMENT != "sim" and NODE_ENVIRONMENT != "pi")
            or (ROLE != 'explorer' and ROLE != 'relay')):
        print("Invalid command line or launch arguments.", file=sys.stderr)
        parser.print_usage()
        sys.exit(1)
    return {
        'NODE_NAMESPACE': NODE_NAMESPACE,
        'NODE_NAME': NODE_NAME,
        'NODE_IP': NODE_IP,
        'ROLE': ROLE,
        'NODE_ENVIRONMENT': NODE_ENVIRONMENT,
        'CONFIG_FILE': CONFIG_FILE,
        'LAUNCH_FILE': LAUNCH_FILE
    }


def print_run_args(args: Dict) -> None:
    print("""===========================\nComs Package | RUNTIME ARGS
___________________________
NODE_NAMESPACE: {0}
NODE_NAME: {1}
NODE_IP: {2}
ROLE: {3}
NODE_ENVIRONMENT: {4}
LAUNCH_FILE: {5}
CONFIG_FILE: {6}\n===========================\n""".format(
        args["NODE_NAMESPACE"], args["NODE_NAME"], args["NODE_IP"], args["ROLE"], args["NODE_ENVIRONMENT"], args['LAUNCH_FILE'], args["CONFIG_FILE"]))


def main() -> None:
    # Initialize ROS node
    rospy.init_node(name="coms", anonymous=True)
    # Handle runtime arguments
    args: Dict = get_run_args()
    print_run_args(args)
    # Set fixed update-rate in Hz
    rospy.Rate(float(rospy.get_param('~rate', '2.0')))
    # TODO: Unblock PI environment (when supported)
    if args["NODE_ENVIRONMENT"] != "sim":
        print("\nEnvironment {} => NOT YET SUPPORTED\n\n".format(args["NODE_ENVIRONMENT"]))
        resp = input("Switch to SIM? (y/n) : ")
        if resp != 'y':
            print("Exiting...")
            sys.exit(0)
        args["NODE_ENVIRONMENT"] = "sim"
        print_run_args(args)

    path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, args["CONFIG_FILE"]), shell=True)
    local_ips = []
    for ip in get_ip_list(path_to_config.decode(ENCODING).strip()):
        if ip != args["NODE_IP"]:
            local_ips.append(ip)


    # Run simulation environment
    simulation = Lite_Simulator(
        local_address=(args["NODE_IP"], STATIC_LISTENER_PORT),
        neighbor_ips=local_ips,
        namespace=args["NODE_NAMESPACE"],
        role=args["ROLE"]
    )

    def exit_handler(signal_received: signal.Signals, frame: any) -> None:
        print("\nExiting from signal interupt")
        sys.exit(0)

    signal.signal(signal.SIGINT, exit_handler)
    signal.signal(signal.SIGTSTP, exit_handler)
    signal.signal(signal.SIGCONT, exit_handler)

    try:
        simulation.run()
    except Exception as e:
        print(e)
        print("Terminating...")


if __name__ == "__main__":
    main()
