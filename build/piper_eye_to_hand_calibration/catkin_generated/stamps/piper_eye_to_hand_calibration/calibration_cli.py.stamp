#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger


def call(service):
    try:
        response = service()
        print(("[OK] " if response.success else "[REJECT] ") + response.message)
        return response.success, response.message
    except rospy.ServiceException as exception:
        print("[ERROR] {}".format(exception))
        return False, str(exception)


def main():
    rospy.init_node("calibration_cli", anonymous=True, disable_signals=True)
    camera = rospy.get_param("~camera", "rs1").strip().strip("/")
    if camera != "rs1":
        raise ValueError("this workflow calibrates rs1 handeye only")
    prefix = "/{}/eye_to_hand".format(camera)
    names = {
        "capture": prefix + "/capture",
        "delete": prefix + "/delete_last",
        "reset": prefix + "/reset",
        "solve": prefix + "/solve",
    }
    print("Waiting for {} calibration services...".format(camera))
    for name in names.values():
        rospy.wait_for_service(name)
    services = {key: rospy.ServiceProxy(name, Trigger) for key, name in names.items()}

    print("{} ready: Enter=capture, d=delete, r=reset, q=solve, c=exit".format(camera))
    while not rospy.is_shutdown():
        try:
            command = input("[{}] > ".format(camera)).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("")
            break
        if command == "":
            call(services["capture"])
        elif command == "d":
            call(services["delete"])
        elif command == "r":
            call(services["reset"])
        elif command == "q":
            passed, _ = call(services["solve"])
            if passed:
                break
        elif command == "c":
            break
        else:
            print("Invalid command. Use Enter, d, r, q, or c.")


if __name__ == "__main__":
    main()
