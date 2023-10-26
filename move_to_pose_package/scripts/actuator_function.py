
#!/usr/bin/env python
# this is the actuator enable disable file bro how we can connect it with our code ....

# this is the function set_actuator_state ... we will copy it to our code....
# i have created another file intentionally so that we can just test it
# we can also import it to our code as well... it is not a problem
# ok please make it till the end...! 

 #could we do it tomorrow bro/ my sister is waitng for me... /ok till tomorrow 

import rospy
from open_manipulator_msgs.srv import SetActuatorState, SetActuatorStateRequest

def set_actuator_state(enable):
    rospy.init_node('simple_actuator_controller', anonymous=True)

    # Wait for the service to be available
    rospy.wait_for_service('/set_actuator_state')

    try:
        # Create a service proxy
        set_state = rospy.ServiceProxy('/set_actuator_state', SetActuatorState)

        # Create a request message
        request = SetActuatorStateRequest()
        request.set_actuator_state = enable

        # Call the service
        response = set_state(request)

        # Return the response of the service call
        return response.is_planned
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    # Example: Enable or disable the actuator based on the requirement
    # This is a blocking call; it will wait until the service has been executed
    result = set_actuator_state(False)  # Or `set_actuator_state(False)` to disable
    print("Service call was successful" if result else "Service call failed")
