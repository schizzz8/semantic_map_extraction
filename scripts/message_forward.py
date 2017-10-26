#!/usr/bin/env python
import roslib
import rospy

from std_msgs.msg import String
from semantic_map_extraction.msg import Obs
from semantic_map_extraction.srv import GetCoordsByName
from move_base_msgs.msg import *
from tcp_interface.msg import RCOMMessage
from functools import partial
import actionlib

last_position = None
last_position_sem = None
last_command = None
confirmation = None
auditing = True

def say(string, voice_pub, robotname):
    global auditing

    auditing = False
    rate = rospy.Rate(5)
    msg = RCOMMessage()
    msg.robotsender = robotname
    msg.robotreceiver = "all"
    msg.value = "say_[" + string + "]"
    voice_pub.publish(msg)
    rate.sleep()
    

def forward(string, robotname=None, voice_pub=None, obs_pub=None, asr_pub=None, update_prop_pub=None, update_pos_pub=None):
    global last_position
    global last_position_sem
    global last_command
    global confirmation
    global auditing

    string = string.value

    if "[END_SYNTH]" in string:
        string.replace("[END_SYNTH]", "")
        auditing = True

    if not auditing:
        return

    string = string.split("\"")
        
    if len(string) < 2:
        return

    string = string[1]

    rate = rospy.Rate(5)
    understood = False
    print "Received: " + string
    
    if "cancel" in string:
        understood = True
        sac = actionlib.SimpleActionClient('/' + robotname + '/move_base', MoveBaseAction)

        sac.wait_for_server()
        sac.cancel_goal()
        sac.cancel_all_goals()
        say("Ok, so what should I do?", voice_pub, robotname)

    if "go" in string:
        print("Command: go")
        understood = True
        name = string.replace("go","")
        rospy.wait_for_service('semantic_map_extraction/get_coords_by_name')
        try:
            request = rospy.ServiceProxy('semantic_map_extraction/get_coords_by_name',
                                         GetCoordsByName)
            position = request(name)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        print position

        if position.real_x == 1000 and position.real_y == 1000:
            say("Sorry I cannot reach " + name + ".", voice_pub, robotname)
            return
        
        sac = actionlib.SimpleActionClient('/' + robotname + '/move_base', MoveBaseAction)
        goal = MoveBaseGoal()
            #set goal
        goal.target_pose.pose.position.x = position.real_x
        goal.target_pose.pose.position.y = position.real_y
        goal.target_pose.pose.orientation.w = 1.0
        goal.target_pose.header.frame_id = '/map'
        goal.target_pose.header.stamp = rospy.Time.now()

        say("I am going to " + name, voice_pub, robotname)

        sac.wait_for_server()
        sac.send_goal(goal)
        
        last_position = position
        last_position_sem = name
         
    if last_position != None and "here" in string:
        understood = True
        name = string.replace("here","")

        if last_position.real_x == 1000 and last_position.real_y == 1000:
            say("Sorry I did not reach " + last_position_sem + ", I do not know where here is.", voice_pub, robotname)
            return
        
        # last_position = None
        
        say("I now know that " + name + " is in " + last_position_sem, voice_pub, robotname)
        say("Is it correct?", voice_pub, robotname)
        confirmation = string

    if confirmation != None and "here" in confirmation and "yes" in string:
        understood = True
        name = confirmation.replace("here","")
        pos = last_position
        msg = name + ":" + str(pos.real_x) + " " + str(pos.real_y)
        print(msg)
        update_pos_pub.publish(msg)
        confirmation = None
        say("Ok, grazie!|it", voice_pub, robotname)
        rate.sleep()

    if last_command != None and "closed" in last_command:
        understood = True
        last_command = None
        name = string

        say("I now know that door in " + name + " is closed", voice_pub, robotname)
        say("Is it correct?", voice_pub, robotname)
        confirmation = name + "doorclosed"

    if confirmation != None and "closed" in confirmation and "yes" in string:
        understood = True
        name = confirmation.replace("closed","")
        msg = name + ":closed"
        print(msg)
        update_prop_pub.publish(msg)
        confirmation = None
        say("Ok, grazie!|it", voice_pub, robotname)
        rate.sleep()

    if confirmation != None and "no" in string:
        understood = True
        say("Ok, I misunderstood! Please, repeat the last command.", voice_pub, robotname)        

    if "closed" in string:
        understood = True
        last_command = "closed"
        say("Which one?", voice_pub, robotname)
        
    if not understood:
        say("Sorry, I did not understand your command.", voice_pub, robotname)

    print("Message forwarded.")


if __name__ == "__main__":
    rospy.init_node('message_forward')
    robotname = rospy.get_param('robot_name', 'robot0')
    obs_pub = rospy.Publisher('/' + robotname + '/ObservationTopic', Obs, queue_size=10)
    asr_pub = rospy.Publisher('/' + robotname + '/ASR', String, queue_size=10)
    update_prop_pub = rospy.Publisher('/update_object_properties_by_name', String, queue_size=10)
    update_pos_pub = rospy.Publisher('/update_object_coords_by_name', String, queue_size=10)
    voice_pub = rospy.Publisher('/RCOMMessage', RCOMMessage, queue_size=10)
    
    forward_messages = partial(forward,
                               robotname=robotname,
                               voice_pub=voice_pub,
                               obs_pub=obs_pub,
                               asr_pub=asr_pub,
                               update_prop_pub=update_prop_pub,
                               update_pos_pub=update_pos_pub)
    
    s = rospy.Subscriber('/RCOMMessage', RCOMMessage, forward_messages)
    
    print "Ready to forward data."
    rospy.spin()
