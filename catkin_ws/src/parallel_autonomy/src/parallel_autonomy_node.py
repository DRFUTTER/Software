#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTags, BoolStamped
from std_msgs.msg import String, Int16, Joy #Imports msg

class ParallelAutonomyNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        self.turn.LEFT = 0
        self.turn.RIGHT = 1
        self.turn.STRAIGHT = 2
        self.turn.NONE = 3

        self.availableTurns = []
        self.turn_direction = self.turn.NONE

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTags, self.cbTag, queue_size=1)
        self.sub_joy = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
       
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz


    def cbJoy(self,msg):

        if msg.buttons[1] and self.turn.LEFT in self.availableTurns: # or self.joy.axes[3] > 0.2:
            if self.turn_direction == self.turn.LEFT:
                self.turn_direction = self.turn.STRAIGHT
            else:
                self.turn_direction = self.turn.LEFT
        elif msg.buttons[2] and self.turn.RIGHT in self.availableTurns: #or self.joy.axes[3] < -0.2:
            if self.turn_direction == self.turn.RIGHT:
                self.turn_direction = self.turn.STRAIGHT
            else:
                self.turn_direction = self.turn.RIGHT
        if self.turn_direction == self.turn.STRAIGHT and self.turn.STRAIGHT not in self.availableTurns and if len(self.availableTurns)>0:
            self.turn_direction = self.turn.NONE
        # blink leds via publish

    def cbTag(self, tag_msgs):
        if(self.fsm_mode == "INTERSECTION_CONTROL"):
            #loop through list of april tags
            for taginfo in tag_msgs.infos:
                print taginfo
                rospy.loginfo("[%s] taginfo." %(taginfo))
                if(taginfo.tag_type == taginfo.SIGN):
                    self.availableTurns = []
                    #go through possible intersection types
                    signType = taginfo.traffic_sign_type
                    if(signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT):
                        self.availableTurns = [self.turn.STRAIGHT,self.turn.LEFT]
                    elif (signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT):
                        self.availableTurns = [self.turn.RIGHT,self.turn.STRAIGHT]
                    elif (signType== taginfo.FOUR_WAY):
                        self.availableTurns = [self.turn.RIGHT,self.turn.STRAIGHT,self.turn.LEFT]
                    elif (signType == taginfo.T_INTERSECTION):
                        self.availableTurns = [self.turn.RIGHT,self.turn.LEFT]

                    #now randomly choose a possible direction
                    if(len(availableTurns)>0):
                        self.turn_type = chosenTurn
                        self.pub_turn_type.publish(self.turn_type)
                        rospy.loginfo("[%s] possible turns %s." %(self.node_name,availableTurns))
                        rospy.loginfo("[%s] Turn type now: %i" %(self.node_name,self.turn_type))

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state




        if(self.fsm_mode == "WAITING_FOR_TURN_DIRECTION"):
            self.availableTurns = []
            self.turn_direction = self.turn.STRAIGHT
            rospy.sleep(2)
            while self.turn_direction is self.turn.NONE:
                pass
            self.pub_turn_type.publish(self.turn_direction) # make sure mapping to turn_type is ok
            rospy.loginfo("[%s] Turn type: %i" %(self.node_name, self.turn_direction))
            
    

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('parallel_autonomy_node', anonymous=False)

    # Create the NodeName object
    node = ParallelAutonomyNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()