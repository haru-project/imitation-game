#!/usr/bin/env python3

import rospy
import random
import actionlib
# import haru_guessing_game.msg
import idmind_tabletop_msgs.msg
import strawberry_ros_msgs.msg
import strawberry_ros_msgs.srv
import std_msgs.msg
import std_srvs.srv
import numpy as np

#For gestures have same ones for Hi or Stop. For OK the routines for Yes. Are creating new routines per each
#Hi is victory, to celebrate. 

ROUTINE_MAPPING = {
    "anger": "600;1001",
    "disgust": "1011;1017;1036",
    "fear": "109;510;508",
    "joy": "107;115;146;422;604;643;818;900;1008;1010;1032",
    "neutral": "325;326;407;902",
    "sadness": "111;113;121;300;304;419;420;603;641;1040",
    "surprise": "129;131;606;904",
    "Hi":"620;622",
    "Stop": "411;412;404;405",
    "Turn Left":"2002",
    "Turn Right": "2003",
    "Ok":"11;102;103;104;322;407;408",
    "Forward":"2004",
    "Backward":"2005",
    "Waving": "411;412;404;405",
    "Shaking head":"105",
    "Nodding": "104",  
    }


class GestureImitator(object):
    # _feedback = haru_guessing_game.msg.GuessingFeedback()
    # _result = haru_guessing_game.msg.GuessingResult()

    def __init__(self, name):
        # self._action_name = name
        # self._as = actionlib.SimpleActionServer(
        #    self._action_name, haru_guessing_game.msg.GuessingAction, execute_cb=self.execute_cb, auto_start=False)
        #self.routine_client = actionlib.SimpleActionClient(
        #    'idmind_tabletop/action_routine', idmind_tabletop_msgs.msg.RoutineAction)
       #self.routine_client.wait_for_server()
        self.people_sub = rospy.Subscriber(
            '/strawberry/people', strawberry_ros_msgs.msg.People, self.peopleCb)

        self.faces_sub = rospy.Subscriber(
            '/strawberry/faces/results', strawberry_ros_msgs.msg.Faces, self.facesCb)
        self.closest_person = rospy.ServiceProxy(
            '/strawberry/get_closest_person', strawberry_ros_msgs.srv.GetClosestPerson)
        #Hand gesture recognizer from Mediapipe
        self.hand_gesture_sub = rospy.Subscriber(
            '/gesture/hand_sign', std_msgs.msg.String, self.handCb)
        # NLP ssml routine selector
        self.ssml_client = rospy.ServiceProxy(
            '/nlp/text_to_ssml', strawberry_ros_msgs.srv.TextToSSML)
        
        #Activity recognition
        self.activity_sub = rospy.Subscriber(
            '/human_action', std_msgs.msg.String, self.activityCb)       
        
        self.emotion_counter = 0
        self.current_emotion = ""
        self.hand_counter = 0
        self.current_hand_gesture = ""
        self.current_activity = ""
        self.activity_counter = 0
        #self.selectRoutineActivity()
        self.selectRoutineEmotion()
        #self.selectRoutineHand()
        # Start the ROS action server
        # self._as.start()
    

    def activityCb(self, msg):
        
        if (msg.data == self.current_activity ):
            self.activity_counter  +=1
        else:
            self.activity_counter = 0
            self.current_activity = msg.data


    def handCb(self, msg):
        
        if (msg.data == self.current_hand_gesture):
            self.hand_counter +=1
        else:
            self.hand_counter = 0
            self.current_hand_gesture = msg.data

    #Check list of labels in: https://github.com/quic/sense/blob/master/sense/downstream_tasks/action_recognition/__init__.py
    def selectRoutineActivity(self):

        rospy.loginfo("Select routine")
        # wait 3 seconds until we monitor the emotion, or a specific timeout
        rospy.sleep(4)
        while (self.activity_counter  < 2):
            rospy.sleep(0.5)
        if (self.activity_counter > 1):  # set a max limit
            rospy.loginfo("User gesture is: " + self.current_activity )
            keyword = self.current_activity

        #Choose the routine, if it is Doing other things it will not be in the keywords on purpose
        if keyword in ROUTINE_MAPPING:
            routine_id = int(np.random.choice(ROUTINE_MAPPING[self.current_activity].split(';')))
            rospy.loginfo("Playing this routine: "+ str(routine_id))
        else:
            print("Choose a random routine if key does not exist")   
        self.activity_counter = ""
        self.activity_counter  = 0  

    def selectRoutineHand(self):

        rospy.loginfo("Select routine")
        # wait 3 seconds until we monitor the emotion, or a specific timeout
        rospy.sleep(3)
        while (self.hand_counter < 4):
            rospy.sleep(0.5)
        if (self.hand_counter > 3):  # set a max limit
            rospy.loginfo("User gesture is: " + self.current_hand_gesture)
            keyword = self.current_hand_gesture

        #Choose the routine
        if keyword in ROUTINE_MAPPING:
            routine_id = int(np.random.choice(ROUTINE_MAPPING[self.current_hand_gesture].split(';')))
            rospy.loginfo("Playing this routine: "+ str(routine_id))
        else:
            print("Choose a random routine if key does not exist")   
        self.current_hand_gesture = ""
        self.hand_counter  = 0     

    def peopleCb(self, data):
        if (data.people):
            # try getting user emotion, but often we get empty
            # rospy.loginfo(data.people[0].face.emotions)
            # Get closest person as long as there are some people in the scene
            try:
                resp = self.closest_person(
                    strawberry_ros_msgs.srv.GetClosestPersonRequest())
                close_id = resp.person_id
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

    def facesCb(self, data):
        if (data.faces):
            highest_em = ''
            highest_em_conf = 0.0
            for em in (data.faces[0].emotions):
                if (em.confidence > highest_em_conf):
                    highest_em_conf = em.confidence
                    highest_em = em.value
            rospy.loginfo("Highest confidence emotion is " +
                          highest_em + " with percentage: "+str(highest_em_conf))
            # Check to see if same emotion has been read a few times, for instance, happy emotion.
            if (highest_em == self.current_emotion):
                self.emotion_counter += 1
            else:
                self.emotion_counter = 0
                self.current_emotion = highest_em
            highest_em = ''
            highest_em_conf = 0.0

    def selectRoutineEmotion(self):

        rospy.loginfo("Select routine")
        # wait 3 seconds until we monitor the emotion, or a specific timeout
        rospy.sleep(3)
        while (self.emotion_counter < 4):
            rospy.sleep(0.5)
        if (self.emotion_counter > 3):  # set a max limit
            rospy.loginfo("User emotion is: " + self.current_emotion)
            keyword = self.current_emotion
        #Choose the routine
        if keyword in ROUTINE_MAPPING:
            routine_id = int(np.random.choice(ROUTINE_MAPPING[self.current_emotion].split(';')))
            rospy.loginfo("Playing this routine: "+ routine_id)
        else:
            print("Choose a random routine if key does not exist")
        self.current_emotion = ""
        self.emotion_counter = 0

    def selectRoutineSSML(self):

        rospy.loginfo("Select routine")
        # wait 3 seconds until we monitor the emotion, or a specific timeout
        rospy.sleep(3)
        while (self.emotion_counter < 4):
            rospy.sleep(0.5)
        if (self.emotion_counter > 3):  # set a max limit
            rospy.loginfo("User emotion is: " + self.current_emotion)
            # Feed it to the NLP ROS
            ssml_req = strawberry_ros_msgs.srv.TextToSSMLRequest()
            try:
                ssml_req.text = self.current_emotion
                ssml_resp = self.ssml_client(ssml_req)
                em_routine = ssml_resp.routine
                rospy.loginfo(em_routine)
                # Return this routine back as action response
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            self.current_emotion = ""
            self.emotion_counter = 0


if __name__ == '__main__':
    rospy.init_node('haru_gesture_imitator')
    print(rospy.get_name())
    server = GestureImitator(rospy.get_name())
    rospy.spin()
