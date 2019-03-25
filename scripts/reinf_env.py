#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import numpy as np

class mobile_robot_reinforce:
   'For controlling mobilr boto in gazebo through openai gym'

   def __init__(self):
      rospy.init_node('mobile_robot', anonymous=True)
      self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
      self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
      self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
      self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
      # self.laser_sub = rospy.Subscriber('laserscan', LaserScan, self.callback)

      self.control_speed = 0.0
      self.control_turn = 0.0
      self.vel_msg = Twist()

      self.vel_msg.linear.x = self.control_speed
      self.vel_msg.linear.y = 0.0
      self.vel_msg.linear.z = 0.0
      self.vel_msg.angular.x = 0.0
      self.vel_msg.angular.y = 0.0
      self.vel_msg.angular.z = self.control_turn

   def read_data(self):
       data = None
       done = False
       while data is None:
           try:
               data = rospy.wait_for_message('/laserscan', LaserScan, timeout=5)
           except:
               print "Laserscan date unavailable from topic /laserscan"
               pass
       obs = np.array([data.ranges[0],data.ranges[180], data.ranges[360] ,data.ranges[540] ,data.ranges[719]])
       if(obs < 0.3).any():
           done = True
       return obs ,done
   
   def reset(self):
       rospy.wait_for_service('/gazebo/reset_world')
       self.reset_simulation()
       rospy.wait_for_service('/gazebo/unpause_physics')
       try:
           self.unpause()
       except rospy.ServiceException, e:
           print ("/gazebo/unpause_physics service call failed")
       obs , _ = self.read_data()
       rospy.wait_for_service('/gazebo/pause_physics')
       try:
           #resp_pause = pause.call()
           self.pause()
       except rospy.ServiceException, e:
           print ("/gazebo/pause_physics service call failed")

       return obs


   def display(self):
      print "currently:\tspeed %sstates\tturn %s " % (self.control_speed,self.control_turn)

   def forward(self):
        control_speed = 0.2
        control_turn = 0.0
        return control_speed ,control_turn

   def backward(self):
        control_speed = -0.2
        control_turn = 0.0
        return control_speed ,control_turn

   def forward_left(self):
        control_speed = 0.2
        control_turn = 0.2
        return control_speed ,control_turn

   def forward_right(self):
        control_speed = 0.2
        control_turn = -0.2
        return control_speed ,control_turn

   def only_left(self):
        control_speed = 0.2
        control_turn = 0.2
        return control_speed ,control_turn

   def stop(self):
        control_speed = 0.0
        control_turn = 0.2
        return control_speed ,control_turn

   def onehot_to_strings(self,action):
         if action== 0: return self.forward()
         elif action  == 1: return self.forward_left()
         elif action  == 2: return self.forward_right()
         elif action  == 3: return self.backward()


   def step(self,action):
       rospy.wait_for_service('/gazebo/unpause_physics')
       try:
           self.unpause()
       except rospy.ServiceException, e:
           print ("/gazebo/unpause_physics service call failed")

       print(self.onehot_to_strings(action))
       self.vel_msg.linear.x, self.vel_msg.angular.z = self.onehot_to_strings(action)
       self.velocity_publisher.publish(self.vel_msg)

       obs ,done = self.read_data()

       rospy.wait_for_service('/gazebo/pause_physics')
       try:
           #resp_pause = pause.call()
           self.pause()
       except rospy.ServiceException, e:
           print ("/gazebo/pause_physics service call failed")

       if done == True:
           reward=-10
           print 'negative reward'
       else:
           reward = 1

       return obs, reward,done


# if __name__ == '__main__':
#     try:
#         robot1 = mobile_robot_reinforce()
#         robot1.backward()
#         robot1.move()
#     except rospy.ROSInterruptException: pass
