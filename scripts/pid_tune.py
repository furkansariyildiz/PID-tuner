#!/usr/bin/env python3

import os
import rospy 
from pid_tuner.msg import MethodType
from pid_tuner.srv import TunePID, TunePIDRequest, TunePIDResponse

class PIDTuner:
  """
  PID Tuner class for tuning PID controllers in ROS. 
  With this class, you can get ideal Kp, Ki and Kd gains via Ziegler-Nichols method.

  For Controller Types:
  - P: Kp = 0.5 * Ku
  - PI: Kp = 0.45 * Ku, Ti = 0.83 * Tu, Ki = 0.54 * Ku / Tu
  - PD: Kp = 0.8 * Ku, Td = 0.125 * Tu, Kd = 0.10 * Ku * Tu
  - PID: Kp = 0.6 * Ku, Ti = 0.5 * Tu, Td = 0.125 * Tu, Ki = 1.2 * Ku / Tu, Kd = 0.075 * Ku * Tu
  - Pessen Integral Rule: Kp = 0.7 * Ku, Ti = 0.4 * Tu, Td = 0.15 * Tu, Ki = 1.75 * Ku / Tu, Kd = 0.105 * Ku * Tu
  - Some Overshoot: Kp = 0.33 * Ku, Ti = 0.5 * Tu, Td = 0.33 * Tu, Ki = 0.66 * Ku / Tu, Kd = 0.11 * Ku * Tu
  - No Overshoot: Kp = 0.2 * Ku, Ti = 0.5 * Tu, Td = 0.33 * Tu, Ki = 0.4 * Ku / Tu, Kd = 0.066 * Ku * Tu
  """
  def __init__(self):
    rospy.init_node('pid_tuner', anonymous=True)
    
    # Parameter initialization
    self.initial_Ku = rospy.get_param('pid_tuner/initial_Ku', 0.0)
    self.initial_Tu = rospy.get_param('pid_tuner/initial_Tu', 0.0)
    self.dKu = rospy.get_param('pid_tuner/dKu', 0.0)
    self.rate = rospy.get_param('pid_tuner/rate', 10.0)
    self.step_input = rospy.get_param('pid_tuner/step_input', 0.0)
    self.initial_condition = rospy.get_param('pid_tuner/initial_condition', 0.0)
    self.time_constant = rospy.get_param('pid_tuner/time_constant', 0.0)
    self.status_topic_name = rospy.get_param('pid_tuner/status/topic_name', '/pid_tuner/status')  
    self.status_topic_type = rospy.get_param('pid_tuner/status/topic_type', 'std_msgs/String')
    self.status_message_type = rospy.get_param('pid_tuner/status/message_type', 'std_msgs.msg')
    self.status_data_type = rospy.get_param('pid_tuner/status/data_type', 'String')
    self.subscriber_name = rospy.get_param('pid_tuner/status/subscriber_name', 'pid_tuner_status_subscriber')
    self.callback_function_name = rospy.get_param('pid_tuner/status/callback_function_name', 'callback_function')
    self.queue_size = rospy.get_param('pid_tuner/status/queue_size', 10)
    self.status_variable_name = rospy.get_param('pid_tuner/status/variable_name', 'status_variable')
    self.command_topic_name = rospy.get_param('pid_tuner/command/topic_name', '/pid_tuner/command')
    self.command_topic_type = rospy.get_param('pid_tuner/command/topic_type', 'std_msgs/String')
    self.command_message_type = rospy.get_param('pid_tuner/command/message_type', 'std_msgs.msg')
    self.command_data_type = rospy.get_param('pid_tuner/command/data_type', 'String')
    self.command_variable_name = rospy.get_param('pid_tuner/command/variable_name', 'command_variable')
    
    self.rate = rospy.Rate(self.rate)  
    self.callback_function_list = {}

    self.import_libraries()
    self.define_messages()
    self.define_class_variables()
    self.define_callback_function()
    self.define_subscriber()

  def import_libraries(self):
    """
    Import libraries for status and command topics.

    For example:
      from std_msgs.msg import String
    """
    exec(("from %s import %s" % (self.status_message_type, self.status_data_type)), globals())
    exec(("from %s import %s" % (self.command_message_type, self.command_data_type)), globals())

  def define_messages(self):
    """
    Define messages as class variables.

    For example:
      self.status_variable = String()
      self.command_variable = String()
    """
    exec("%s = %s()" % (self.status_variable_name, self.status_data_type))
    exec("%s = %s()" % (self.command_variable_name, self.command_data_type))

  def define_class_variables(self):
    """
    Define dynamic class variables to use in the exec function.

    For example:
      self.string_variable = String()
    """
    exec("%s = %s" % (self.status_variable_name, self.status_data_type))
    exec("%s = %s" % (self.command_variable_name, self.command_data_type))

  def define_callback_function(self):
    """
    Define callback function for the status topic.

    For example:
      def callback_function(self, msg):
        self.status_variable = msg
    """
    exec("def %s(self, msg): self.%s = msg" %
         (self.callback_function_name, self.status_variable_name), globals(), self.callback_function_list)

    for function in self.callback_function_list:
      if  not hasattr(self.__class__, function):
        setattr(self.__class__, function, self.callback_function_list[function])
  
  def define_subscriber(self):
    """
    Define subscriber for the status topic.

    For example:
      self.status_subscriber = rospy.Subscriber('/pid_tuner/status', String, self.callback_function, queue_size=10)
    """
    exec("self.%s = rospy.Subscriber('%s', %s, self.%s, queue_size=%d)" % 
         (self.subscriber_name, self.status_topic_name, self.status_data_type, self.callback_function_name, self.queue_size))

  def run(self):
    while not rospy.is_shutdown():
      rospy.loginfo("PID Tuner.")
      self.rate.sleep()

if __name__ == '__main__':
  try:
    pid_tuner = PIDTuner()
    pid_tuner.run()
  except rospy.ROSInterruptException:
    pass
  except Exception as e:
    rospy.logerr(f"An error occurred: {e}")
  finally:
    rospy.loginfo("PID Tuner node has been shut down.")