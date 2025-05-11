#!/usr/bin/env python3

from pid import PID
from pid_tuner.msg import MethodType
from pid_tuner.srv import TunePID, TunePIDRequest, TunePIDResponse
import rospy 

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
    self.sleep_time_after_reset = rospy.get_param('pid_tuner/sleep_time_after_reset', 10.0)
    self.step_input = rospy.get_param('pid_tuner/step_input', 0.0)
    self.initial_condition = rospy.get_param('pid_tuner/initial_condition', 0.0)
    self.time_constant = rospy.get_param('pid_tuner/time_constant', 0.0)
    self.status_topic_name = rospy.get_param('pid_tuner/status/topic_name', '/pid_tuner/status')  
    self.status_topic_type = rospy.get_param('pid_tuner/status/topic_type', 'std_msgs/String')
    self.status_message_type = rospy.get_param('pid_tuner/status/message_type', 'std_msgs.msg')
    self.status_data_type = rospy.get_param('pid_tuner/status/data_type', 'String')
    self.status_message = rospy.get_param('pid_tuner/status/message', '.data')
    self.status_subscriber_name = rospy.get_param('pid_tuner/status/subscriber_name', 'pid_tuner_status_subscriber')
    self.callback_function_name = rospy.get_param('pid_tuner/status/callback_function_name', 'callback_function')
    self.queue_size = rospy.get_param('pid_tuner/status/queue_size', 10)
    self.status_variable_name = rospy.get_param('pid_tuner/status/variable_name', 'status_variable')
    self.command_topic_name = rospy.get_param('pid_tuner/command/topic_name', '/pid_tuner/command')
    self.command_topic_type = rospy.get_param('pid_tuner/command/topic_type', 'std_msgs/String')
    self.command_message_type = rospy.get_param('pid_tuner/command/message_type', 'std_msgs.msg')
    self.command_data_type = rospy.get_param('pid_tuner/command/data_type', 'String')
    self.command_message = rospy.get_param('pid_tuner/command/message', '.data')
    self.command_publisher_name = rospy.get_param('pid_tuner/command/publisher_name', 'pid_tuner_command_publisher')
    self.command_variable_name = rospy.get_param('pid_tuner/command/variable_name', 'command_variable')
    
    # Service
    self.pid_tuner_service = rospy.Service('tune_pid', TunePID, self.tune_pid_service_advertiser)

    # Class variables
    self.pid = PID()
    self.rate = rospy.Rate(self.rate)  
    self.sleep_time_after_reset = rospy.Duration(self.sleep_time_after_reset)
    self.callback_function_list = {}
    self.method_type = None
    self.Ku = 0.0
    self.Tu = 0.0
    self.Kp = 0.0
    self.Ki = 0.0
    self.Kd = 0.0
    self.Ti = 0.0
    self.Td = 0.0

    # Dynamic functions via exec()
    self.import_libraries()
    self.define_messages()
    self.define_class_variables()
    self.define_callback_function()
    self.define_subscriber()
    self.define_publisher()
    
    # Initialize dynamic variables
    exec("self.%s = %s()" % (self.status_variable_name, self.status_data_type))
    exec("self.%s = %s()" % (self.command_variable_name, self.command_data_type))

  def tune_pid_service_advertiser(self, req: TunePIDRequest):
    """
    Service callback function for tuning PID controller.

    :param req: Request object containing the method type and parameters.
    :return resp: Response object with the tuned PID gains.
    """
    self.method_type = req.method.method_type
    response = TunePIDResponse()
    
    try:
      self.start_tune()
      if self.method_type == MethodType.P:
        self.Kp = 0.5 * self.Ku
        self.Ki = 0.0
        self.Kd = 0.0
      elif self.method_type == MethodType.PI:
        self.Kp = 0.45 * self.Ku
        self.Ti = 0.83 * self.Tu
        self.Ki = 0.54 * self.Ku / self.Tu
        self.Kd = 0.0
      elif self.method_type == MethodType.PD:
        self.Kp = 0.8 * self.Ku
        self.Td = 0.125 * self.Tu
        self.Kd = 0.10 * self.Ku * self.Tu
        self.Ki = 0.0
      elif self.method_type == MethodType.PID:
        self.Kp = 0.6 * self.Ku
        self.Ti = 0.5 * self.Tu
        self.Td = 0.125 * self.Tu
        self.Ki = 1.2 * self.Ku / self.Tu
        self.Kd = 0.075 * self.Ku * self.Tu
      elif self.method_type == MethodType.PESSEN_INTEGRAL:
        self.Kp = 0.7 * self.Ku
        self.Ti = 0.4 * self.Tu
        self.Td = 0.15 * self.Tu
        self.Ki = 1.75 * self.Ku / self.Tu
        self.Kd = 0.105 * self.Ku * self.Tu
      elif self.method_type == MethodType.SOME_OVERSHOOT:
        self.Kp = 0.33 * self.Ku
        self.Ti = 0.5 * self.Tu
        self.Td = 0.33 * self.Tu
        self.Ki = 0.66 * self.Ku / self.Tu
        self.Kd = 0.11 * self.Ku * self.Tu
      elif self.method_type == MethodType.NO_OVERSHOOT:
        self.Kp = 0.2 * self.Ku
        self.Ti = 0.5 * self.Tu
        self.Td = 0.33 * self.Tu
        self.Ki = 0.4 * self.Ku / self.Tu
        self.Kd = 0.066 * self.Ku * self.Tu
      else:
        rospy.logwarn("Invalid method type. Please choose a valid method.")

      response.Kp = self.Kp
      response.Ki = self.Ki
      response.Kd = self.Kd
    except Exception as e:
      rospy.logerr(f"Error in PID tuning: {e}")

    self.Ku = self.initial_Ku
    self.Tu = self.initial_Tu
    self.Kp = 0.0
    self.Ki = 0.0
    self.Kd = 0.0
    self.Ti = 0.0
    self.Td = 0.0

    return response

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
         (self.status_subscriber_name, self.status_topic_name, self.status_data_type, self.callback_function_name, self.queue_size))

  def define_publisher(self):
    """
    Define publisher for the command topic.

    For example:
      self.command_publisher = rospy.Publisher('/pid_tuner/command', String, queue_size=10)
    """
    exec("self.%s = rospy.Publisher('%s', %s, queue_size=%d)" % 
         (self.command_publisher_name, self.command_topic_name, self.command_data_type, self.queue_size))

  def start_tune(self):
    """
    Method to start the PID tuning process.
    This method will be called when the service is called.
    """
    initial_time = rospy.get_time()
    current_time = initial_time
    previous_time = initial_time
    self.Ku = self.initial_Ku
    self.Tu = self.initial_Tu
    self.pid.set_pid_gains(self.Ku, 0.0, 0.0)

    while not rospy.is_shutdown():
      current_time = rospy.get_time()
      elapsed_time = current_time - initial_time

      if elapsed_time >= self.time_constant:
        self.Ku += self.dKu
        initial_time = current_time
        self.pid.set_pid_gains(self.Ku, 0.0, 0.0)
        self.pid.reset()
        setattr(getattr(self, self.command_variable_name), self.command_message, self.initial_condition)
        getattr(self, self.command_publisher_name).publish(getattr(self, self.command_variable_name))
        rospy.sleep(self.sleep_time_after_reset.to_sec())
        rospy.loginfo(f"Ku: {self.Ku}, Tu: {self.Tu}")
      else:
        dt = current_time - previous_time
        error = self.step_input - getattr(getattr(self, self.status_variable_name), self.status_message)
        control_input = self.pid.pid_output(error, dt)
        setattr(getattr(self, self.command_variable_name), self.command_message, control_input)
        getattr(self, self.command_publisher_name).publish(getattr(self, self.command_variable_name))      
        previous_time = current_time
        self.rate.sleep()

if __name__ == '__main__':
  try:
    pid_tuner = PIDTuner()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
  except Exception as e:
    rospy.logerr(f"An error occurred: {e}")
  finally:
    rospy.loginfo("PID Tuner node has been shut down.")