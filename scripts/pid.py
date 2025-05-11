#!/usr/bin/env python3

class PID:
  def __init__(self, Kp=0.0, Ki=0.0, Kd=0.0):
    """
    PID controller class for tuning PID controllers in ROS.
    """
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

    self.P = 0.0
    self.I = 0.0
    self.D = 0.0

    self.previous_error = 0.0
    
  def set_pid_gains(self, Kp, Ki, Kd):
    """
    Setter the PID gains.

    Parameters:
      Kp: Proportional gain
      Ki: Integral gain
      Kd: Derivative gain
    """
    self.Kp = Kp
    self.Ki = Ki
    self.Kd = Kd

  def get_pid_gains(self):
    """
    Getter the PID gains.

    Returns:
      Gains: Tuple of (Kp, Ki, Kd)
    """
    return self.Kp, self.Ki, self.Kd
  
  def reset(self):
    """
    Reset the PID controller.
    """
    self.P = 0.0
    self.I = 0.0
    self.D = 0.0
    self.previous_error = 0.0
  
  def pid_output(self, error, dt):
    """
    Calculate the PID output.

    Parameters:
      error: Error signal
      dt: Time difference (seconds)

    Returns:
      output: PID output
    """
    self.P = self.Kp * error
    self.I += self.Ki * error * dt
    self.D = self.Kd * (error - self.previous_error) / dt 
    self.previous_error = error
    return self.P + self.I + self.D