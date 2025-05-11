#!/usr/bin/env python3

import math
import rospy

class FirstOrder:
  def __init__(self, K=0.0, T=0.0, system_output=0.0):
    """
    First-order system class for tune PID example. 

    Parameters:
      K: Gain of the system
      T: Time constant of the system
      system_output: Initial output of the system
    """
    self.K = K
    self.T = T
    self.system_output = system_output
    
    self.previous_output = 0.0
    self.previous_time = 0.0

  def set_system_parameters(self, K, T):
    self.K = K
    self.T = T

  def get_system_parameters(self):
    return self.K, self.T
  
  def simulate(self, command, dt):
    """
    Simulate the first-order system.

    Parameters:
      input_signal: Input signal to the system
      dt: Time difference (seconds)

    Returns:
      output: Output of the system
    """
    output = self.system_output + (self.K * command - self.system_output) * (1 - math.exp(-dt / self.T))

    return output