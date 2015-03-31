"""This module helps load and read holds.  Defines
the Hold class, and the functions readHold(text) and writeHold(h).
"""

import vectorops
import so3

class Hold:
    """A Hold, similar to the Hold class in the C++ RobotSim library.

    Attributes:
        - link: the link index
        - ikConstraint: an IKObjective object
          (see klampt.robotsim.IKObjective or klampt.ik.objective)
        - contacts: a list of ContactPoint objects
          (see klampt.contact.ContactPoint)
    """
    def __init__(self):
        self.link = None
        self.ikConstraint = None
        self.contacts = []


