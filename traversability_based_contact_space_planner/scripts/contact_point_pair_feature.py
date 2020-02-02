import numpy as np
import math
import openravepy as rave
import IPython


class contact_point_pair_feature: # x, y
    def __init__(self,contact_points_pair): # the input two contact points
        self.cp1 = contact_points_pair[0]
        self.cp2 = contact_points_pair[1]

        self.d = np.linalg.norm(self.cp1.position - self.cp2.position)

        cp12_unit_vec = (self.cp2.position - self.cp1.position) / self.d
        cp21_unit_vec = -cp12_unit_vec

        self.theta1 = math.acos(np.dot(cp21_unit_vec,self.cp1.normal))
        self.theta2 = math.acos(np.dot(cp12_unit_vec,self.cp2.normal))

        if self.cp1.group == 0 and self.cp2.group == 0:
            self.type = 0 # feet contacts pair
        elif self.cp1.group == 1 and self.cp2.group == 1:
            self.type = 1 # palms contacts pair
        elif (self.cp1.group == 0 and self.cp2.group == 1) or (self.cp1.group == 1 and self.cp2.group == 0):
            self.type = 2 # foot and palm contacts pair

    def __eq__(self, other):
        return (self.d == other.d and self.theta1 == other.theta1 and self.theta2 == other.theta2)

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_valid(self):
        if self.type == 0:
            return (self.d <= 0.85)
        elif self.type == 1:
            return (self.d <= 2.0)
        elif self.type == 2:
            return (self.d <= 2.0)

    def get_feature_vector(self):
        return [self.d, 0.5/math.pi*self.theta1, 0.5/math.pi*self.theta1]
