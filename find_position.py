from Inverse_Kinematics import forward_kinematics
import numpy as np

class EndPosition:
    def __init__(self,q1, q2, q3, q4):
        self.teta1 = q1
        self.teta2 = q2
        self.teta3 = q3
        self.teta4 = q4

        self.dh_1 = forward_kinematics.DH(0, 0, 13.1179, self.teta1)
        self.dh_2 = forward_kinematics.DH(10.330, 0, 0, self.teta2)
        self.dh_3 = forward_kinematics.DH(10.922, 90, 0, self.teta3)
        self.dh_4 = forward_kinematics.DH(3.683000, 90, 0, self.teta4)

        self.final_dh = np.dot(np.dot(np.dot(self.dh_1.get_matrix(), self.dh_2.get_matrix()), self.dh_3.get_matrix()),self.dh_4.get_matrix())

    def get_final_matrix(self):
        return self.final_dh



end_p = EndPosition(0,0,0,0)
