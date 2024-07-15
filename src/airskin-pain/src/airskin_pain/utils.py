import PyKDL as kdl
import numpy as np
import rospy
from std_msgs.msg import Int32


class KDLHandle:
    def __init__(self, end_effector, kdl_tree):
        self.end_effector = end_effector
        # Create chain from base_link to end-effector
        self.chain = kdl_tree.getChain("base_link", self.end_effector)
        # Init solver for the given chain;
        self.vel_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        # Get num active joints (active = not fixed)
        self.num_joints = self.chain.getNrOfJoints()
        # Init output array for joint velocities
        self.velocities = kdl.JntArray(self.num_joints)
        # FK solver
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        # output frame for FK
        self.fk_frame = kdl.Frame()
        # Jac solver
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        # jac output
        self.jac = kdl.Jacobian(self.num_joints)
        # dyn solver
        self.dyn_solver = kdl.ChainDynParam(self.chain, kdl.Vector(0, 0, -9.81))
        # dyn output
        self.dyn = kdl.JntSpaceInertiaMatrix(self.num_joints)
        # FK vel solver
        self.fk_vel_solver = kdl.ChainFkSolverVel_recursive(self.chain)
        # output frame for FK vel
        self.fk_vel_frame = kdl.FrameVel()

        segment_mass = 0
        segments = self.chain.getNrOfSegments()
        for s in range(segments):
            segment_mass += self.chain.getSegment(s).getInertia().getMass()

        self.mass = segment_mass

    @staticmethod
    def np_to_kdl(ar):
        kdl_ar = kdl.JntArray(len(ar))
        for idx, _ in enumerate(ar):
            kdl_ar[idx] = _
        return kdl_ar

    @staticmethod
    def kdl_to_np(ar, rows=0):
        if rows == 0:
            rows = ar.rows()
        python_ar = np.zeros(rows)
        for idx, _ in enumerate(ar):
            python_ar[idx] = _
        return python_ar

    @staticmethod
    def kdl_to_np_mat(mat, rows=0, cols=0):
        if rows == 0:
            rows = mat.rows()
        if cols == 0:
            cols = mat.columns()
        out = np.zeros((rows, cols))
        for i in range(rows):
            for j in range(cols):
                out[i, j] = mat[i, j]
        return out


class AirskinFeedbackStandAlone:
    THRESHOLDS = {"sim": np.array([10, 650, 1000]), "real": np.array([15, 200, 300])}
    NUM_LINKS = np.array([4, 6, 1])

    def __init__(self, setup, thresholds, lag, influence):
        if len(thresholds) == 0:
            self.thresholds = None
        else:
            self.thresholds = []
            for _ in range(3):
                self.thresholds += [self.THRESHOLDS[setup][thresholds[_]]] * self.NUM_LINKS[_]
        self.setup = setup
        self.lag = lag
        self.influence = influence
        self.signals = [False] * 11
        self.filteredY = None
        self.filteredStdY = None
        self.stdFilter = None
        self.old_value = 0
        self.i = 0
        self.sub = None
        self.mode = 0
        self.pub = rospy.Publisher("/airskin_pain/touch", Int32, queue_size=1)
        self.msg = Int32()
        self.untouch_timestamp = None

    def __call__(self, msg):
        if self.thresholds is None:
            return 0
        new_value = msg.pressures
        if self.filteredY is None:
            self.filteredY = np.tile(new_value, (self.lag, 1))
            self.filteredStdY = np.tile(new_value, (self.lag, 1))
            self.avgFilter = np.mean(self.filteredY, axis=0)
            self.stdFilter = np.std(self.filteredStdY, axis=0)
            self.old_value = new_value

        if self.i == self.lag*2 and self.setup == "real":
            rospy.logerr("INIT DONE")

        if self.i >= self.lag:
            self.signals = np.subtract(new_value, self.avgFilter) > np.multiply(self.thresholds, self.stdFilter)

        if self.untouch_timestamp is not None and rospy.get_time() - self.untouch_timestamp > 2:
            self.untouch_timestamp = None

        if self.untouch_timestamp is None and self.i >= self.lag*2 and self.mode == 0 and np.any(self.signals):
            self.msg.data = np.nonzero(self.signals)[0][0]
            self.pub.publish(self.msg)
            self.mode = 1
        elif self.mode == 1 and not np.any(self.signals):
            self.msg.data = -1
            self.pub.publish(self.msg)
            self.mode = 0
            self.untouch_timestamp = rospy.get_time()

        calc_touch = np.multiply((self.influence * np.array(new_value) + (1 - self.influence) * self.old_value), self.signals)
        calc_no_touch = np.multiply(new_value, np.logical_not(self.signals))

        calc_no_touch_delay = np.multiply(self.filteredY[self.i % self.lag], np.logical_not(self.signals))
        calc_touch_delay = np.multiply(self.filteredStdY[self.i % self.lag], self.signals)

        self.old_value = self.filteredY[self.i % self.lag]
        self.filteredY[self.i % self.lag] = np.add(calc_touch, calc_no_touch)
        self.filteredStdY[self.i % self.lag] = np.add(calc_touch_delay, calc_no_touch_delay)

        self.avgFilter = np.mean(self.filteredY, axis=0)
        newStdFilter = np.std(self.filteredStdY, axis=0)
        self.stdFilter[np.logical_not(self.signals)] = newStdFilter[np.logical_not(self.signals)]

        self.i += 1

    def thr_cb(self, msg):
        self.thresholds = np.array(self.THRESHOLDS[self.setup][np.array(msg.data)])