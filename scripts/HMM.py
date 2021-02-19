#!/home/manel/python2_env/bin/python2
import numpy as np
import os
from math import cos, sin, tan, radians, sqrt
from hmmlearn import hmm
import time
import rospy
import scipy.stats as stats
import tf.transformations
import cv2
import threading
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
import roslib
from openface import pyopenface as of
from vizzy_playground.msg import FaceExtraction
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseArray,PoseStamped
from sensor_msgs.msg import CameraInfo,CompressedImage


class PeopleExtractor:

    def __init__(self):
        self.face_extract = FaceExtraction()
        self.pose_publisher = rospy.Publisher("faces", FaceExtraction, queue_size=1)
        
        self.people_sub = rospy.Subscriber("/people", PoseArray, self.callback ,queue_size=1)
        
        

    def callback(self,ros_data):

        people = ros_data
        
        person_extract = people.poses[0]

        position = person_extract.position
        orientation = person_extract.orientation
        rot = (orientation.x, orientation.y, orientation.z, orientation.w)
        R,P,Y = tf.transformations.euler_from_quaternion(rot)
        self.face_extract.poses_R = [P,Y,R]
        
        self.face_extract.poses_T = np.array([position.y,position.z,position.x])*1000
        self.pose_publisher.publish(self.face_extract)
        

class FaceExtractor:

    def __init__(self):

        # Initialize ROS subs and pubs

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #/vizzy/r_camera/image_rect_color/compressed
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage,
                                          self.callback, queue_size=1,
                                          buff_size=2**24)

        self.pose_publisher = rospy.Publisher("faces", FaceExtraction,
                                              queue_size=1)

        # Params
        self.crop_x = rospy.get_param('~crop_x_resolution', 1280)
        self.crop_y = rospy.get_param('~crop_y_resolution', 1280)
        self.rescale_factor = rospy.get_param('~rescale_factor', 1.0)

        # Initialize openface
        self.det_parameters = of.FaceModelParameters()
        self.det_parameters.model_location = rospy.get_param('~model_location',
            '/home/manel/OpenFace/build/bin/model/main_ceclm_general.txt')
        self.det_parameters.mtcnn_face_detector_location = rospy.get_param('~mtcnn_face_detector_location',
            '/home/manel/OpenFace/build/bin/model/mtcnn_detector/MTCNN_detector.txt')

        self.face_model = of.CLNF(self.det_parameters.model_location)
        self.intrinsics = None

        # Get intrinsics first
        MAX_WAIT = 60

        print("Waiting for camera_info topic to be available")

        try:
            #/vizzy/r_camera/camera_info
            camera_info_data = rospy.wait_for_message(
                "/usb_cam/camera_info", CameraInfo, MAX_WAIT)
        except rospy.ROSException as e:
            rospy.logfatal(
                "Could not get camera parameters after %d seconds... shutdown this node", MAX_WAIT)
            rospy.signal_shutdown("Failed to get camera parameters in time...")
        except rospy.ROSInterruptException as e:
            rospy.loginfo("Interrupt while waiting for camera parameters")

        if camera_info_data.header.frame_id == "r_camera_vision_link":
            P = np.array([camera_info_data.P]).reshape(3, 4)
            self.intrinsics = P[0:3, 0:3]
            self.intrinsics.shape = [3, 3]
            self.distCoeffs = np.zeros([1, len(camera_info_data.D)])
            print("Setting intrinsics of Vizzy")
        else:
            self.intrinsics = np.array([camera_info_data.K]).reshape(3, 3)
            self.distCoeffs = camera_info_data.D
            print("Setting intrinsics")

        if not self.face_model.loaded_successfully:
            print("error: could not load the landmark detector")

        if not self.face_model.eye_model:
            print("Warning: no eye model found")

        self.face_extract = FaceExtraction()
        self.sequence_reader = of.SequenceCapture()
        arguments = ['/home/manel/OpenFace/lib/local/FaceAnalyser/']
        face_analysis_params = of.FaceAnalyserParameters(arguments)
        self.face_analyser = of.FaceAnalyser(face_analysis_params)

        if (np.size(self.face_analyser.GetAUClassNames()) == 0 and np.size(self.face_analyser.GetAUClassNames()) == 0):
	        print("WARNING: no Action Unit models found")

    def callback(self, ros_data):

        if self.intrinsics is None:
            return

        self.header = ros_data.header

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cy = image_np.shape[0]/2
        cx = image_np.shape[1]/2

        crop_res = (self.crop_y, self.crop_x)

        # Coordinates offset from cropped to full image
        x_offset = cx-self.crop_x/2.0
        y_offset = cy-self.crop_y/2.0

        im = image_np[cy-crop_res[0]/2:cy+crop_res[0]/2,
                      cx-crop_res[1]/2:cx+crop_res[1]/2]

        s = 1024/(cx*2.0)

        im = cv2.resize(image_np, (int(s*cx), int(s*cy)))

        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY, dstCn=0)
        grayscale_image = np.ubyte(gray)
        success = of.DetectLandmarksInVideo(im, self.face_model, self.det_parameters, grayscale_image)

        f_x = self.intrinsics[0, 0]
        f_y = self.intrinsics[1, 1]
        c_x = self.intrinsics[0, 2]  # -x_offset
        c_y = self.intrinsics[1, 2]  # -y_offset

        # Estimation of the head pose through OF
        pose_estimate = of.GetPose(self.face_model, f_x, f_y, c_x, c_y)
        pose_estimate_stamped = PoseStamped()
        pose_estimate_stamped.header.frame_id = self.header.frame_id
        pose_estimate_stamped.header.stamp = self.header.stamp
        pose_estimate_stamped.pose.position.x = pose_estimate[0]
        pose_estimate_stamped.pose.position.y = pose_estimate[1]
        pose_estimate_stamped.pose.position.z = pose_estimate[2]

        pose_estimate_stamped.pose.orientation.w = 1


        try:
            transform = self.tfBuffer.lookup_transform("base_footprint",
                                                       self.header.frame_id,
                                                       self.header.stamp)
        except Exception as e:
            print("Error in transform: " + str(e))
            return


        ptf = tf2_geometry_msgs.do_transform_pose(pose_estimate_stamped, transform)
        euler_angles = tf.transformations.euler_from_quaternion(ptf.pose.orientation,axes='sxyz')

        # Estimation of head landmarks for gaze calculation
        landmarks = self.face_model.GetShape(f_x, f_y, c_x, c_y)

        gazeDirection0 = (0, 0, -1)
        gazeDirection1 = (0, 0, -1)

        # Estimation of gaze direction vectors
        gazeDirection0 = of.EstimateGaze(
            self.face_model, gazeDirection0, f_x, f_y, c_x, c_y, True)
        gazeDirection1 = of.EstimateGaze(
            self.face_model, gazeDirection1, f_x, f_y, c_x, c_y, False)

        self.face_analyser.AddNextFrame(im, self.face_model.detected_landmarks, self.face_model.detection_success,
                                        self.sequence_reader.time_stamp, self.sequence_reader.IsWebcam())

        # Estimation of action units for smile calculation
        aus_intensity = self.face_analyser.GetCurrentAUsReg()

        # Updating face_extract variable with the new information
        self.face_extract.poses_T = np.array(
             [ptf.pose.position.x, ptf.pose.position.y, ptf.pose.position.z])
        self.face_extract.poses_R = np.array(
             [euler_angles[1], euler_angles[2],euler_angles[0]])
        # self.face_extract.poses_T = np.array(
        #     [pose_estimate[0], pose_estimate[1], pose_estimate[2]])
        # self.face_extract.poses_R = np.array(
        #     [pose_estimate[3], pose_estimate[4]])
        self.face_extract.left_landmarks = np.array(
            [landmarks[0][36], landmarks[0][39], landmarks[1][36], landmarks[1][39]])
        self.face_extract.right_landmarks = np.array(
            [landmarks[0][42], landmarks[0][45], landmarks[1][42], landmarks[1][45]])
        self.face_extract.left_gaze_vector = gazeDirection0
        self.face_extract.right_gaze_vector = gazeDirection1
        self.face_extract.action_units = [
            aus_intensity[1][1], aus_intensity[4][1]]  # AU 06 and 12

        # Publishes face_extract into /faces topic
        self.pose_publisher.publish(self.face_extract)


class HMModel:

    def __init__(self):

        # Get trained greeting model
       
        self.accuracy = [0,0]
        while (min(self.accuracy)<0.8):
            self.model = self.get_trained_model()
            self.pi = self.model.startprob_
            self.accuracy = self.get_test_accuracy()

        self.robot_pose = PoseWithCovarianceStamped()
        self.face_extract = FaceExtraction()

        # Initiate publishers and subscribers
        
        self.state_pub = rospy.Publisher("/state", Int16,
                                              queue_size=1)
        self.face_sub = rospy.Subscriber("faces", FaceExtraction,
                                          self.callback, queue_size=1)

        self.alpha = self.pi
        self.movements=[0,0,0]
        self.count = 0
        self.prev_time = 0
        self.iteration = 0
        threading.Thread(target = self.keys_thread).start()
        self.key = ''
        try:
            transform = self.tfBuffer.lookup_transform("base_footprint",
                                                       "neck_tilt_link",
                                                       self.header.stamp)
        except Exception as e:
            print("Error in transform: " + str(e))
            return


        self.face_point = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        
        
        print("movements? (1:Distance Salutation, 2:Close Salutation, 3:Head Dip)")
        

    def get_trained_model(self):

        x_train = np.empty((1, 7))
        lengths_train = []

        for filename in os.listdir("good sequences/Train"):
        # Reads every sequence file in Train folder
            f = open(os.path.join('good sequences/Train', filename), 'r')
            lines = f.readlines()
            lines.pop(0)

            # Extracts info from the files
            for i in range(len(lines)):

                lines[i] = lines[i].strip()
                lines[i] = lines[i].split(' ')
                lines[i].pop(0)
                lines[i].pop(7)

                lines[i] = map(float, lines[i])

            # Concatenates the info into one vector with all the observations
            x_train = np.concatenate([x_train, lines], axis=0)
            lengths_train.append(np.shape(lines)[0])

        x_train = x_train[1:, :]

        # Trains a model using the observations vector and the lengths of the several sequences
        model = hmm.GaussianHMM(n_components=6, n_iter=100)
        model.fit(x_train, lengths_train)


        return model

    def get_test_accuracy(self):

        x_test = np.empty((1, 7))
        lengths_test = []
        states_test = []

        # Reads files from Test folder
        for filename in os.listdir("good sequences/Test"):

            f = open(os.path.join('good sequences/Test', filename), 'r')
            lines = f.readlines()
            lines.pop(0)

            for i in range(len(lines)):

                lines[i] = lines[i].strip()
                lines[i] = lines[i].split(' ')
                lines[i].pop(0)
                states_test.append(lines[i][7])
                lines[i].pop(7)

                lines[i] = map(float, lines[i])

            # Extracts and concatenates all the observations
            x_test = np.concatenate([x_test, lines], axis=0)
            lengths_test.append(np.shape(lines)[0])

        x_test = x_test[1:, :]

        # Predicts the sequences of states with the observations and the lengths, using the Viterbi algorithm
        states_pred = []
        y_pred = self.model.predict(x_test, lengths_test)

        # Gives the predicted states their correct label according to our model
        for y in y_pred:
            if self.model.means_[y][2] > 1.4:
                states_pred.append("IA")
            elif self.model.means_[y][4] == 1:
                states_pred.append("DS")
            elif self.model.means_[y][6] == 1:
                states_pred.append("HD")
            elif self.model.means_[y][1] > 500:
                states_pred.append("APP")
            elif self.model.means_[y][5] == 0 and self.model.means_[y][0] < 1600:
                states_pred.append("FA")
            elif self.model.means_[y][5] == 1:
                states_pred.append("CS")


        # Checks if predicted values correspond to our labels and calculates accuracy
        cnt = 0
        for i in range(len(states_test)):
            if states_test[i] == states_pred[i]:
                cnt = cnt + 1

        
        viterbi_accuracy = float(cnt)/float(len(states_test))


        # Predicts states again, now using the Forward algorithm
        states_pred = []
        y_pred = []
        cnt = 0
        for j in range(len(lengths_test)):
            alpha = self.pi
            for i in range(lengths_test[j]):
                alpha = self.iterate(x_test[cnt, :], alpha, i)
                y_pred.append(np.argmax(alpha))
                cnt += 1

        for y in y_pred:
            if self.model.means_[y][2] > 1.4:
                states_pred.append("IA")
            elif self.model.means_[y][4] == 1:
                states_pred.append("DS")
            elif self.model.means_[y][6] == 1:
                states_pred.append("HD")
            elif self.model.means_[y][1] > 500:
                states_pred.append("APP")
            elif self.model.means_[y][5] == 0 and self.model.means_[y][0] < 1600:
                states_pred.append("FA")
            elif self.model.means_[y][5] == 1:
                states_pred.append("CS")

        cnt = 0
        
        # Accuracy with Forward algorithm
        for i in range(len(states_pred)):
            if states_test[i] == states_pred[i]:
                cnt = cnt + 1
        forward_accuracy = float(cnt)/float((len(states_test)))

        return [viterbi_accuracy, forward_accuracy]

    def gaze_detector_1(self, angle):

            # Features:
            # left_gaze is the vector of the gaze direction of the left eye (in the image)
            # right_gaze is the vector of the gaze direction of the right eye (in the image)
            # left_position_1 is the position of the left border of the left eye
            # left_position_2 is the position of the right border of the left eye
            # right_position_1 is the position of the left border of the right eye
            # right_position_2 is the position of the right border of the right eye

            left_gaze_x = self.face_extract.left_gaze_vector[0]
            left_gaze_y = self.face_extract.left_gaze_vector[1]
            right_gaze_x = self.face_extract.right_gaze_vector[0]
            right_gaze_y = self.face_extract.right_gaze_vector[1]
            left_position_x1 = self.face_extract.left_landmarks[0]
            left_position_x2 = self.face_extract.left_landmarks[1]
            left_position_y1 = self.face_extract.left_landmarks[2]
            left_position_y2 = self.face_extract.left_landmarks[3]
            right_position_x1 = self.face_extract.right_landmarks[0]
            right_position_x2 = self.face_extract.right_landmarks[1]
            right_position_y1 = self.face_extract.right_landmarks[2]
            right_position_y2 = self.face_extract.right_landmarks[3]

            T = self.face_extract.poses_T

            # Averaging these features to get position of center of the eyes
            left_position_x = (left_position_x1 + left_position_x2)/2
            left_position_y = (left_position_y1 + left_position_y2)/2
            right_position_x = (right_position_x1 + right_position_x2)/2
            right_position_y = (right_position_y1 + right_position_y2)/2

            # Adding the gaze vectors to the position of the eyes will result in the gaze point in the camera plane
            left_gazepoint = np.array(
                [left_gaze_x+left_position_x, left_gaze_y+left_position_y])
            right_gazepoint = np.array(
                [right_gaze_x+right_position_x, right_gaze_y+right_position_y])

            # Using the center point between the 2 eyes as the gaze point
            gaze_point = (left_gazepoint + right_gazepoint)/2
            distance = np.linalg.norm(
                [T[0]-gaze_point[0], T[1]-gaze_point[1], T[2]-gaze_point[2]])

            # Find radius of base of the cone related to the sight of the person
            radius = distance * tan(radians(angle/2))

            # Find distance between camera (0,0) and gaze point
            dist_to_camera = np.linalg.norm(gaze_point)

            # If the camera is inside this circle (dist < radius), we consider that the person is gazing at the camera.
            # returns a ratio where: <1 is inside the circle, >1 is outside and 0 is direct gaze
            return dist_to_camera/radius

    def gaze_detector_2(self, angle):

        # R = array 1x3 with orientation angles of the face [Rx, Ry, Rz]
        # T = array 1x3 with position of the face [Tx,Ty,Tz]

        Rx = self.face_extract.poses_R[0]
        Ry = self.face_extract.poses_R[1]
        Rz = self.face_extract.poses_R[2]
        Tx = self.face_extract.poses_T[0]
        Ty = self.face_extract.poses_T[1]
        Tz = self.face_extract.poses_T[2]
        T = [Tx, Ty, Tz]

        # this point (x,y) is where the face is pointing at, in the plane of the camera
        y_point = abs(Tx)*tan(Rz) - Ty
        z_point = abs(Tx)*tan(Ry) + Tz
        gaze_point = [0, y_point, z_point]

        distance = np.linalg.norm(T-gaze_point)
        # same procedure as gaze_1
        dist_to_face = np.linalg.norm(gaze_point - self.face_point)

        radius = distance * tan(radians(angle/2))
        return (dist_to_face/radius)

    def gaze_detector(self, angle, change_threshold):

    # Chooses detector 1 if distance is smaller than a threshold and detector 2 if bigger
        if (self.distance_detector() < change_threshold and self.face_extract.left_gaze_vector != []):
            x = self.gaze_detector_1(angle)
            return x
        else:
            x = self.gaze_detector_2(angle)
            return x

    def smile_detector(self):
        # Detects smile of people with their action units
        x = (self.face_extract.action_units[0] +
             self.face_extract.action_units[1])/2
        return x

    def distance_detector(self):
        # Calculates the distance which target person is away in the (x,y) plane (ignores height)
        x = np.linalg.norm([self.face_extract.poses_T[0],self.face_extract.poses_T[2]])
        return x

    def speed_detector(self, pos_r, prev_pos_p, prev_t):
        # Calculates speed of target person, comparing robot's position with person's position in the last observation and in the present.
    
        distance = self.distance_detector()
        t = time.time()

        prev_distance = np.linalg.norm([pos_r - prev_pos_p])
        speed = (prev_distance - distance)/(t - prev_t)

        return speed

    def iterate(self, observation, alpha, num):

        min_prob = 1e-100 # To avoid impossible states given a vector of observations

        y = np.ones(np.shape(self.model.means_)[0]) # variable which will contain probability of each state given set of observations (1x6)
    
        for i in range(np.shape(self.model.means_)[0]): #for each state (6)
            for j in range(np.shape(self.model.means_)[1]): #for each observation (7)
                prob = stats.norm.pdf(observation[j],self.model.means_[i,j], sqrt(self.model.covars_[i,j,j]))
                y[i] = prob * y[i] # this value of y is the combined probability of every feature belonging in its Gaussian distribution for this state
                        
        
        y = y/sum(y)  # Normalizing the values, so that probabilities' sum is 1
        for i in range(len(y)):
            if y[i]<min_prob:
                y[i] = min_prob
        y = y/sum(y)


        D = np.diagflat(y)

        # In case it's the first iteration, we need to use pi matrix for the state probabilities
        if (num == 0):
            alpha = D.dot(self.pi)

        else:
            A_T = self.model.transmat_.transpose()
            tmp = A_T.dot(alpha)
            alpha = D.dot(tmp)
        

        alpha = alpha/np.sum(alpha) # (1x6) vector giving probabilities of each state to be the next given a set of observations
        return alpha
    
    def find_true_state(self,state):

        if self.model.means_[state][2] > 1.4:
            new_state = 1
        elif self.model.means_[state][4] == 1:
            new_state = 2
        elif self.model.means_[state][6] == 1:
            new_state = 3
        elif self.model.means_[state][1] > 500:
            new_state = 4
        elif self.model.means_[state][5] == 0 and self.model.means_[state][0] < 1600:
            new_state = 5
        elif self.model.means_[state][5] == 1:
            new_state = 6
        else:
            new_state = 0

        return new_state

    def keys_thread(self):
        lock = threading.Lock()
        while True:
            with lock:
                self.key = raw_input()

    def callback(self, ros_data):
        

        t = time.time()
        
        self.face_extract = ros_data
        #self.robot_pose = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
        
        distance = self.distance_detector()
        gaze = self.gaze_detector_2(60)
        smile = self.smile_detector()
        speed = 0

        # For speed calculation
        # robot_position = self.robot_pose.pose.pose.position
        # robot_position = np.array([robot_position.x,robot_position.y])*1000
        # robot_orientation = self.robot_pose.pose.pose.orientation
        # robot_orientation = [robot_orientation.x,robot_orientation.y,robot_orientation.z,robot_orientation.w]
        
        # euler_angles = tf.transformations.euler_from_quaternion(robot_orientation,axes='sxyz')
        # yaw = euler_angles[2]
        # R_matrix = np.array([[cos(yaw),-sin(yaw)],[sin(yaw),cos(yaw)]])
        # person_position = [self.face_extract.poses_T[2], self.face_extract.poses_T[0]]

        # person_position_world = R_matrix.dot(person_position) + robot_position

        # if self.prev_time == 0:
        #     speed = 0

        # else:
        #     speed = self.speed_detector(robot_position, self.prev_person_position, self.prev_time)
        
        # self.prev_time = t
        # self.prev_person_position = person_position_world


        obs = [distance, speed, gaze, smile, self.movements[0],self.movements[1],self.movements[2]]

        # Most probable state calculation

        self.alpha = self.iterate(obs, self.alpha, self.iteration)
        state = np.argmax(self.alpha)
        new_state = self.find_true_state(state)
        #self.state_pub.publish(new_state)
     
        print(obs)
        print(new_state)
        if (self.count == 2):
            self.movements[0]=self.movements[2] = 0
            
        if (self.count == 3):
            self.movements[1] = 0
            
        
        if self.key == '1':
            self.movements[0] = 1
            self.count = 0
            
        if self.key == '2':
            self.movements[1] = 1
            self.count = 0
        if self.key == '3':
            self.movements[2] = 1
            self.count = 0

        self.count += 1
        self.key = ''

        self.iteration += 1

        # Waits 0.2 seconds (time interval between observations)
        time.sleep(0.2)


           

if __name__ == '__main__':
    rospy.init_node('hmm')
    server1 = FaceExtractor()
    #server2 = HMModel()
    rospy.spin()
