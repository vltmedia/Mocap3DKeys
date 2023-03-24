from mocap3Dkeys.mocap3Dconstants import *
from mocap3Dkeys.common.transform import *
import operator
import numpy as np
import mocap3Dkeys.utils as utils

def ConvertToFloats(data):
    floatItems = []
    for j in range(len(data)):
        floatItems.append(float(data[j]))
    return floatItems
        
class Mocap3DKey:
    def __init__(self, keypoints_3d = [], keypoints = [], joints = [], bbox = [], track_id = 999):
        self.keypoints_3d = keypoints_3d
        self.keypoints = keypoints
        # self.jointsIndex = jointIndex
        self.joints = joints
        self.bbox = bbox
        self.track_id = track_id
        

    def Add3DKeys(self, data = []):
        for i in range(len(data)):
            item = data[i]
            # convert all the data in items to float
            floatItems = item
            # floatItems = ConvertToFloats(item)
            self.keypoints_3d.append(item)

    def Add3DPositionKeys(self, data = []):
        for i in range(len(data)):
            item = data[i]
            # convert all the data in items to float
            floatItems = item
            # floatItems = ConvertToFloats(item)
            self.Add3DTransformKey(position = [item[0], item[1], item[2]])

    def Add3DTransformKey(self, position = [0,0,0], rotation = [0,0,0,0], scale = [1,1,1]):
        self.keypoints_3d.append(KeyTransform(position = position, rotation = rotation, scale = scale))

    def Add3DPositionKey(self, position = [0,0,0]):
        self.Add3DTransformKey(position = position)
        
    def Remove3DKey(self, index):
        self.keypoints_3d.pop(index)
    

        
    def Set3DKeys(self, data = []):
        self.keypoints_3d = []
        self.Add3DKeys(data)
        
    def Set3DPositionKeys(self, data = []):
        self.keypoints_3d = []
        self.Add3DPositionKeys(data)
        
            
    def Add2DKeys(self, data = []):
        self.keypoints = []
        for i in range(len(data)):
            item = data[i]
            floatItems = item
            # floatItems = ConvertToFloats(item)
            self.keypoints.append(KeyTransform(position = [floatItems[0],floatItems[1], floatItems[2]]))
    
    def Set2DKeys(self, data = []):
        self.keypoints = []
        self.Add2DKeys(data)
                
    def Add3DKey(self, data):
        self.keypoints_3d.append(data)
    def Add2DKey(self, data):
        self.keypoints.append(data)
    def AddBBox(self, data):
        for i in range(len(data)):
            item = data[i]
            self.bbox.append(float(item))
        
class Mocap3DKeys:
    # make points based on template
    def __init__(self, keys = [], mocapTypes = ["BODY"], engine = "COCO", version = 1.0, data = [], frames = 0):
        self.frames = frames
        self.keys = keys
        self.engine = engine
        self.version = version
        self.data = data
        self.mocapTypes = mocapTypes
    
    def AddKey(self, key):
        self.keys.append(key)
    
    def AddNewKey(self, keypoints_3d = [], keypoints = [], bbox = [], track_id = 999):
        self.keys.append(Mocap3DKey(keypoints_3d, keypoints,[], bbox, track_id))
        
    def GetKey(self, index):
        return self.keys[index]
    
    def GetKeys(self):
        return self.keys
    
    def GetKeyCount(self):
        return len(self.keys)
    
    def SaveJSON(self, path):
        import json
        # self.convertToDictionary()
        jsdata = json.dumps(self.__dict__,  default=lambda o: o.__dict__, indent=4)
        # print("data", jsdata)
        with open(path, 'w') as outfile:
            outfile.write(jsdata)
    
    def SavePickel(self, path):
        import pickle
        with open(path, 'wb') as outfile:
            pickle.dump(self, outfile)
            
        
    #remove jittery keypoints by applying a median filter along each axis
    def median_filter(self, kpts, window_size = 3):

        import copy
        filtered = copy.deepcopy(kpts)

        from scipy.signal import medfilt


        #apply median filter to get rid of poor keypoints estimations
        for joint in filtered['joints']:
            joint_kpts = filtered[joint]
            xs = joint_kpts[0]
            ys = joint_kpts[1]
            zs = joint_kpts[2]
            # xs = joint_kpts[:,0]
            # ys = joint_kpts[:,1]
            # zs = joint_kpts[:,2]
            xs = medfilt(xs, window_size)
            ys = medfilt(ys, window_size)
            zs = medfilt(zs, window_size)
            filtered[joint] = np.stack([xs, ys, zs], axis = -1)

        return filtered

    def get_bone_lengths(self, kpts):

        """
        We have to define an initial skeleton pose(T pose).
        In this case we need to known the length of each bone.
        Here we calculate the length of each bone from data
        """

        bone_lengths = {}
        for joint in kpts['joints']:
            if joint == 'hips': continue
            parent = kpts['hierarchy'][joint][0]

            joint_kpts = kpts[joint]
            parent_kpts = kpts[parent]

            _bone = joint_kpts - parent_kpts
            _bone_lengths = np.sqrt(np.sum(np.square(_bone), axis = -1))

            _bone_length = np.median(_bone_lengths)
            bone_lengths[joint] = _bone_length

            # plt.hist(bone_lengths, bins = 25)
            # plt.title(joint)
            # plt.show()

        #print(bone_lengths)
        kpts['bone_lengths'] = bone_lengths
        return

    #Here we define the T pose and we normalize the T pose by the length of the hips to neck distance.
    def get_base_skeleton(self, kpts, normalization_bone = 'neck'):

        #this defines a generic skeleton to which we can apply rotations to
        body_lengths = kpts['bone_lengths']

        #define skeleton offset directions
        offset_directions = {}
        offset_directions['lefthip'] = np.array([1,0,0])
        offset_directions['leftknee'] = np.array([0,-1, 0])
        offset_directions['leftfoot'] = np.array([0,-1, 0])

        offset_directions['righthip'] = np.array([-1,0,0])
        offset_directions['rightknee'] = np.array([0,-1, 0])
        offset_directions['rightfoot'] = np.array([0,-1, 0])

        offset_directions['neck'] = np.array([0,1,0])

        offset_directions['leftshoulder'] = np.array([1,0,0])
        offset_directions['leftelbow'] = np.array([1,0,0])
        offset_directions['leftwrist'] = np.array([1,0,0])

        offset_directions['rightshoulder'] = np.array([-1,0,0])
        offset_directions['rightelbow'] = np.array([-1,0,0])
        offset_directions['rightwrist'] = np.array([-1,0,0])

        #set bone normalization length. Set to 1 if you dont want normalization
        normalization = kpts['bone_lengths'][normalization_bone]
        #normalization = 1


        #base skeleton set by multiplying offset directions by measured bone lengths. In this case we use the average of two sided limbs. E.g left and right hip averaged
        base_skeleton = {'hips': np.array([0,0,0])}
        def _set_length(joint_type):
            base_skeleton['left' + joint_type] = offset_directions['left' + joint_type] * ((body_lengths['left' + joint_type] + body_lengths['right' + joint_type])/(2 * normalization))
            base_skeleton['right' + joint_type] = offset_directions['right' + joint_type] * ((body_lengths['left' + joint_type] + body_lengths['right' + joint_type])/(2 * normalization))

        _set_length('hip')
        _set_length('knee')
        _set_length('foot')
        _set_length('shoulder')
        _set_length('elbow')
        _set_length('wrist')
        base_skeleton['neck'] = offset_directions['neck'] * (body_lengths['neck']/normalization)

        kpts['offset_directions'] = offset_directions
        kpts['base_skeleton'] = base_skeleton
        kpts['normalization'] = normalization

        return

    
    def convertToBodyJointsDictionary(self):
        #its easier to manipulate keypoints by joint name
        v = 0
        if "BODY" in self.mocapTypes:
            keypoints_to_index = KeypointsIndexes_COCO
            if self.engine == "MediaPipe":
                keypoints_to_index = KeypointsIndexes_MediaPipe
            for keymaps in self.keys:
                # for item in keymaps['keypoints_3d']:
                kpts = keymaps
                kpts_dict = {}
                for key, k_index in keypoints_to_index.items():
                    try:
                        kpts_dict[key] = kpts['keypoints_3d'][k_index]['position']
                    except:
                        kpts_dict[key] = kpts.keypoints_3d[k_index].position
                    # kpts_dict[key] = kpts['keypoints_3d'][:,k_index]
                kpts_dict['joints'] = list(keypoints_to_index.keys())
                try:
                    kpts['joints']  =kpts_dict
                    # kpts['jointsIndex']  = kpts_dict['joints']
                except:
                    kpts.joints  =kpts_dict
                    kpts.jointsIndex  = kpts_dict['joints']
                # item.joints = kpts_dict['joints']
            v = 0
            
            return kpts_dict
        else:
            return []
        
    #calculate the rotation of the root joint with respect to the world coordinates
    def get_hips_position_and_rotation(self, frame_pos, root_joint = 'hips', root_define_joints = ['lefthip', 'neck']):

        #root position is saved directly
        root_position = frame_pos[root_joint]

        #calculate unit vectors of root joint
        root_u = frame_pos[root_define_joints[0]] - frame_pos[root_joint]
        root_u = root_u/np.sqrt(np.sum(np.square(root_u)))
        root_v = frame_pos[root_define_joints[1]] - frame_pos[root_joint]
        root_v = root_v/np.sqrt(np.sum(np.square(root_v)))
        root_w = np.cross(root_u, root_v)

        #Make the rotation matrix
        C = np.array([root_u, root_v, root_w]).T
        thetaz,thetay, thetax = utils.Decompose_R_ZXY(C)
        root_rotation = np.array([thetaz, thetax, thetay])

        return root_position, root_rotation

    #calculate the rotation matrix and joint angles input joint
    def get_joint_rotations(self, joint_name, joints_hierarchy, joints_offsets, frame_rotations, frame_pos):

        _invR = np.eye(3)
        for i, parent_name in enumerate(joints_hierarchy[joint_name]):
            if i == 0: continue
            _r_angles = frame_rotations[parent_name]
            R = utils.get_R_z(_r_angles[0]) @ utils.get_R_x(_r_angles[1]) @ utils.get_R_y(_r_angles[2])
            _invR = _invR@R.T

        b = _invR @ (frame_pos[joint_name] - frame_pos[joints_hierarchy[joint_name][0]])

        _R = utils.Get_R2(joints_offsets[joint_name], b)
        tz, ty, tx = utils.Decompose_R_ZXY(_R)
        joint_rs = np.array([tz, tx, ty])
        #print(np.degrees(joint_rs))

        return joint_rs

    #helper function that composes a chain of rotation matrices
    def get_rotation_chain(self, joint, hierarchy, frame_rotations):

        hierarchy = hierarchy[::-1]

        #this code assumes ZXY rotation order
        R = np.eye(3)
        for parent in hierarchy:
            angles = frame_rotations[parent]
            _R = utils.get_R_z(angles[0])@utils.get_R_x(angles[1])@utils.get_R_y(angles[2])
            R = R @ _R

        return R    
    #calculate the joint angles frame by frame.
    def calculate_joint_angles(self, kpts):

        #set up emtpy container for joint angles
        for joint in kpts['joints']:
            kpts[joint+'_angles'] = []

        for framenum in range(kpts['hips'].shape[0]):

            #get the keypoints positions in the current frame
            frame_pos = {}
            for joint in kpts['joints']:
                frame_pos[joint] = kpts[joint][framenum]

            root_position, root_rotation = self.get_hips_position_and_rotation(frame_pos)

            frame_rotations = {'hips': root_rotation}

            #center the body pose
            for joint in kpts['joints']:
                frame_pos[joint] = frame_pos[joint] - root_position

            #get the max joints connectsion
            max_connected_joints = 0
            for joint in kpts['joints']:
                if len(kpts['hierarchy'][joint]) > max_connected_joints:
                    max_connected_joints = len(kpts['hierarchy'][joint])

            depth = 2
            while(depth <= max_connected_joints):
                for joint in kpts['joints']:
                    if len(kpts['hierarchy'][joint]) == depth:
                        joint_rs = self.get_joint_rotations(joint, kpts['hierarchy'], kpts['offset_directions'], frame_rotations, frame_pos)
                        parent = kpts['hierarchy'][joint][0]
                        frame_rotations[parent] = joint_rs
                depth += 1

            #for completeness, add zero rotation angles for endpoints. This is not necessary as they are never used.
            for _j in kpts['joints']:
                if _j not in list(frame_rotations.keys()):
                    frame_rotations[_j] = np.array([0.,0.,0.])

            #update dictionary with current angles.
            for joint in kpts['joints']:
                kpts[joint + '_angles'].append(frame_rotations[joint])


        #convert joint angles list to numpy arrays.
        for joint in kpts['joints']:
            kpts[joint+'_angles'] = np.array(kpts[joint + '_angles'])
            #print(joint, kpts[joint+'_angles'].shape)

        return
    
    def add_hips_and_neck(self, kpts):
        #we add two new keypoints which are the mid point between the hips and mid point between the shoulders

        #add hips kpts
        difference = np.array(kpts['lefthip']) - np.array(kpts['righthip'])
        # difference = map(operator.sub, kpts['lefthip'] , kpts['righthip'])
        difference = difference/2
        hips =np.array(kpts['righthip']) + difference
        kpts['hips'] = hips.tolist()
        
        kpts['joints'].append('hips')


        #add neck kpts
        difference = np.array(kpts['leftshoulder']) - np.array(kpts['rightshoulder'])
        difference = difference/2
        neck = np.array(kpts['rightshoulder']) + difference
        kpts['neck'] = neck.tolist()
        kpts['joints'].append('neck')

        #define the hierarchy of the joints
        hierarchy = {'hips': [],
                    'lefthip': ['hips'], 'leftknee': ['lefthip', 'hips'], 'leftfoot': ['leftknee', 'lefthip', 'hips'],
                    'righthip': ['hips'], 'rightknee': ['righthip', 'hips'], 'rightfoot': ['rightknee', 'righthip', 'hips'],
                    'neck': ['hips'],
                    'leftshoulder': ['neck', 'hips'], 'leftelbow': ['leftshoulder', 'neck', 'hips'], 'leftwrist': ['leftelbow', 'leftshoulder', 'neck', 'hips'],
                    'rightshoulder': ['neck', 'hips'], 'rightelbow': ['rightshoulder', 'neck', 'hips'], 'rightwrist': ['rightelbow', 'rightshoulder', 'neck', 'hips']
                    }

        kpts['hierarchy'] = hierarchy
        kpts['root_joint'] = 'hips'

        return kpts    
    def generateJointRotations(self):
        kpts = self.convertToBodyJointsDictionary()
        
        self.add_hips_and_neck(kpts)
        filtered_kpts = self.median_filter(kpts)
        self.get_bone_lengths(filtered_kpts)
        self.get_base_skeleton(filtered_kpts)

        self.calculate_joint_angles(filtered_kpts)
        
        v = 0

    
    
    
    [staticmethod]     
    def LoadJSONAsMocap3DKeysObject( path):
        import json
        with open(path, 'r') as infile:
            data = json.load(infile)
        return Mocap3DKeys(**data)
    
    
        