from mocap3Dkeys.mocap3Dconstants import *
from mocap3Dkeys.common.transform import *

        
class Mocap3DKey:
    def __init__(self, keypoints_3d = [], keypoints = [], joints = [],jointIndex = [], bbox = [], track_id = 999):
        self.keypoints_3d = keypoints_3d
        self.keypoints = keypoints
        self.jointsIndex = jointIndex
        self.joints = joints
        self.bbox = bbox
        self.track_id = track_id
        
    def Add3DKeys(self, data = []):
        for i in range(len(data)):
            item = data[i]
            self.keypoints_3d.append(KeyTransform(position = [item[0], item[1], item[2]]))
    def Add2DKeys(self, data = []):
        for i in range(len(data)):
            item = data[i]
            self.keypoints.append(KeyTransform(position = [item[0], item[1], item[2]]))
        
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
    def __init__(self, keys = [], mocapTypes = ["BODY"], engine = "COCO", version = 1.0, data = []):
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
                    pos = kpts['keypoints_3d']
                    
                    kpts_dict[key] = kpts['keypoints_3d'][k_index]['position']
                    # kpts_dict[key] = kpts['keypoints_3d'][:,k_index]
                kpts_dict['joints'] = list(keypoints_to_index.keys())
                kpts['joints']  =kpts_dict
                kpts['jointsIndex']  = kpts_dict['joints']
                # item.joints = kpts_dict['joints']
            v = 0
            
            return kpts_dict
        else:
            return []

    
    
    
    [staticmethod]     
    def LoadJSONAsMocap3DKeysObject( path):
        import json
        with open(path, 'r') as infile:
            data = json.load(infile)
        return Mocap3DKeys(**data)
    
    
        