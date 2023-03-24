from mocap3Dkeys.mocap3Dconstants import *
from mocap3Dkeys.common.transform import *

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
        
    def Set3DKeys(self, data = []):
        self.keypoints_3d = []
        self.Add3DKeys(data)
        
            
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
                        kpts_dict[key] = kpts['keypoints_3d'][k_index]
                    except:
                        kpts_dict[key] = kpts.keypoints_3d[k_index]
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

    
    
    
    [staticmethod]     
    def LoadJSONAsMocap3DKeysObject( path):
        import json
        with open(path, 'r') as infile:
            data = json.load(infile)
        return Mocap3DKeys(**data)
    
    
        