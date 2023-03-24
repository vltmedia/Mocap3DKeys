
import json, os
from mocap3Dkeys.mocap3DKeys import Mocap3DKeys, Mocap3DKey
file = "mock\pose_lift_results_vis.json"
data = open(file).read()
jsdata = json.loads(data)

jsdata['keys'][0]['keypoints_3d']

mocap3dKeys = Mocap3DKeys()


for key in jsdata['keys']:
    mocapKey = Mocap3DKey()
    mocapKey.Add3DKeys( key['keypoints_3d'])
    mocap3dKeys.AddKey(mocapKey)
mocap3dKeys.SaveJSON("mock\outMocapData_R.json")
# read json file
v = 0