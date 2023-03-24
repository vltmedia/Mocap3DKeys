
import json, os
from mocap3Dkeys.mocap3DKeys import Mocap3DKeys
file = "mock\outMocapData.json"

# read json file
mocap3dKeys = Mocap3DKeys.LoadJSONAsMocap3DKeysObject(file)
mocap3dKeys.generateJointRotations()
v = 0