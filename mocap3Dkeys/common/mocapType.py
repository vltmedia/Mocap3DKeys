
# make an enum for the different types of mocap data
# this is used to determine which mocap data to load

mocapTypes = { 
"BODY": 0,
"FACE": 1,
"HAND": 2,
"FEET": 3,
"CAR": 4,
"PERSON": 5,
"PET": 6,
"DOG": 7,
"CAT": 8,
"BIRD": 9,
"OBJECT": 10,
}    

def GetMocapTypeIndexes(names = ["BODY"]):
    indexes = []
    for name in names:
        indexes.append(mocapTypes[name])
    return indexes

def GetMocapTypes(names = ["BODY"]):
    types = []
    for name in names:
        types.append(mocapTypes[name])
    return types


def GetMocapType(name):
    return mocapTypes[name]

def GetMocapTypeByIndex(index):
    for key, value in mocapTypes.items():
        if value == index:
            return key
    return None

