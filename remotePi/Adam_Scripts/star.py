import time
import serial
import server
import requests
from multiprocessing import Process

DONE = 0 # end of instructions list
TURN_RIGHT = 1  # 90 degrees
TURN_LEFT = 2  # 90 degrees
FOLLOW_FRONT = 3  # until next junction
FOLLOW_RIGHT = 4  # until next junction
FOLLOW_BACK = 5  # until next junction
FOLLOW_LEFT = 6  # until next junction
GRAB = 7
DROP = 8
SET_HEIGHT_GROUND = 13 # for ground level
SET_HEIGHT_LOWER_SHELF = 9  # for lower shelves
SET_HEIGHT_ON_BOX = 10  # for on top of ground level boxes
SET_HEIGHT_UPPER_SHELF = 12  # for upper shelves

# every path starts and ends at JUNCTION on the hand-drawn map

# Starts facing west, ends facing north
JUNCTION_TO_A = [FOLLOW_FRONT, FOLLOW_FRONT, FOLLOW_FRONT, TURN_RIGHT, FOLLOW_FRONT]  # ENDS F
JUNCTION_TO_B = [FOLLOW_FRONT, FOLLOW_FRONT, TURN_RIGHT, FOLLOW_FRONT]
JUNCTION_TO_C = [FOLLOW_FRONT, TURN_RIGHT, FOLLOW_FRONT]

# starts facing north, ends facing east
A_TO_JUNCTION = [FOLLOW_BACK, TURN_RIGHT, FOLLOW_FRONT, FOLLOW_FRONT, FOLLOW_FRONT]
B_TO_JUNCTION = [FOLLOW_BACK, TURN_RIGHT, FOLLOW_FRONT, FOLLOW_FRONT]
C_TO_JUNCTION = [FOLLOW_BACK, TURN_RIGHT, FOLLOW_FRONT]


# starts facing west, ends facing east
PICKUP_FROM_JUNCTION_AND_RETURN = dict()
PICKUP_FROM_JUNCTION_AND_RETURN[1] = [SET_HEIGHT_LOWER_SHELF] + JUNCTION_TO_A + [GRAB] + A_TO_JUNCTION
PICKUP_FROM_JUNCTION_AND_RETURN[2] = [SET_HEIGHT_UPPER_SHELF] + JUNCTION_TO_A + [GRAB] + A_TO_JUNCTION
PICKUP_FROM_JUNCTION_AND_RETURN[3] = [SET_HEIGHT_LOWER_SHELF] + JUNCTION_TO_B + [GRAB] + B_TO_JUNCTION
PICKUP_FROM_JUNCTION_AND_RETURN[4] = [SET_HEIGHT_UPPER_SHELF] + JUNCTION_TO_B + [GRAB] + B_TO_JUNCTION
PICKUP_FROM_JUNCTION_AND_RETURN[5] = [SET_HEIGHT_LOWER_SHELF] + JUNCTION_TO_C + [GRAB] + C_TO_JUNCTION
PICKUP_FROM_JUNCTION_AND_RETURN[6] = [SET_HEIGHT_UPPER_SHELF] + JUNCTION_TO_C + [GRAB] + C_TO_JUNCTION

# starts facing east, ends facing east
JUNCTION_TO_VAN = [FOLLOW_LEFT]
FROM_VAN_PLACE_AND_RETURN = dict()
FROM_VAN_PLACE_AND_RETURN[1] = [SET_HEIGHT_GROUND, FOLLOW_FRONT, FOLLOW_FRONT, DROP, FOLLOW_BACK, FOLLOW_BACK]
FROM_VAN_PLACE_AND_RETURN[2] = [SET_HEIGHT_ON_BOX, FOLLOW_FRONT, FOLLOW_FRONT, DROP, FOLLOW_BACK, FOLLOW_BACK]
FROM_VAN_PLACE_AND_RETURN[3] = [SET_HEIGHT_GROUND, FOLLOW_FRONT, DROP, FOLLOW_BACK]
FROM_VAN_PLACE_AND_RETURN[4] = [SET_HEIGHT_ON_BOX, FOLLOW_FRONT, DROP, FOLLOW_BACK]
FROM_VAN_PLACE_AND_RETURN[5] = [SET_HEIGHT_GROUND, DROP]
FROM_VAN_PLACE_AND_RETURN[6] = [SET_HEIGHT_ON_BOX, DROP]

# starts facing east, ends facing west
VAN_TO_JUNCTION = [FOLLOW_RIGHT, TURN_LEFT, TURN_LEFT]

# starts facing west, ends facing west
STACK = dict()
for i in range(1,7):
    STACK[i] = PICKUP_FROM_JUNCTION_AND_RETURN[i] + JUNCTION_TO_VAN + FROM_VAN_PLACE_AND_RETURN[i] + VAN_TO_JUNCTION
START_TO_JUNCTION = [FOLLOW_FRONT]
JUNCTION_TO_START = [FOLLOW_BACK]


ser = serial.Serial('/dev/ttyACM0')


def formatinstructions(text):
    pspairs = text.split(';')  # package-shelf-pairs
    instructions = []
    instructions = instructions + START_TO_JUNCTION
    for i in range(0, len(pspairs)):
        pair = pspairs[i].split(',')
        package = pair[0]
        shelf = pair[1]
        instructions = instructions + STACK[int(package)]
    instructions = instructions + JUNCTION_TO_START + [DONE]
    return instructions


if __name__ == "__main__":
    process = Process(target=server.run)
    process.start()
    is_executing_plan = False
    time.sleep(5)
    while 1:
        if (not is_executing_plan):
            r = requests.get('http://0.0.0.0:5000/getinstructions')
            if (r.text != "0"):
                instructions = formatinstructions(r.text.strip())
                ser.write(bytearray(instructions))
                is_executing_plan = True
        elif (ser.in_waiting > 0):
            line = ser.readline().strip()
            if (line == "done"):
                is_executing_plan = False
            else:
                requests.get('http://0.0.0.0:5000/addnotification?not=' + line)
