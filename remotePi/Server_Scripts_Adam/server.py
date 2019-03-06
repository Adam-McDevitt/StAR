# !/usr/bin/env python

# HOW TO USE
#
# ROBOT:
# - Poll the URL http://veemon:5000/getinstructions to receive instructions that have been sent by the app. After an
# instruction is received by the robot it is reset, so the next time the URL is accessed it will return
# INSTRUCTION_RESET_VALUE
# - Add notifications to the notification list with http://veemon:5000/addnotification?not=x where x is the notification
# string to be received by the app.
#
# APP:
# - Send instructions to the robot by accessing http://veemon:5000/setinstructions?inst=x where x is the instruction to
# be received by the robot
# - Receive notifications from the robot by polling http://veemon:5000/getnotifications. Notifications will be returned
# as a string, where (if there is more than one notification) notifications are separated by semicolons. After a list of
# notifications has been received, the list will be reset, so the next time the URL is accessed it will return the empty
# string. (The empty string means there are no notifications.)

from flask import Flask
from flask import request

app = Flask(__name__)
INSTRUCTION_RESET_VALUE = "6"
INITIAL_INSTRUCTION = "0"

@app.route('/test')
def home():
    return "Server is working!"

instructions = INITIAL_INSTRUCTION
notifications = []

# URL accessed by robot to receive instructions from the app
# Instructions are reset to RESET_VALUE after being accessed
@app.route('/getinstructions')
def get_instructions():
    global instructions
    ret = instructions
    instructions = INSTRUCTION_RESET_VALUE
    return ret

# URL accessed by app to send instructions to server
@app.route('/setinstructions')
def send_instructions():
    global instructions
    instructions = request.args['inst']
    return "Instructions set to " + instructions

@app.route('/addnotification')
def add_notification():
    global notifications
    notification = request.args['not']
    notifications.append(notification)
    return "Succesfully appended notification \"" + notification + "\" to the list"

@app.route('/getnotifications')
def get_notifications():
    global notifications
    ret = ""
    for notification in notifications:
        ret = ret + notification + ";"
    ret = ret[0:-1] # cuts off last semicolon
    notifications = []
    return ret

# Start the server without the dictionary function
def run():
    app.run(host='0.0.0.0')


if __name__ == "__main__":
    run()
