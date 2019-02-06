# !/usr/bin/env python

from flask import Flask
from flask import request
from threading import Lock

myLock = Lock()
instructions = 1
app = Flask(__name__)

@app.route('/')
def home():
    return "Home"

@app.route('/getinstructions')
def get_instructions():
    with myLock:
        return str(instructions)
@app.route('/setinstructions')
def send_instructions():
    global instructions
    with myLock:
        instructions = request.args['inst']
        return "Instructions set to " + instructions

dictionary = None
@app.route('/getval')
def getval():
    key = str(request.args['key'])
    if key in dictionary:
        return dictionary[key]
    else:
        return "N/A"

def run(d):
    global dictionary
    dictionary = d
    run()

def run():
    app.run(host='0.0.0.0')

if __name__=="__main__":
    print "running"
    run()
