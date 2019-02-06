# !/usr/bin/env python

from flask import Flask
from flask import request

forward = True

app = Flask(__name__)

@app.route('/test')
def home():
    return "Server is working!"

instructions = "forward"
@app.route('/getinstructions')
def get_instructions():
    return instructions
@app.route('/sendinstructions')
def send_instructions():
    global instructions
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