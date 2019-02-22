from nxturtle import NXTurtle
import math
import nxt

turtle = NXTurtle(connect=True)

# Distance between left and right wheel in [cm] (measured from middle of treads)
AXIS_LENGTH = 12.5

# Wheel diameter in [cm]
WHEEL_DIAMETER = 4.4

# Now we can calculate the number of wheel turns it takes, to move the turtle
# by one unit (i.e. one centimeter)
tachoPerUnit = 360.0 / (WHEEL_DIAMETER * math.pi)
correction = 1.00
turtle.set_tacho_units_per_unit(correction * tachoPerUnit)

# It should also be possible to calculate the number of wheel turns it takes,
# to turn the turtle by one degree
tachoPerDegree = AXIS_LENGTH / WHEEL_DIAMETER

# ... observation shows that Archimedes was wrong, or our turtle is not
# perfect. Either way this correction factor will improve the results:
correction = 0.90

turtle.set_tacho_units_per_degree(correction * tachoPerDegree)

# This function is passed to our turtle. It will be called whenever the pen
# should be raised or put down.

def pen_handler(turtle, on):
    power = 50
    tacho_units = 150
    if on:
        power *= -1
    turtle.penMotor.turn(power, tacho_units)

turtle.set_pen_handler(pen_handler)

### Go an try it

# get set...
turtle.pendown()

turtle.fd(25)
