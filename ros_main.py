#!/usr/bin/env python

# --- ROS :
import rospy
from geometry_msgs.msg import Vector3
from balise.msg import Obj
from balise.msg import ObjArray

# --- Global Libraries :
import time
import sqlite3

# --- Personal Files :
# Format to respect [(id, x, y, theta)]
objets = [(1, 2000, 3000, 180)]
plants = [(15, 20, 30, 40)]

def talker():
    # Initialize the ROS Node which will publish data
    rospy.init_node('b_camera')

    # Initialize the publishers
    objetsPub = rospy.Publisher('balise/position/objets', ObjArray, queue_size=10)
    robotsPub = rospy.Publisher('balise/position/robots', ObjArray, queue_size=10)
    plantsPub = rospy.Publisher('balise/position/plants', ObjArray, queue_size=10)
    rate = rospy.Rate(10)  # rate put to 1Hz, which means it runs the code every 1s

    # Connect to the SQLite database
    conn = sqlite3.connect('/home/student/Desktop/Test/src/balise/src/robot_coordinates.db')
    c = conn.cursor()

    # Create tables if they don't exist
    c.execute('''
        CREATE TABLE IF NOT EXISTS Obj (
            ident INTEGER PRIMARY KEY,
            dscript TEXT,
            position_x REAL,
            position_y REAL,
            position_z REAL,
            theta REAL
        )
''')

    # Main loop :
    while not rospy.is_shutdown():
        # --- 4) Publish the information on ROS
        lObjets = ObjArray()
        lRobots = ObjArray()
        lPlants = ObjArray()

        # Split the objects according to their type
        for val in objets:
            # in the msg, position = (x, y, z=0)
            position = Vector3(val[1], val[2], 0)
            theta = val[3]
            ident = val[0]

            # We'll sort the type of object (robot or SPanel)

            # ID ROBOT (0 et 1)
            if ident < 2:
                if ident == 0:
                    # robot blue team
                    dscript = "rBleu"
                else:
                    # robot yellow team
                    dscript = "rJaune"
                # robot = rBleu, 0, x,y,z, theta
                robot = Obj(dscript, ident, position, theta)
                # we add it in the msg
                lRobots.elements.append(robot)

            # ID SOLAR PANELS (2 to 9)
            else:
                if 2 <= ident <= 9:
                    dscript = "SPanel"
                    objet = Obj(dscript, ident, position, theta)
                    lObjets.elements.append(objet)

            c.execute('''
                INSERT OR REPLACE INTO Obj (dscript, ident, position_x, position_y, position_z, theta)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (dscript, ident, position.x, position.y, position.z, theta))
        conn.commit()

        for val in plants:
            ident = val[0]
            position = Vector3(val[1], val[2], 0)
            theta = val[3]
            dscript = "Plant"

            #ID PLANTS (10 to 36)
            if 10 <= ident <= 36:
                plante = Obj(dscript, ident, position, theta)
                lPlants.elements.append(plante)

            c.execute('''
                INSERT OR REPLACE INTO Obj (dscript, ident, position_x, position_y, position_z, theta)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (dscript, ident, position.x, position.y, position.z, theta))
        conn.commit()

        # Publish the object elements on the publishers created beforehand
        objetsPub.publish(lObjets)
        robotsPub.publish(lRobots)
        plantsPub.publish(lPlants)

        rate.sleep()

    # Close the database connection when done
    conn.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
