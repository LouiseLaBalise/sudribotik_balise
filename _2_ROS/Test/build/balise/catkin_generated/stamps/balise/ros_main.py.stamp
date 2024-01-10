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
path = '/home/student/Desktop/Test/src/balise/src/coordinates.db'

# Format to respect [(id, x, y, theta)]
objets = [(2, 2000, 3000, 180), (8, 100, 400, 90), (12, 1000, 4000, 0), (88, 0, 0, 10)]
plants = [(36, 20, 30, 40), (36, 25, 30, 59), (13, 30, 100, 0), (0, 0, 45, 0)]

def talker():
    # Initialize the ROS Node which will publish data
    rospy.init_node('b_camera')

    # Initialize the publishers
    objetsPub = rospy.Publisher('balise/position/objets', ObjArray, queue_size=10)
    robotsPub = rospy.Publisher('balise/position/robots', ObjArray, queue_size=10)
    plantsPub = rospy.Publisher('balise/position/plants', ObjArray, queue_size=10)
    rate = rospy.Rate(10)  # rate put to 1Hz, which means it runs the code every 1s

    # Connect to the SQLite database
    conn = sqlite3.connect(path)
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
    c.execute('''
        CREATE TABLE IF NOT EXISTS Plants (
            id INTEGER PRIMARY KEY,
            ident INTEGER,
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

            # We'll sort the type of object (Robots or Elements)

            # ID ROBOT (Aruco ID between 1 and 10)
            if ident <= 10:

                if ident >= 1 and ident <= 5:
                    # robot blue team
                    dscript = "rBleu"
                
                elif ident>=6 and ident <= 10:
                    # robot yellow team
                    dscript = "rJaune"
                
                # robot = rBleu, 0, x,y,z, theta
                robot = Obj(dscript, ident, position, theta)
                # we add it in the msg
                lRobots.elements.append(robot)

            # ID ELEMENTS (Aruco ID between 11 and 50)
            elif ident >= 11 and ident <= 50 :
                dscript = "Elements"
                objet = Obj(dscript, ident, position, theta)
                lObjets.elements.append(objet)
            
            # ID USAGES (Aruco ID between 51 and beyond)
            else:
                if ident >= 51 and ident <= 70 :
                    dscript = "usageBleu"
                elif ident >= 71 and ident <= 90 :
                    dscript = "usageJaune"
                else:
                    dscript = "usageMix"
                
                objet = Obj(dscript, ident, position, theta)
                lObjets.elements.append(objet)

            c.execute('''
                INSERT OR REPLACE INTO Obj (dscript, ident, position_x, position_y, position_z, theta)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (dscript, ident, position.x, position.y, position.z, theta))
        conn.commit()

        for val in plants:
            
            position = Vector3(val[1], val[2], 0)
            theta = val[3]
            ident = val[0]

            if ident == 36 :
                dscript = "Plantefragile"
            elif ident == 13:
                dscript = "Planteresistante"
            else:
                dscript = "Plante"
            
            plante = Obj(dscript, ident, position, theta)
            lPlants.elements.append(plante)

            c.execute('''
                INSERT OR REPLACE INTO Plants (dscript, ident, position_x, position_y, position_z, theta)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (dscript, ident, position.x, position.y, position.z, theta))
        conn.commit()

        # Publish the object elements on the publishers created beforehand
        objetsPub.publish(lObjets)
        robotsPub.publish(lRobots)
        plantsPub.publish(lPlants)

         # Wait for one second
        time.sleep(1)

        # Clear the Plants table
        c.execute('DELETE FROM Plants')
        conn.commit()

        rate.sleep()

    # Close the database connection when done
    conn.close()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
