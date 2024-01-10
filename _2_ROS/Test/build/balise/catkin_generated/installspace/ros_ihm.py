#!/usr/bin/env python3

# --- ROS :
import rospy
from geometry_msgs.msg import Vector3
from balise.msg import Obj
from balise.msg import ObjArray

# --- Global Libraries :
import time
import math
import os

# --- SQLite :
import sqlite3

# Define the path to the SQLite database file
database_path = '/home/student/Desktop/Test/src/balise/src/robot_coordinates.db'

# Create the directory if it doesn't exist
os.makedirs(os.path.dirname(database_path), exist_ok=True)

# Initialize the database connection
try:
    mydb = sqlite3.connect(database_path)
    cursor = mydb.cursor()

    # Create tables if they don't exist
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS objets (
            id INTEGER PRIMARY KEY,
            time TIMESTAMP,
            E1_pos_X INTEGER,
            E1_pos_Y INTEGER,
            E1_type TEXT,
            E2_pos_X INTEGER,
            -- ... add other columns as needed
            E36_type TEXT
        )
    ''')

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS robots (
            id INTEGER PRIMARY KEY,
            time TIMESTAMP,
            R1_pos_X INTEGER,
            R1_pos_Y INTEGER,
            R1_pos_theta INTEGER,
            R1_type TEXT,
            R2_pos_X INTEGER,
            -- ... add other columns as needed
            R36_type TEXT
        )
    ''')

    mydb.commit()

except sqlite3.Error as e:
    print(f"SQLite Error: {e}")

obj_count = 1
rob_count = 1

# Initialize the ROS Node from which the subscribers read data
rospy.init_node('b_IHM')

# Initialize the callback functions
def update_database(data, table_name, id_prefix, position_prefix):
    global cursor, mydb
    i = 1
    for obj in data.elements:
        query = f"UPDATE `{table_name}` SET `time` = CURRENT_TIMESTAMP, `{id_prefix}{i}_pos_X` = {math.floor(obj.position.x/10)}, `{id_prefix}{i}_pos_Y` = {math.floor(obj.position.y/10)}, `{id_prefix}{i}_type` = \"{obj.dscript}\" WHERE `{table_name}`.`id` = {obj_count}"
        print(query)
        try:
            cursor.execute(query)
        except sqlite3.Error as e:
            print(f"SQLite Error: {e}")
        i += 1

    for j in range(i, 37):
        query = f"UPDATE `{table_name}` SET `time` = CURRENT_TIMESTAMP, `{id_prefix}{j}_pos_X` = 0, `{id_prefix}{j}_pos_Y` = 0, `{id_prefix}{j}_type` = \"\" WHERE `{table_name}`.`id` = {j}"
        print(query)
        try:
            cursor.execute(query)
        except sqlite3.Error as e:
            print(f"SQLite Error: {e}")

def objet_callback(data):
    global obj_count
    update_database(data, 'objets', 'E', 'E')
    obj_count += 1 if obj_count < 5 else -4

def robot_callback(data):
    global rob_count
    update_database(data, 'robots', 'R', 'R')
    rob_count += 1 if rob_count < 5 else -4

def plants_callback(data):
    global cursor, mydb
    i = 1
    for obj in data.elements:
        query = f"UPDATE `plants` SET `time` = CURRENT_TIMESTAMP, `E{i}_pos_X` = {math.floor(obj.position.x/10)}, `E{i}_pos_Y` = {math.floor(obj.position.y/10)}, `E{i}_type` = \"{obj.dscript}\" WHERE `plants`.`id` = {obj_count}"
        print(query)
        try:
            cursor.execute(query)
        except sqlite3.Error as e:
            print(f"SQLite Error: {e}")
        i += 1

    for j in range(i, 37):
        query = f"UPDATE `plants` SET `time` = CURRENT_TIMESTAMP, `E{j}_pos_X` = 0, `E{j}_pos_Y` = 0, `E{j}_type` = \"\" WHERE `plants`.`id` = {j}"
        print(query)
        try:
            cursor.execute(query)
        except sqlite3.Error as e:
            print(f"SQLite Error: {e}")

# Initialize the subscribers
rospy.Subscriber('balise/position/robots', ObjArray, robot_callback)
rospy.Subscriber('balise/position/objets', ObjArray, objet_callback)
rospy.Subscriber('balise/position/plants', ObjArray, plants_callback)

# Spin to keep the script running and processing callbacks
rospy.spin()
