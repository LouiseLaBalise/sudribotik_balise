#!/usr/bin/env python3
import os
import sys
import rospy
import sqlite3
from balise_msgs.msg import ArrayPositionPx, ArrayPositionPxWithType

FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)

sys.path.insert(1, FILE_PATH.split("_2_ROS")[0]) #add parent folder to python path
from init import init_database
MAX_DATA_COUNT = 50 #max of rows alowed in a table
DATABASE_PATH = init_database.FILE_PATH_FOR_DATABASE #database path




"""
Insert data received into its appropriate table.
    table_name (str)    ->      table name.
    data (dict)         ->      data to insert with key as column and value as data.
"""
def insertDataIntoDatabase(table_name:str, data:dict):

    #Get keys
    keys = ','.join(data.keys())
    #Get question marks based on number of data entries
    question_marks = ','.join(list("?"*len(data)))
    #Get values
    values = tuple(data.values())

    #Connect to database
    with sqlite3.connect(DATABASE_PATH) as connection:
        cursor = connection.cursor()

        try:

            #Count the number of row already in the table
            cursor.execute(f"SELECT COUNT(*) FROM {table_name}")
            nb_lines = cursor.fetchone()[0]

            #Delete exceeding lines
            if nb_lines >= MAX_DATA_COUNT:
                rows_to_delete = nb_lines - MAX_DATA_COUNT + 1
                cursor.execute(f"DELETE FROM {table_name} WHERE id IN (SELECT id FROM {table_name} ORDER BY id LIMIT ?)", (rows_to_delete,))

            #Inserting data into the table
            cursor.execute(f"INSERT INTO {table_name} ({keys}) VALUES ({question_marks})", values)
            connection.commit()
        
        except sqlite3.Error as e:
            print(e)
            print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}")




"""
Callback for main robots position.
Insert data in a database.
"""
def robotPosCallback(data):
    data_dict = {}

    #Fill the dict with robot data 
    for robot in data.array_of_positionspx_with_type:
        data_dict["type"] = robot.type
        data_dict["position_x"] = robot.x
        data_dict["position_y"] = robot.y
        data_dict["position_theta"] = robot.theta
    
    insertDataIntoDatabase("robots", data_dict)



"""
Callback for pamis position.
Insert data in a database.
"""
def pamiPosCallback(data):
    data_dict = {}

    #Fill the dict with pami data 
    for pami in data.array_of_positionspx_with_type:
        typ,num = pami.type.split('_')
        data_dict["type"] = typ
        data_dict["num"] = int(num)
        data_dict["position_x"] = pami.x
        data_dict["position_y"] = pami.y
        data_dict["position_theta"] = pami.theta
    
    insertDataIntoDatabase("pamis", data_dict)



"""
Callback for plants position.
Insert data in a database.
"""
def plantPosCallback(data):
    data_dict = {}

    #Fill the dict with plant data 
    for plant in data.array_of_positionspx:
        data_dict["position_x"] = plant.x
        data_dict["position_y"] = plant.y
        data_dict["position_theta"] = plant.theta
    
    insertDataIntoDatabase("plants", data_dict)
    

"""
Callback for solar panel orientation
"""
def solarpanelPosCallback(data):
    data_dict = {}

    #Fill the dict with solar panel data 
    for solarpanel in data.array_of_positionspx_with_type:
        typ,num = solarpanel.type.split('_')
        data_dict["type"] = typ
        data_dict["num"] = int(num)
        data_dict["position_x"] = solarpanel.x
        data_dict["position_y"] = solarpanel.y
        data_dict["position_theta"] = solarpanel.theta
    
    insertDataIntoDatabase("pamis", data_dict)
    

"""
Fetch and parse data from ros on ihm sub topics
"""
def subscriber():

    
    # Tell node name to rospy
    rospy.init_node('listener', anonymous=True)

    # This node will listen to these topics
    rospy.Subscriber("balise/position/robots", ArrayPositionPxWithType, robotPosCallback)
    rospy.Subscriber("balise/position/pamis", ArrayPositionPxWithType, pamiPosCallback)
    rospy.Subscriber("balise/position/plants", ArrayPositionPx, plantPosCallback)
    rospy.Subscriber("balise/position/solarpanel", ArrayPositionPxWithType, solarpanelPosCallback)
    #rospy.Subscriber("balise/position/pots", ArrayPositionPx, callback)

    # Keeps python from exiting until this node is stopped
    # also it permits to this node to listen to new messages on mentioned topics
    # and to run specified callbacks
    rospy.spin()



if __name__ == '__main__':
    subscriber()