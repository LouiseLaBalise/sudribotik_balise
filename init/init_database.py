import os
import traceback
import sqlite3

"""
This file delete then create and initalize an empty database for a match at the root of the project.
There is currently 5 tables initialized :
        robots
        pamis
        plants
        solarpanel
        score
        
You can add more following the gloabl pattern.
"""


FILE_PATH = os.path.abspath(__file__)
FILE_NAME = os.path.basename(FILE_PATH)
FILE_PATH_FOR_DATABASE = FILE_PATH.split("init")[0]+"match_data.db" #database full path

#Delete previous one if it exists
if os.path.isfile(FILE_PATH_FOR_DATABASE):
    os.remove(FILE_PATH_FOR_DATABASE)

#Connect to database and close it when <with> block is exited
with sqlite3.connect(FILE_PATH_FOR_DATABASE) as connection:
    cursor = connection.cursor()

    try :
        #Create these tables if they dont exist
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_robots
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    type TEXT,
                    position_x INTEGER,
                    position_y INTEGER,
                    position_theta INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_pamis
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    type TEXT,
                    num INTEGER,
                    position_x INTEGER,
                    position_y INTEGER,
                    position_theta INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_plants
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    position_x INTEGER,
                    position_y INTEGER,
                    position_theta INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_solarpanel
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    type TEXT,
                    num INTEGER,
                    position_x INTEGER,
                    position_y INTEGER,
                    position_theta INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_pots
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    position_x INTEGER,
                    position_y INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS b_score
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    points INTEGER
                    )
                    ''')
        cursor.execute(''' CREATE TABLE IF NOT EXISTS r_aruco
                    (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    tag INTEGER,
                    ax INTEGER,
                    ay INTEGER,
                    bx INTEGER,
                    by INTEGER,
                    cx INTEGER,
                    cy INTEGER,
                    dx INTEGER,
                    dy INTEGER
                    )
                    ''')
        connection.commit()


    except sqlite3.Error as e:
        traceback_str = traceback.format_exc()
        print(f"Log [{os.times().elapsed}] - {FILE_NAME} : {e}\n{traceback_str}")

