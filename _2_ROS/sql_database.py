# sql_database.py

import sqlite3

def create_coordinates_table():
    conn = sqlite3.connect('your_database.db')  # Replace 'your_database.db' with your actual database name
    cursor = conn.cursor()

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS received_coordinates (
            id INTEGER PRIMARY KEY,
            object_id INTEGER,
            x REAL,
            y REAL,
            z REAL
        )
    ''')

    conn.commit()
    conn.close()

def insert_received_coordinates(object_id, coordinates):
    conn = sqlite3.connect('your_database.db')  # Replace 'your_database.db' with your actual database name
    cursor = conn.cursor()

    cursor.execute('''
        INSERT INTO received_coordinates (object_id, x, y, z)
        VALUES (?, ?, ?, ?)
    ''', (object_id, coordinates[0], coordinates[1], coordinates[2]))

    conn.commit()
    conn.close()

if __name__ == '__main__':
    create_coordinates_table()
