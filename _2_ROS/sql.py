import sqlite3

def create_table():
    conn = sqlite3.connect('database.db')  # Replace 'database.db' with your desired database name
    cursor = conn.cursor()

    cursor.execute('''
        CREATE TABLE IF NOT EXISTS object_coordinates (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            x REAL,
            y REAL,
            z REAL
        )
    ''')

    conn.commit()
    conn.close()

def insert_coordinates(coordinates):
    conn = sqlite3.connect('your_database.db')  # Replace 'your_database.db' with your actual database name
    cursor = conn.cursor()

    cursor.execute('INSERT INTO object_coordinates (x, y, z) VALUES (?, ?, ?)', coordinates)

    conn.commit()
    conn.close()

if __name__ == '__main__':
    # Example coordinates, replace with your actual data
    object_coordinates = (1.23, 4.56, 7.89)

    create_table()
    insert_coordinates(object_coordinates)