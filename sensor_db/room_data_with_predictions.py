# This script connects to a MySQL database to retrieve room information from the 'room_setting' table.
# It then adds predicted fine dust concentration values to each room's data.
# The final output is a list of rooms with their respective thresholds and predictions.

import pymysql


def get_room_data():
    # MySQL database connection configuration
    config = {
        'user': 'your_username',
        'password': 'your_password',
        'host': '127.0.0.1',
        'database': 'ROOM',
    }

    try:
        # Connect to the database
        conn = pymysql.connect(**config)
        cursor = conn.cursor()

        # Query to retrieve data
        query = "SELECT name, threshold FROM room_setting"
        cursor.execute(query)

        # Convert the result to a list
        rooms_data = []
        for row in cursor.fetchall():
            rooms_data.append([row[0], row[1]])

        return rooms_data

    except pymysql.MySQLError as err:
        print(f"Error: {err}")
        return None
    finally:
        # Close the connection
        cursor.close()
        conn.close()


def add_predictions(rooms_data, predictions):
    # Add predictions to each room's data
    for i in range(len(rooms_data)):
        if i < len(predictions):
            rooms_data[i].append(predictions[i])
        else:
            rooms_data[i].append(None)  # Add None if there are no predictions
    return rooms_data


def get_rooms_with_predictions(predict):
    # Retrieve room data
    rooms_data = get_room_data()

    if rooms_data:
        # Add predictions to room data
        rooms_data_with_predictions = add_predictions(rooms_data, predict)

        # Return the result
        return rooms_data_with_predictions
    else:
        return None


# Example function call
# predict = [33, 52, 16, 45]
# result = get_rooms_with_predictions(predict)
# if result:
#     for room in result:
#         print(room)
# else:
#     print("Failed to retrieve data.")
