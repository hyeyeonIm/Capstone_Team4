"""
This script calculates the order in which rooms should be purified based on their predicted fine dust levels.
The input data is a list of rooms, where each room is represented by a list containing:
- Room name (str)
- Threshold value (int)
- Predicted fine dust level (int)

The purification order is determined as follows:
1. Rooms with a predicted fine dust level greater than 51 are prioritized first.
2. Rooms with a predicted fine dust level less than 30 are prioritized last.
3. Rooms with a predicted fine dust level between 31 and 50 are prioritized based on a calculated criteria value.
   - The criteria value is calculated as: 50 - 20 * (threshold / 100)
   - Rooms where the predicted fine dust level is greater than the criteria value are prioritized higher.
4. Rooms with the same priority are sorted alphabetically by their names.

The final output is a list of room names in the order they should be purified.
"""
# Test data
# rooms_data = [['Room1', 70, 33], ['Room2', 60, 52], ['Room3', 50, 16], ['Room4', 70, 45]]


def calculate_purification_order(rooms_data):
    # Extracting rooms with their predicted fine dust levels and their thresholds
    rooms = []
    for room in rooms_data:
        room_name, threshold, fine_dust_level = room
        # Calculating the criteria value
        criteria_value = 50 - 20 * (threshold / 100)
        rooms.append((room_name, threshold, fine_dust_level, criteria_value))

    # Sorting and determining the order of purification
    first_to_purify = []
    last_to_purify = []
    other_rooms = []

    for room in rooms:
        room_name, _, fine_dust_level, criteria_value = room
        if fine_dust_level > 51:
            first_to_purify.append(room)
        elif fine_dust_level < 30:
            last_to_purify.append(room)
        else:
            other_rooms.append(room)

    # Sort the rooms that need immediate purification by name
    first_to_purify.sort(key=lambda x: x[0])
    last_to_purify.sort(key=lambda x: x[0])

    # For the other rooms, prioritize based on their criteria
    other_rooms.sort(key=lambda x: (-1 if x[2] > x[3] else 1, x[0]))
    # x[2] is the predicted fine dust level
    # x[3] is the criteria value
    # x[0] is the room name

    # Combine the lists to get the final order
    final_order = first_to_purify + other_rooms + last_to_purify

    # Extracting just the room names for the final order
    final_order_names = [room[0] for room in final_order]
    return final_order_names
