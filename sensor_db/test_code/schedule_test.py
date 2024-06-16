# Input data
rooms_data = [['Room1', 70, 33], ['Room2', 60, 52], ['Room3', 50, 16], ['Room4', 70, 45]]

# Extracting rooms with their predicted fine dust levels and their thresholds
rooms = []
for room in rooms_data:
    room_name, threshold, fine_dust_level = room
    # Calculating the criteria value
    criteria_value = 50 - 20 * (threshold / 100)
    rooms.append((room_name, threshold, fine_dust_level, criteria_value))

# Sorting and determining the order of ventilation
first_to_ventilate = []
last_to_ventilate = []
other_rooms = []

for room in rooms:
    room_name, _, fine_dust_level, criteria_value = room
    if fine_dust_level > 51:
        first_to_ventilate.append(room)
    elif fine_dust_level < 30:
        last_to_ventilate.append(room)
    else:
        other_rooms.append(room)

# Sort the rooms that need immediate ventilation by name
first_to_ventilate.sort(key=lambda x: x[0])
last_to_ventilate.sort(key=lambda x: x[0])

# For the other rooms, prioritize based on their criteria
other_rooms.sort(key=lambda x: (-1 if x[2] > x[3] else 1, x[0]))

# Combine the lists to get the final order
final_order = first_to_ventilate + other_rooms + last_to_ventilate

# Extracting just the room names for the final order
final_order_names = [room[0] for room in final_order]
print(final_order_names)
