import json

# CONFIGURATION
world_width = 60
world_height = 50

rows = 2
aisles_per_row = 8

aisle_width = 2.5
shelf_depth = 1.2
shelf_length = 16.75

row_gap = 4
start_x = 0
start_y = 0


# GENERATION
shelves = []
aisles = []

aisle_counter = 1

for r in range(rows):

    row_y = start_y + r * (shelf_length + row_gap)
    current_x = start_x

    for a in range(aisles_per_row):

        # left shelf
        shelves.append({
            "polygon": [
                [current_x, row_y],
                [current_x + shelf_depth, row_y],
                [current_x + shelf_depth, row_y + shelf_length],
                [current_x, row_y + shelf_length]
            ]
        })

        # right shelf
        right_x = current_x + shelf_depth + aisle_width

        shelves.append({
            "polygon": [
                [right_x, row_y],
                [right_x + shelf_depth, row_y],
                [right_x + shelf_depth, row_y + shelf_length],
                [right_x, row_y + shelf_length]
            ]
        })

        aisle_center = current_x + shelf_depth + (aisle_width / 2)

        aisles.append({
            "label": f"A{aisle_counter}",
            "x": round(aisle_center, 3),
            "row": r
        })

        aisle_counter += 1

        current_x += shelf_depth + aisle_width + shelf_depth


layout = {
    "world": {
        "width": world_width,
        "height": world_height
    },
    "rows": rows,
    "aisles_per_row": aisles_per_row,
    "shelf_length": shelf_length,
    "row_gap": row_gap,
    "start_y": start_y,
    "shelves": shelves,
    "aisles": aisles
}

with open("store_layout.json", "w") as f:
    json.dump(layout, f, indent=2)

print("store_layout.json generated")