import json
import os

FIELD_LENGTH = 16.54175
FIELD_WIDTH = 8.0137
DIRECTORY = "./src/main/deploy/pathplanner"


def generate_red_aliance(file_name: str, directory: str):
    waypoints = []

    path_file = open(f"{directory}/{file_name}", 'r', encoding="utf-8")
    new_path_file = open(
        f"{directory}/{file_name.split('.')[0]}_red.path", "w", encoding="utf-8")

    path_data = json.loads(path_file.read())

    for waypoint in path_data["waypoints"]:
        waypoint["anchorPoint"]["x"] = FIELD_LENGTH - \
            waypoint["anchorPoint"]["x"]
        if 180 - waypoint["holonomicAngle"] == 0:
            waypoint["holonomicAngle"] = -0.01
        else:
            waypoint["holonomicAngle"] = 180 - waypoint["holonomicAngle"]

        if waypoint["nextControl"] != None:
            waypoint["nextControl"]["x"] = FIELD_LENGTH - \
                waypoint["nextControl"]["x"]

        if waypoint["prevControl"] != None:
            waypoint["prevControl"]["x"] = FIELD_LENGTH - \
                waypoint["prevControl"]["x"]

        waypoints.append(waypoint)

    path_data["waypoints"] = waypoints
    new_path_file.write(json.dumps(path_data))

    path_file.close()
    new_path_file.close()


files = os.listdir(DIRECTORY)

for file in files:
    if file.split("_")[-1] == "red.path":
        continue

    generate_red_aliance(file, DIRECTORY)
    print(f"{file}_red.path: ✅")
