import json

file = open('arena_mapper.json')
data = json.load(file)

points = []
for i in data:
    data[i][2] = 20
    points.append(data[i])

print(data)