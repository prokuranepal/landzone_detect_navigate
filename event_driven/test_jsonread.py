import json

while True:
    try:
        with open('test.txt') as f:
            json_data = json.load(f)
            print(json_data)
    except:
        continue