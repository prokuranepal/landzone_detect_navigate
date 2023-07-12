# Landing zone detect Navigate

Here the darknet by Joseph Redmon is used to run YOLO model to detect the landing zone of our drone. 
Then the coordinates generated are passed through sockets to a event driven python program which will control the drone.

## Installation

- Run the darknet model
```bash
pip install pymavlink
```

## Usage

```
python dronekit_detect_land.py
```
## Outuput
![land_yolo_tiny-2](https://github.com/prokuranepal/landzone_detect_navigate/assets/33648198/1d3bbe0c-aa90-44f8-99be-a719b5b7e9d8)


## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
