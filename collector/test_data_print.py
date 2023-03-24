import os

FILE_PREFIX = "test_data_"
FILE_EXT = ".txt"

def cat(filename):
    with open(filename, 'r') as f:
        while True:
            line = f.readline()
            if not line:
                break
            print(line.strip())

min_timestamp = float("inf")
latest_file = None

for file in os.listdir('/'):
    if FILE_PREFIX not in file:
        continue

    begin = len(FILE_PREFIX)
    end = len(file) - len(FILE_EXT)
    timestamp = int(file[begin:end])
    if timestamp < min_timestamp:
        timestamp = min_timestamp
        latest_file = file

if latest_file:
    cat(latest_file)