import os

FILE_PREFIX = "test_data_"

for file in os.listdir('/'):
    if FILE_PREFIX not in file:
        continue

    os.remove(file)