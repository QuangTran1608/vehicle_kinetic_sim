def cat(filename):
    with open(filename, 'r') as file:
        return file.read()

print(cat('test_data.txt'))