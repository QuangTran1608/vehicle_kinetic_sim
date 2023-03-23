import os

def ls(dir='/', indent=''):
    if dir and dir[-1] != '/': dir += '/'
    for entry in os.listdir(dir):
        stat = os.stat(dir + entry)
        if stat[0] == 16384:
            print(indent + entry + '/')
            ls(dir + entry + '/', indent + '  ')
        if stat[0] == 32768:
            print(indent + entry, stat[6])

def cat(filename):
    with open(filename, 'r') as file:
        return file.read()

ls()
