import csv 
import sys

filename = str(sys.argv[1])

data = []  
with open(filename) as csvfile:
    csvreader = csv.reader(csvfile, delimiter=',')
    for row in csvreader:
        data.append(row)

for i in range(len(data[0])):
    line = 'double ' + data[0][i] + '[] = {'

    for j in range(1, len(data)-1):
        line = line + data[j][i] + ', '

    line = line + data[len(data)-1][i]  + '};'

    print(line)