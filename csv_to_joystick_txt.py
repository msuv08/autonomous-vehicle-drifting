import csv

with open('dataset/ikddata2.csv', newline='') as csvfile, open('output.txt', 'w') as txtfile:
    reader = csv.reader(csvfile)
    for row in reader:
        joystick_values = row[1]
        txtfile.write(joystick_values + '\n')
