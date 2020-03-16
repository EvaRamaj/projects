import csv
import random

if __name__ == "__main__":
	csvfile = open("random.csv", "w")
	writer = csv.writer(csvfile)

	for i in range(10000):
		filename = "random_%s.png" % str(i).zfill(5)

		row = [filename]
		for i in range(8):
			row.append(random.randint(0,3))
		writer.writerow(row)

