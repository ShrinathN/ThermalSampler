import sys
import pandas as pd

#python3 pwm.py [csv filename] [time offset] [time suffix] [value suffix]

FILENAME = sys.argv[1]
TIME_OFFSET = float(sys.argv[2])
TIME_SUFFIX = sys.argv[3]
VALUE_SUFFIX = sys.argv[4]

data = pd.read_csv(FILENAME)
# two columns timestamp and current (in uA)

starting_time = data['timestamp'][0]
data['timestamp'] = (data['timestamp'] - starting_time) + TIME_OFFSET

for i in range(0, len(data)):
	print(round(data['timestamp'][i], 2), end='')
	print(TIME_SUFFIX, end=' ')
	print(round(data['current'][i], 2), end='')
	print(VALUE_SUFFIX)
