Python Helper
#############

This python script converts .bin file extracted from the wearable to a .csv file.

Install Library
###############

To use this script, you will need the following libraries:
struct, csv, argparse, os

Use 
###

To use the script, simple run::

  python decode_bin.py <path/to/imu_log.bin> -o <path/to/output.csv>

The output csv file should have 7 columns::

  timestamp_ms accel_x accel_y accel_z gyro_x gyro_y gyro_z
