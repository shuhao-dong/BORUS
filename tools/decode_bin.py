import struct
import csv
import argparse
import os

# --- Configuration ---
# Structure format string:
# '<' = Little-endian
# 'I' = unsigned int (4 bytes for uint32_t timestamp)
# 'h' = short (2 bytes for int16_t) - we need 6 of them
STRUCT_FORMAT = '<Ihhhhhh'
RECORD_SIZE = struct.calcsize(STRUCT_FORMAT) # Should be 16 bytes
IMU_SCALING_FACTOR = 100.0 # Divide by this to get original float values


def decode_log_file(input_bin_path, output_csv_path):
    """
    Decodes the binary IMU log file and writes the data to a CSV file.

    Args:
        input_bin_path (str): Path to the input imu_log.bin file.
        output_csv_path (str): Path to the output CSV file to be created.
    """
    print(f"Decoding binary file: {input_bin_path}")
    print(f"Outputting to CSV: {output_csv_path}")
    print(f"Expected record size: {RECORD_SIZE} bytes")

    records_processed = 0
    try:
        with open(input_bin_path, 'rb') as infile, \
             open(output_csv_path, 'w', newline='') as outfile:

            csv_writer = csv.writer(outfile)
            # Write CSV header
            csv_writer.writerow([
                'timestamp_ms',
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z'
            ])

            while True:
                chunk = infile.read(RECORD_SIZE)
                if not chunk:
                    break # End of file

                if len(chunk) < RECORD_SIZE:
                    print(f"\nWarning: Incomplete record found at the end of the file ({len(chunk)} bytes). Stopping.")
                    break

                # Unpack the binary data according to the format string
                try:
                    unpacked_data = struct.unpack(STRUCT_FORMAT, chunk)
                except struct.error as e:
                    print(f"\nError unpacking record {records_processed + 1}: {e}")
                    print("Stopping due to data format error.")
                    break

                # Extract values
                timestamp_ms = unpacked_data[0]
                # Scale the IMU readings back to float
                accel_x = unpacked_data[1] / IMU_SCALING_FACTOR
                accel_y = unpacked_data[2] / IMU_SCALING_FACTOR
                accel_z = unpacked_data[3] / IMU_SCALING_FACTOR
                gyro_x = unpacked_data[4] / IMU_SCALING_FACTOR
                gyro_y = unpacked_data[5] / IMU_SCALING_FACTOR
                gyro_z = unpacked_data[6] / IMU_SCALING_FACTOR

                # Write row to CSV
                csv_writer.writerow([
                    timestamp_ms,
                    f"{accel_x:.4f}", f"{accel_y:.4f}", f"{accel_z:.4f}",
                    f"{gyro_x:.4f}", f"{gyro_y:.4f}", f"{gyro_z:.4f}"
                ])
                records_processed += 1

                # Optional: Print progress
                if records_processed % 1000 == 0:
                    print(f"\rProcessed {records_processed} records...", end="")

    except FileNotFoundError:
        print(f"\nError: Input file not found: {input_bin_path}")
        return
    except IOError as e:
        print(f"\nError reading/writing file: {e}")
        return
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        return

    print(f"\nFinished decoding. Processed {records_processed} records.")
    print(f"CSV data saved to: {output_csv_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Decode imu_log.bin from Zephyr device to CSV.")
    parser.add_argument(
        "input_file",
        help="Path to the input imu_log.bin file."
    )
    parser.add_argument(
        "-o", "--output",
        help="Path to the output CSV file. Defaults to input filename with .csv extension.",
        default=None
    )

    args = parser.parse_args()

    input_path = args.input_file
    output_path = args.output

    if not output_path:
        # Default output path: replace .bin with .csv
        base_name = os.path.splitext(input_path)[0]
        output_path = base_name + ".csv"

    decode_log_file(input_path, output_path)