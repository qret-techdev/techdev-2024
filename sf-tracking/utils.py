import datetime
import cv2
import csv
import os

def initialize_video_writer(cap, width, height, fps):
    os.makedirs('output_video', exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"output_video/output_{timestamp}.mp4"
    return cv2.VideoWriter(filename, fourcc, fps, (width, height))

def get_params(filename, param):
    """
    Reads the specified parameter from a CSV file and returns the associated values.

    Parameters:
    filename (str): The path to the CSV file.
    param (str): The parameter name to retrieve values for.

    Returns:
    tuple or single value: Returns a tuple if the parameter has multiple values, otherwise returns a single value.
    """
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if row[0] == param:
                # Handle the case where there are multiple values
                if len(row) > 2:
                    values_tuple = tuple(map(float, row[1:]))
                    return values_tuple
                else:
                    # Return a single value as a float if it's numeric, or keep as string if not
                    try:
                        return float(row[1])
                    except ValueError:
                        return row[1].strip('\'"')  # Strip any surrounding quotes if it's a string

    # If the parameter is not found, raise an error
    raise ValueError(f"Parameter '{param}' not found in the CSV file.")