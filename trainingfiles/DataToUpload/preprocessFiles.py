import os
import re
import csv

def extract_data_between_markers(file_path, start_marker, end_marker, include_headers=False):
    """
    Extract data between start and end markers in a file.
    Returns list of rows (as strings) found between the markers.
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
    
    # Escape markers for regex if they contain special characters
    start_pattern = re.escape(start_marker)
    end_pattern = re.escape(end_marker)
    
    # Find all occurrences between start and end markers
    pattern = f"{start_pattern}(.*?){end_pattern}"
    matches = re.findall(pattern, content, re.DOTALL)
    
    if not matches:
        return []
    
    # Get the first match and split into lines
    data_section = matches[0].strip()
    lines = data_section.split('\n')
    
    # Filter out empty lines and strip whitespace
    lines = [line.strip() for line in lines if line.strip()]
    
    return lines

def process_file(file_path, filename):
    """
    Process a single file to extract both raw and processed data.
    """
    base_name = filename.replace('.txt', '')
    
    # Check if it's a -RealData.txt file, skip if it is
    if filename.endswith('-RealData.txt'):
        print(f"Skipping {filename} (ends with -RealData.txt)")
        return
    
    print(f"Processing {filename}...")
    
    # Define markers
    raw_start = """Format: RAW,Time_0.1s,IR_raw,Red_raw,GSR_V,GSR_raw
------------------------------------------"""
    raw_end = "\n------------------------------------------"
    
    proc_start = """Format: PROC,Time_0.1s,BPM,SpO2,GSR_V,GSR_raw
------------------------------------------"""
    proc_end = "\n=========================================="
    
    # Extract raw data
    raw_lines = extract_data_between_markers(file_path, raw_start, raw_end)
    
    if raw_lines:
        # Create Data_RawIR folder if it doesn't exist
        raw_folder = "Data_RawIR-test"
        os.makedirs(raw_folder, exist_ok=True)
        
        # Write raw data to CSV
        raw_output_file = os.path.join(raw_folder, f"{base_name}-RawData.csv")
        with open(raw_output_file, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            
            # First line should be the headers
            headers = "Format: RAW,Time_0.1s,IR_raw,Red_raw,GSR_V,GSR_raw"
            writer.writerow(headers.split(','))
            
            # Write the data rows
            for line in raw_lines:
                # Skip the header line if it appears in the data
                if "Format:" not in line:
                    # Split by comma (assuming CSV format)
                    row = line.split(',')
                    writer.writerow(row)
        
        print(f"  Created {raw_output_file} with {len(raw_lines)} rows")
    else:
        print(f"  No raw data found in {filename}")
    
    # Extract processed data
    proc_lines = extract_data_between_markers(file_path, proc_start, proc_end)
    
    if proc_lines:
        # Create Data_ProcessedIR folder if it doesn't exist
        proc_folder = "Data_ProcessedIR-test"
        os.makedirs(proc_folder, exist_ok=True)
        
        # Write processed data to CSV
        proc_output_file = os.path.join(proc_folder, f"{base_name}-ProcIRData.csv")
        with open(proc_output_file, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            
            # First line should be the headers
            headers = "Format: PROC,Time_0.1s,BPM,SpO2,GSR_V,GSR_raw"
            writer.writerow(headers.split(','))
            
            # Write the data rows
            for line in proc_lines:
                # Skip the header line if it appears in the data
                if "Format:" not in line:
                    # Split by comma (assuming CSV format)
                    row = line.split(',')
                    writer.writerow(row)
        
        print(f"  Created {proc_output_file} with {len(proc_lines)} rows")
    else:
        print(f"  No processed data found in {filename}")

def main():
    """
    Main function to process all .txt files in the current directory.
    """
    # Get current directory
    current_dir = os.getcwd()
    
    # Find all .txt files in current directory
    txt_files = [f for f in os.listdir(current_dir) if f.endswith('.txt')]
    
    if not txt_files:
        print("No .txt files found in current directory.")
        return
    
    print(f"Found {len(txt_files)} .txt file(s)")
    print("-" * 50)
    
    # Process each file
    for filename in txt_files:
        file_path = os.path.join(current_dir, filename)
        process_file(file_path, filename)
    
    print("-" * 50)
    print("Processing complete!")

if __name__ == "__main__":
    main()