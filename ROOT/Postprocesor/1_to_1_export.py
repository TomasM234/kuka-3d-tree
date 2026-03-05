import sys
import os

def process_csv(input_csv, output_txt):
    if not os.path.exists(input_csv):
        print(f"Error: Input file {input_csv} does not exist.")
        sys.exit(1)
        
    try:
        with open(input_csv, 'r', encoding='utf-8') as infile, \
             open(output_txt, 'w', encoding='ascii', newline='') as outfile:
            for line in infile:
                outfile.write(line.rstrip('\n\r') + '\r\n')
        print(f"Successfully processed {input_csv} to {output_txt}")
    except Exception as e:
        print(f"Error processing file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python 1_to_1_export.py <input_csv> <output_txt>")
        sys.exit(1)
        
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    process_csv(input_file, output_file)
