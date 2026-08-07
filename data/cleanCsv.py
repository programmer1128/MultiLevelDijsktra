import sys

print("Cleaning \\u0000 null characters from overture_locations.csv...")

input_file = "overture_locations.csv"
output_file = "overture_locations_clean.csv"

count = 0
with open(input_file, "r", encoding="utf-8", errors="ignore") as infile, \
     open(output_file, "w", encoding="utf-8") as outfile:
    
    for line in infile:
        # Remove literal '\u0000' escape sequences and raw NUL bytes '\x00'
        cleaned_line = line.replace("\\u0000", "").replace("\x00", "")
        outfile.write(cleaned_line)
        count += 1
        if count % 1000000 == 0:
            print(f"Processed {count // 1000000}M lines...")

print(f"Done! Cleaned CSV saved as '{output_file}'.")