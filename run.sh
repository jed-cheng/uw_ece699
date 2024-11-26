#!/bin/bash

# Create the paint directory if it doesn't exist
mkdir -p paint
counter=0
# Loop through each .mp3 file in the music folder
for file in music/*.mp3; do
  # Get the base name of the file (without the directory and extension)
  # start from the ninth file
  counter=$((counter + 1))
  if [ $counter -lt 9 ]; then
    continue
  fi
  base_name=$(basename "$file" .mp3)
  
  # Run main.py with the input file and save the output in the paint folder
  python main.py "$file" --output "paint/${base_name}.png"
done