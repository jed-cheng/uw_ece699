#!/bin/bash

input_file="music/Prelude and Fugue in C (WTK, Book I, No.1), BWV 846.mp3"

# Create the paint directory if it doesn't exist
mkdir -p feature

# Test different numbers of robots
for num_robot in 6 9 12; do
  output_file="feature/output_robot${num_robot}_L1_trail15_color3.png"
  python main.py "$input_file" --robot "$num_robot" --output "$output_file"
done

# Test different L values
for L_value in 1 3 5; do
  output_file="feature/output_robot6_L${L_value}_trail15_color3.png"
  python main.py "$input_file" --l "$L_value" --output "$output_file"
done

# Test different trail widths
for trail_width in 10 15 20; do
  output_file="feature/output_robot6_L1_trail${trail_width}_color3.png"
  python main.py "$input_file" --trail "$trail_width" --output "$output_file"
done

# Test different numbers of colors
for color_num in 1 2 3; do
  output_file="feature/output_robot6_L1_trail15_color${color_num}.png"
  python main.py "$input_file" --color "$color_num" --output "$output_file"
done