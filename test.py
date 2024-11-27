import matplotlib.pyplot as plt
import numpy as np

# Function to convert polar to cartesian coordinates
def pol2cart(theta, r):
    theta_rad = np.radians(theta)
    x = r * np.cos(theta_rad)
    y = r * np.sin(theta_rad)
    return x, y

# Define radius
R = 1

# Define chords and their polar locations
chord_to_center = {
    # Inner Circle (Major Chords)
    'C': pol2cart(0, R),
    'G': pol2cart(30, R),
    'D': pol2cart(60, R),
    'A': pol2cart(90, R),
    'E': pol2cart(120, R),
    'B': pol2cart(150, R),
    'F#': pol2cart(180, R),
    'Db': pol2cart(210, R),
    'C#': pol2cart(210, R),
    'Ab': pol2cart(240, R),
    'Eb': pol2cart(270, R),
    'Bb': pol2cart(300, R),
    'F': pol2cart(330, R),
    
    # Middle Circle (Minor Chords)
    'Am': pol2cart(0, 2*R),
    'Em': pol2cart(30, 2*R),
    'Bm': pol2cart(60, 2*R),
    'F#m': pol2cart(90, 2*R),
    'C#m': pol2cart(120, 2*R),
    'G#m': pol2cart(150, 2*R),
    'Abm': pol2cart(150, 2*R),
    'D#m': pol2cart(180, 2*R),
    'Bbm': pol2cart(210, 2*R),
    'Fm': pol2cart(240, 2*R),
    'Cm': pol2cart(270, 2*R),
    'Gm': pol2cart(300, 2*R),
    'Dm': pol2cart(330, 2*R),
    
    # Outer Circle (Diminished Chords and others)
    'B°': pol2cart(0, 3*R),
    'F#°': pol2cart(30, 3*R),
    'C#°': pol2cart(60, 3*R),
    'G#°': pol2cart(90, 3*R),
    'D#°': pol2cart(120, 3*R),
    'A#°': pol2cart(150, 3*R),
    'E#°': pol2cart(180, 3*R),
    'B#°': pol2cart(210, 3*R),
    'F°': pol2cart(240, 3*R),
    'C°': pol2cart(270, 3*R),
    'G°': pol2cart(300, 3*R),
    'D°': pol2cart(330, 3*R)
}

# Plot the circle
plt.figure(figsize=(8, 8))
plotted_coords = set()
for chord, (x, y) in chord_to_center.items():
    if (x, y) not in plotted_coords:
        plt.text(x, y, chord, fontsize=24, ha='center', va='center')  # Annotate chord names with larger font size
        plotted_coords.add((x, y))

# Draw the circles
circle_radii = [R, 2*R, 3*R]
for radius in circle_radii:
    circle = plt.Circle((0, 0), radius, color='gray', fill=False, linestyle='--')
    plt.gca().add_artist(circle)

# Formatting
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
plt.gca().set_aspect('equal', adjustable='datalim')
plt.xlim(-3.5*R, 3.5*R)  # Set x-axis limits to show the entire circle
plt.ylim(-3.5*R, 3.5*R)  # Set y-axis limits to show the entire circle
plt.title("Chord Positions on Circle")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()