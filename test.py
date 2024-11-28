import matplotlib.pyplot as plt
import numpy as np

# Define the function
def tempo_control_L(tempo, TEMPO_MAX=200):
    return 10 - ((9) / TEMPO_MAX) * np.minimum(tempo, TEMPO_MAX)

# Generate a range of tempo values
tempo_values = np.linspace(0, 300, 500)  # From 0 to 300 with 500 points

# Calculate the function values for each tempo
L_values = tempo_control_L(tempo_values)

# Plot the function
plt.figure(figsize=(10, 6))
plt.plot(tempo_values, L_values, label='L = 10 - ((9) / TEMPO_MAX) * min(tempo, TEMPO_MAX)')
plt.xlabel('Tempo')
plt.ylabel('L')
plt.title('Plot of L as a function of Tempo')
plt.axhline(0, color='black', linewidth=0.5, linestyle='--')
plt.axvline(0, color='black', linewidth=0.5, linestyle='--')
plt.grid(True)
plt.legend()
plt.show()