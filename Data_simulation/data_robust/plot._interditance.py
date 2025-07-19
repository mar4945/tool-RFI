import numpy as np
import matplotlib.pyplot as plt



# Load the first dataset
data1 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_45.txt')
x45 = data1[:, 0]
y45 = data1[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_46.txt')
x46 = data2[:, 0]
y46 = data2[:, 1]

# Load the first dataset
data1 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_47.txt')
x47 = data1[:, 0]
y47 = data1[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_48.txt')
x48 = data2[:, 0]
y48 = data2[:, 1]

# Load the first dataset
data1 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_49.txt')
x49 = data1[:, 0]
y49 = data1[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_50.txt')
x50 = data2[:, 0]
y50 = data2[:, 1]

# Load the first dataset
data1 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_51.txt')
x51 = data1[:, 0]
y51 = data1[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_52.txt')
x52 = data2[:, 0]
y52 = data2[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_53.txt')
x53 = data2[:, 0]
y53 = data2[:, 1]

# Load the first dataset
data1 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_54.txt')
x54 = data1[:, 0]
y54 = data1[:, 1]

# Load the second dataset
data2 = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_55.txt')
x55 = data2[:, 0]
y55 = data2[:, 1]



import numpy as np
import math 
# Carica tutti i vettori y
y_vectors = []
for i in range(45, 56):
    file_path = f'C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_{i}.txt'
    data = np.loadtxt(file_path)
    y = data[:, 1]
    y_vectors.append(y)

# Stack dei vettori y per riga (ogni riga è un istante di tempo)
Y = np.column_stack(y_vectors)
n_signals = Y.shape[1]  # numero di segnali (colonne)

# Calcolo media campionaria per ogni istante (riga)
mean_y = [sum(row) / n_signals for row in Y]

# Calcolo varianza e deviazione standard manualmente
var_y = []
std_y = []

for i, row in enumerate(Y):
    mean_i = mean_y[i]
    squared_diffs = [(val - mean_i) ** 2 for val in row]
    var_i = sum(squared_diffs) / (n_signals - 1)  # varianza campionaria
    std_i = math.sqrt(var_i)  # deviazione standard campionaria

    var_y.append(var_i)
    std_y.append(std_i)

# Asse x dal primo file
x = np.loadtxt('C:/Users/user/Desktop/toolRFI/tool-RFI/Data_simulation/data_robust/interDistance_45.txt')[:, 0]

# Calcolo dei limiti per la fascia ±3σ
upper_bound = [m + 3*s for m, s in zip(mean_y, std_y)]
lower_bound = [m - 3*s for m, s in zip(mean_y, std_y)]

# Write to text files (pure Python I/O)
with open("mean_y.txt", "w") as f_mean, \
     open("upper_bound.txt", "w") as f_upper, \
     open("lower_bound.txt", "w") as f_lower:

    f_mean.write("x mean_y\n")
    f_upper.write("x upper_bound\n")
    f_lower.write("x lower_bound\n")

    for xi, mi, ui, li in zip(x, mean_y, upper_bound, lower_bound):
        f_mean.write(f"{xi:.6f} {mi:.6f}\n")
        f_upper.write(f"{xi:.6f} {ui:.6f}\n")
        f_lower.write(f"{xi:.6f} {li:.6f}\n")


# Plot dei risultati
plt.figure(figsize=(12, 8))

# Media con fascia ±3σ
plt.subplot(3, 1, 1)
plt.plot(x, mean_y, label='Media campionaria', color='blue')
plt.fill_between(x, lower_bound, upper_bound, color='blue', alpha=0.2, label='±3 Dev. Std.')
plt.ylabel('Media')
plt.title('Media, Varianza e Deviazione Standard (con fascia ±3σ)')
plt.grid(True)
plt.legend()

# Varianza
plt.subplot(3, 1, 2)
plt.plot(x, var_y, label='Varianza campionaria', color='red')
plt.ylabel('Varianza')
plt.grid(True)
plt.legend()

# Deviazione standard
plt.subplot(3, 1, 3)
plt.plot(x, std_y, label='Deviazione standard campionaria', color='green')
plt.xlabel('x')
plt.ylabel('Dev. Std.')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()