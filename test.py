import numpy as np
import time

# Generate exponential distribution sample
generated_number = np.random.exponential(0.1, 100000) + 1

print(generated_number)
print(np.mean(generated_number))

