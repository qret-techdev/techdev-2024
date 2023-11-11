import numpy as np

# Create two NumPy arrays
array1 = np.array([5, 7, 8, 10])
array2 = np.array([3, 6, 9, 12])

# Subtract the arrays and set negative values to 0
result = np.maximum(array1 - array2, 0)

print("Result:")
print(result)