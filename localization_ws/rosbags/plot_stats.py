import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import linregress

# Load the CSV file
df = pd.read_csv('error.csv', delimiter=',')

# Calculate the error for each row
df['error'] = ((df['target_x'] - df['/interval/x_empty_lb'])**2 + (df['target_y'] - df['/interval/y_empty_lb'])**2)**0.5

# Group by L and calculate the mean error
mean_error_by_L = df.groupby('L')['error'].mean()

# Prepare data for linear regression
L_values = mean_error_by_L.index.to_numpy()
mean_errors = mean_error_by_L.values

# Perform linear regression for mean error
slope, intercept, r_value, p_value, std_err = linregress(L_values, mean_errors)

# Plot the mean error for each L
plt.figure(figsize=(10, 6))
mean_error_by_L.plot(kind='bar')
plt.plot(L_values - 2, intercept + slope * L_values, color='red', label='Linear Regression')
plt.xlabel('L')
plt.ylabel('Mean Error')
plt.title('Mean Error relative to L')
plt.legend()

# Calculate relative error in percent
df['relative_error_percent'] = df['error'] / df['L'] * 100

# Group by L and calculate the mean relative error
mean_relative_error_by_L = df.groupby('L')['relative_error_percent'].mean()

# Prepare data for linear regression for relative error percentage
relative_errors = mean_relative_error_by_L.values
slope_rel, intercept_rel, r_value_rel, p_value_rel, std_err_rel = linregress(L_values, relative_errors)

# Plot the mean relative error for each L with linear regression
plt.figure(figsize=(10, 6))
mean_relative_error_by_L.plot(kind='bar', color='orange')
plt.plot(L_values - 2, intercept_rel + slope_rel * L_values, color='blue', label='Linear Regression')
plt.xlabel('L')
plt.ylabel('Mean Relative Error (%)')
plt.title('Mean Relative Error (%) relative to L')
plt.legend()
plt.show()
