import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv(r'')

# Extract the columns x, y, and z
x = data['x']
y = data['y']
z = data['z']

# Merge x, y, and z into a single array
xyz = np.stack((x, y, z), axis=1)

# Create a time variable based on the index of the DataFrame
time = data.index


data= pd.read_csv(r'')

import ast 
# Extract the columns x, y, and z
pos= data['position']

data['position'] = data['position'].apply(ast.literal_eval)

# Create separate arrays for each value in the list
array1 = data['position'].apply(lambda x: x[0]).to_numpy()
array2 = data['position'].apply(lambda x: x[1]).to_numpy()
array3 = data['position'].apply(lambda x: x[2]).to_numpy()


# Plot x, y, and z as functions of time
x= x[:len(pos)]
y= y[:len(pos)]
z= z[:len(pos)]
time= time[:len(pos)]




import matplotlib.ticker as ticker

def micro_to_seconds(x, pos):
    return '{:.1f}'.format(x / 1000)  # Convert from microseconds to seconds



plt.gca().xaxis.set_major_formatter(ticker.FuncFormatter(micro_to_seconds))


plt.plot(array1,'r', label='Commanded x')
plt.plot(array2,'g' ,label='Commanded y')
plt.plot(array3, 'b',label='Commanded z')
plt.plot(x, 'b--',label='Actual x')
plt.plot(y, 'y--',label='Actual Y')
plt.plot(z, 'r--',label='Actual z')






plt.xlabel('Time')
plt.ylabel('Value')
plt.title('Vehicle Local Position')
plt.legend()




plt.show()
