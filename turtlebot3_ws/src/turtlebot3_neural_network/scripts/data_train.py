import tensorflow as tf
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Read Data
columns = []
for i in range(120):
    columns.append(f'x_{i}')

columns = columns + ['v', 'w']

df = pd.read_csv('data.csv')
df.replace([np.inf], 3.5, inplace = True)

df.columns = columns
df.loc[df['v'] == 0.3, "v"] = 1
df.loc[df['v'] == 0.3, "w"] = 0
df.loc[df['w'] == 0.3, "v"] = 0
df.loc[df['w'] == 0.3, "w"] = 1

print("Raw Data")
print(df.head(5))
print(f"Number of Data Entries: {len(df)}")
print(f"Number of v Entries: {len(df[df['v'] == 1])}")
print(f"Number of v Entries: {len(df[df['w'] == 1])}")

y = pd.concat([df.pop(x) for x in ['v', 'w']], axis = 1)
X = df

print("Prepared Data")
print("y Dataframe\n")
print(y.head(5))

print("X Dataframe\n")
print(X.head(5))

# Prepare Data for Tensorflow
X_tensor = tf.convert_to_tensor(X)

# Create Model
model = tf.keras.models.Sequential([
    tf.keras.layers.Dense(100, activation='relu'),
    tf.keras.layers.Dense(2, activation = 'sigmoid')
])

model.compile(loss = 'categorical_crossentropy', 
              optimizer = 'adam', 
              metrics=['accuracy'])
model.fit(X_tensor, y, epochs = 30, batch_size = 8, validation_split = 0.2, shuffle = True)

model.save('model.hdf5')