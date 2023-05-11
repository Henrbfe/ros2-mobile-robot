import pandas as pd
import matplotlib.pyplot as plt


df = pd.read_csv("dead_reckoning_measures.csv", header=0, index_col=0)
print(df.head())
df.plot(x="time", y=["x","x_calc"], kind="line", figsize=(10, 10))
plt.show()