import matplotlib.pyplot as plt
import numpy as np

data = []
with open("output.txt") as f:
    for value in f:
        raw = value.split("\n")
        raw.pop()
        for i in raw:
            data.append(float(i))

x = np.linspace(0, 60, num=1362)
plt.plot(x,data, 'r')
plt.xlabel("Time")
plt.ylabel("Position Error")
plt.title("Localization Error over Time")
plt.show()