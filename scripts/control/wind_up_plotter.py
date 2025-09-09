import pandas as pd
import matplotlib.pyplot as plt

# Path
CSV_PATH = "../../results/control/wind_up.csv"

# Load
data = pd.read_csv(CSV_PATH)

# Plot
print(data.head());

plt.figure(figsize=(10, 6))
plt.plot(data["time"], data["P"], label="Proportional (P)", linestyle="--")
plt.plot(data["time"], data["I"], label="Integral (I)", linestyle="--")
plt.plot(data["time"], data["D"], label="Derivative (D)", linestyle="--")

plt.xlabel("Time")
plt.ylabel("Value")
plt.title("Ant-Windup Test")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
