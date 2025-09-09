import pandas as pd
import matplotlib.pyplot as plt

# Path
CSV_PATH = "../../results/control/pid_test.csv"

# Load
data = pd.read_csv(CSV_PATH)

# Plot
print(data.head());

plt.figure(figsize=(10, 6))
plt.plot(data["time"], data["ref"], label="Reference Point", linestyle="--")
plt.plot(data["time"], data["y"], label="Output (y)")
#plt.plot(data["time"], data["P"], label="Proportional (P)", linestyle="--")
#plt.plot(data["time"], data["I"], label="Integral (I)", linestyle="--")
#plt.plot(data["time"], data["D"], label="Derivative (D)", linestyle="--")
#plt.plot(data["time"], data["u"], label="Actuation (u)", linestyle="--")

plt.xlabel("Time")
plt.ylabel("Value")
plt.title("2nd Order PID system output")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
