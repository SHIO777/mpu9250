import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

filename1 = "2_uncalibrated.csv"
filename2 = "2_calibrated.csv"


df = pd.read_csv(filename1, delimiter=",", encoding="cp932")
# df.head()
df2 = pd.read_csv(filename2, delimiter=",", encoding="cp932")
# df2.head()

fig = plt.figure(figsize=(15,10))
ax = fig.add_subplot(211, xlim=(0, 500), ylim=(-2.0, 2.0))
ax2 = fig.add_subplot(212, xlim=(0, 500), ylim=(-2.0, 2.0))

ax.plot(df['x [dps]'], label="$\omega_x, Uncalibrated$")
ax.plot(df['y [dps]'], label="$\omega_y, Uncalibrated$")
ax.plot(df['z [dps]'], label="$\omega_z, Uncalibrated$")

ax2.plot(df2['x [dps]'], label="$\omega_x, Calibrated$")
ax2.plot(df2['y [dps]'], label="$\omega_y, Calibrated$")
ax2.plot(df2['z [dps]'], label="$\omega_z, Calibrated$")

ax.legend()
ax2.legend()
ax.set_title("Gyroscope Calibration Offset Correction", fontsize=22)
ax.set_xlabel("", fontsize=18)
ax.set_ylabel("$\omega_{x, y, z}$ [$^{\circ}/sec$]", fontsize=18)
ax2.set_xlabel("Sample", fontsize=18)
ax2.set_ylabel("$\omega_{x, y, z}$ [$^{\circ}/sec$]", fontsize=18)

plt.savefig('gyro.png', dpi=500)
plt.close('all')
