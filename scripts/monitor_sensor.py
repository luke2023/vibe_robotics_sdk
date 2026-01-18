from viberobotics.sensor.imu_arduino import RemoteIMU
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import numpy as np

# Create figure for plotting
fig, ax = plt.subplots(3, 4, figsize=(16, 8))
plt.tight_layout()
xs = []
ys = []

imu = RemoteIMU(port='/dev/ttyACM1', baudrate=9600)
quats, gyros, accs = [], [], []
times = []

start_time = time.time()

def animate(i, times, quats, gyros, accs):
    # Add x and y to lists
    times.append(time.time() - start_time)
    quats.append(imu.quaternion)
    gyros.append(imu.gyro)
    accs.append(imu.acc)
    
    times = times[-20:]
    quats = quats[-20:]
    gyros = gyros[-20:]
    accs = accs[-20:]

    rows = [quats, gyros, accs]
    n_cols = [4, 3, 3]
    row_titles = ['Quaternion', 'Gyroscope (rad/s)', 'Accelerometer (m/s²)']
    for r in range(3):
        vmin = min(min(row) for row in rows[r])
        vmax = max(max(row) for row in rows[r])
        vlim = max(np.abs(vmin), np.abs(vmax))
        vmin, vmax = -vlim * 1.5, vlim * 1.5
        for c in range(n_cols[r]):
            ax[r, c].clear()
            ax[r, c].plot(times, [row[c] for row in rows[r]], label=f'{row_titles[r]} {c}')
            ax[r, c].set_ylim(vmin, vmax)  # Corrected np.ab to np.abs(vmin) * 0.1
            
            ax[r, c].set_ylabel(row_titles[r])
            ax[r, c].legend(loc='upper left')

ani = animation.FuncAnimation(fig, animate, fargs=(times, quats, gyros, accs), interval=10)
plt.show()
