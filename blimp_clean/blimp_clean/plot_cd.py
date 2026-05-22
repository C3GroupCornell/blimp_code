import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

Kv = 0.1307
Cd = 0.0300

# Motor input u from 0..1 (avoid 0 to prevent division by zero)
u = np.linspace(0.2, 1.0, 400)

# Drag term as % of motor thrust:  Cd*zdot / (2*Kv*u)
# Plotted for several representative velocities.
zdot_values = [0.1, 0.2, 0.3, 0.5]

fig, ax = plt.subplots(figsize=(8, 5))

for zdot in zdot_values:
    pct = Cd * zdot / (2 * Kv * u) * 100
    ax.plot(u, pct, linewidth=1.6, label=f'ż = {zdot} m/s')

ax.axhline(100, color='k', linewidth=0.8, linestyle='--', label='100% (drag = motor)')
ax.set_xlim(0.2, 1)
ax.set_ylim(0, None)
ax.set_xlabel('Motor input u (V)')
ax.set_ylabel('Cd·ż / (2·Kv·u)  (%)')
ax.set_title(
    f'Drag term as % of motor thrust vs motor input\n'
    f'Kv={Kv}  Cd={Cd}'
)
ax.legend(fontsize=8, loc='upper right')
ax.grid(alpha=0.3)
fig.tight_layout()
plt.show()
