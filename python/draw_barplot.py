import matplotlib.pyplot as plt

# Data
data = [[0.02, 0.07, 0.13, 0.14], [0.16, 1.20, 9.50, 85.9]]
x_labels = ['20^{10}', '100^{10}', '200^{10}', '200^{20}']

# Plot
fig, ax = plt.subplots()
bar_width = 0.35
opacity = 0.8

for i, d in enumerate(data):
    x = [j for j in range(len(d))]
    ax.bar([x_val + i * bar_width for x_val in x], d, bar_width,
           alpha=opacity, label=['Ours', 'Baseline'][i])

ax.set_xticks([x_val + bar_width / 2 for x_val in x])
ax.set_xticklabels(x_labels)
ax.set_xlabel('Search space size')
ax.set_ylabel('Planning time(s)')
ax.legend()

# y_min, y_max = 0, max(max(data))
# ax.set_ylim(y_min, y_max)
ax.set_yscale('log')

fm = "eps"
fig.savefig("bar_plot." +fm, format=fm)
