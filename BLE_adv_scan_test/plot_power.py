from pylab import arange, plot, show, semilogy, grid, xlabel, ylabel, savefig

values = []
f = open("log.txt", "r")
for line in f:
    values.append(float(line))



print len(values)
t = arange(float(0), float(len(values)), 1.0)
plot(t, values)
grid(True)
#savefig("transmit_delay.png")
show()
