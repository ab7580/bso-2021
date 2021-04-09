import os
import matplotlib.pyplot as plt
import numpy as np

p0 = 101325
def get_alt(p, T):
    return ((pow(p0/p, 1/5.257) - 1)*(T+273.15)) / 0.0065

if __name__ == "__main__":
    folder = "data"
    format = "%Y-%m-%d %H:%M:%S"
    figure, axis = plt.subplots(1, 2, figsize=(10, 5))

    avgs = []
    for file in os.listdir(folder):
        if file != "3stuk.log":
            continue

        if file == "1stuk.log":
            axis[0].set_ylim([91700, 91850])
        elif file == "2stuk.log":
            axis[0].set_ylim([91640, 91720])
        elif file == "3stuk.log":
            axis[0].set_ylim([91700, 91850])

        times = []
        pressures = []
        ts = []
        with open(folder + "/" + file, 'r') as f:
            time = 0
            for line in f:
                line = line.strip()
                if line.startswith("time"):
                    time = line.split(";")[1]
                else:
                    pressure = float(line.split(";")[0])
                    temperature = float(line.split(";")[1])
                    secs = int(line.split(";")[2])
                    times.append(secs)
                    pressures.append(pressure)
                    ts.append(temperature)
        avg_p = np.mean(pressures)
        avg_t = np.mean(ts)
        axis[0].plot(times[1:], pressures[1:])
        axis[0].plot(times[1:], [avg_p for p in pressures][1:])
        axis[0].set_title("pressure")
        axis[0].set_xlabel("time [seconds]")
        axis[0].set_ylabel("pressure [Pa]")
        axis[1].plot(times[1:], ts[1:])
        axis[1].plot(times[1:], [avg_t for t in ts][1:])
        axis[1].set_title("temperature")
        axis[1].set_xlabel("time [seconds]")
        axis[1].set_ylabel("temperature [°C]")
        print(f"average temperature: {avg_t}")
        print(f"average pressure: {avg_p}")
        print(f"determined altitude for {file} is: {get_alt(avg_p, avg_t)}")
    plt.legend()
    plt.show()
