import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    folder = "data2"
    figure, axis = plt.subplots(1, 2, figsize=(10, 5))

    files  = ["pa.log", "ap.log"]

    avgs = []
    times = []
    pressures = []
    ts = []
    for file in files:
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
                    if file == "ap.log" and secs == 1:
                        last_sec = times[-1]
                    if file == "ap.log":
                        times.append(secs + last_sec)
                    else:
                        times.append(secs)
                    pressures.append(pressure)
                    ts.append(temperature)
        avg_p = np.mean(pressures)
        avg_t = np.mean(ts)
        axis[0].plot(times[1:], pressures[1:])
        axis[0].set_title("tlak")
        axis[0].set_xlabel("čas [sekunde]")
        axis[0].set_ylabel("tlak [Pa]")
        axis[1].plot(times[1:], ts[1:])
        axis[1].set_title("temperatura")
        axis[1].set_xlabel("čas [sekunde]")
        axis[1].set_ylabel("temperatura [°C]")
    plt.legend()
    plt.show()
