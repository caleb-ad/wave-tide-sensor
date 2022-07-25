import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from statistics import median, stdev
import os
import sys


seconds_in_year = 3600 * 24 * 365


def main(data_path):
    data = []
    for data_file in filter(lambda de: de.is_file(), os.scandir(data_path)):
        try:
            data.append(int(data_file.name[:data_file.name.find('.')], base=16))
        except ValueError:
            pass

    print(f"found {len(data)} well-named files")

    # outlier_file_names(data)
    # file_names_minute_allignment(data, allign=6)
    measurement_density(data)
    plt.show(block=True)


def outlier_file_names(file_name_data):
    med = median(file_name_data)
    dev = stdev(file_name_data)
    outliers = list(
            filter(
                lambda num: abs(num - med) > 4*dev,
                file_name_data
            )
        )
    print(f"found {len(outliers)} outliers")
    file_names_minute_allignment(outliers, allign=10, name="Outlier Allignment")


def file_names_minute_allignment(file_name_data, allign=2, name="File Names % 60"):
    fig, ax = plt.subplots()
    ax.set_title(name)
    N, bins, patches = ax.hist(list(map(lambda num: (num % 3600) / 60.0, file_name_data)), bins=60, range=(0, 60))
    for b, p in zip(bins, patches):
        if b%allign == 0 or (b+1)%allign == 0:
            p.set_facecolor(mcolors.CSS4_COLORS["darkviolet"])


def measurement_density(file_name_data):
    med = median(file_name_data)
    # do outlier rejection based on premise that test lasted less than a year
    corrected = list(filter(lambda num: abs(num - med) < seconds_in_year, file_name_data))
    corrected.sort()
    density = [[],[0]]
    hour_zero = corrected[0] // 3600
    current_hr = hour_zero
    for time in corrected:
        if time // 3600 > current_hr:
            density[0].append(current_hr - hour_zero)
            density[1].append(0)
            current_hr = time // 3600
        density[1][-1] += 1

    density[0].append(current_hr - hour_zero)

    fig, ax = plt.subplots()
    ax.set_title("Measurement density")
    ax.set_xlabel("Hours since measurement began")
    ax.set_ylabel("# Measurements")
    ax.scatter(density[0], density[1])



if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
