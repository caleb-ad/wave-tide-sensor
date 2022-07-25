import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from statistics import median, stdev
import os
import sys

def main(data_path):
    data = []
    for data_file in filter(lambda de: de.is_file(), os.scandir(data_path)):
        try:
            data.append(int(data_file.name[:data_file.name.find('.')], base=16))
        except ValueError:
            pass

    print(f"found {len(data)} well-named files")

    outlier_file_names(data)
    file_names_minute_allignment(data, allign=6)
    plt.show(block=True)


def outlier_file_names(file_name_data):
    med = median(file_name_data)
    dev = stdev(file_name_data)
    outliers = list(
            filter(
                lambda num: abs(num - med) > 3*dev,
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




if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
