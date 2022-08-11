import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from statistics import median, stdev, mean
from collections import deque
from math import sqrt
import time
import os
import sys


seconds_in_year = 3600 * 24 * 365


class datum:
    def __init__(self, time, dist, temp, hum):
        self.time = time
        self.dist = dist
        self.temp = temp
        self.hum  = hum

    def __repr__(self):
        return "{}, {}, {}, {}".format(self.time, self.dist, self.temp, self.hum)

    def __iadd__(self, other):
        self.time += other.time
        self.dist += other.dist
        self.temp += other.temp
        self.hum  += other.hum
        return self

    def apply(self, func):
        self.time = int(func(self.time))
        self.dist = int(func(self.dist))
        self.temp = float(func(self.temp))
        self.hum  = float(func(self.hum))

    #TODO do correction
    def dist_corrected(self):
        return self.dist

    def get_dist(d, assign=None):
        if assign is not None:
            d.dist = assign
        return d.dist

    def get_temp(d, assign=None):
        if assign is not None:
            d.temp = assign
        return d.temp

    def get_time(d):
        return d.time

    def from_str(data_str):
        data_str = data_str.split(", ")
        time = int(data_str[0][:data_str[0].find('.')])
        dist = int(data_str[1])
        temp = float(data_str[2])
        hum = float(data_str[3])
        return datum(time, dist, temp, hum)


def main(data_path, multiple_sets, file_names, temp_analysis, stilltek_path):
    data_files = []
    data = []
    analysis_dim = datum.get_temp if temp_analysis else datum.get_dist

    if stilltek_path is not None:
        data.append(get_stilltek(stilltek_path))
        if not multiple_sets:
            graph_data(data[0], dim=analysis_dim)

    if multiple_sets:
        for data_folder in filter(lambda de: not de.is_file(),  os.scandir(data_path)):
            for sub_data_folder in filter(lambda de: not de.is_file(), os.scandir(data_folder.path)):
                if sub_data_folder.name == "Data":
                    raw = remove_outliers(collect_data(sub_data_folder.path), key=datum.get_time)
                    # data.append(raw)
                    data.append(smooth(remove_outliers_by_delta(condense(raw, 5, key=datum.get_time), key=analysis_dim)))

        graph_data_compare(data, color_list=["blue", "crimson", "purple", "lime"], marker_list=['D', '<', '^', '>', 'v'], dim=analysis_dim)
        # variation_statistics(data, 6 * 60, 2 * 60)

    if file_names:
        data_files = collect_filenames(data_path)
        outlier_file_names(data_files)
        file_names_minute_allignment(data_files, allign=6)
        measurement_density(data_files)
        print(f"found {len(data_files)} well-named files")

    if not multiple_sets and not file_names and stilltek_path is None:
        graph_data(remove_outliers(collect_data(data_path), key=datum.get_time), dim=analysis_dim)

    plt.show(block=True)


def collect_filenames(path):
    data_files = []
    for data_file in filter(lambda de: de.is_file(), os.scandir(path)):
        try:
            data_files.append(int(data_file.name[:data_file.name.find('.')], base=16))
        except ValueError:
            pass
    return data_files


# read data from folder of data files
def collect_data(path):
    data = []
    for data_file in filter(lambda de: de.is_file(), os.scandir(path)):
        data.extend(get_data(data_file.path))
    return data


# read data from file
def get_data(file_name):
    data = []
    with open(file_name) as data_file:
        for line in data_file.readlines():
            data.append(datum.from_str(line))
    return data


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
    corrected = remove_outliers(file_name_data)
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


def remove_outliers(wave_data, key=lambda x: x):
    med = median(map(lambda point: key(point), wave_data))
    # do outlier rejection based on premise that test lasted less than a year
    corrected = list(filter(lambda num: abs(key(num) - med) < seconds_in_year, wave_data))
    return corrected


def remove_outliers_by_delta(data, key=lambda x: x):
    # when BUF_SIZE is small, delta_buf_squre_sum may become less than 0 due to floating point imprecision,
    # in this case sqrt() will throw an error
    BUF_SIZE = 40
    delta_buf = deque([abs(key(data[i]) - key(data[i-1])) for i in range(1, min(BUF_SIZE, len(data)))])
    delta_buf_sum = sum(delta_buf)
    delta_buf_squares = deque(map(lambda x:(x - delta_buf_sum / BUF_SIZE) * (x - delta_buf_sum / BUF_SIZE), delta_buf))
    delta_buf_square_sum = sum(delta_buf_squares)
    corrected = []
    prev = key(data[0])
    for datum in data:
        datum_key = key(datum)
        next_delta = abs(datum_key - prev)
        current_mean = delta_buf_sum / BUF_SIZE
        if next_delta <= current_mean + sqrt(delta_buf_square_sum) / BUF_SIZE :
            corrected.append(datum)
        delta_buf_sum -= delta_buf.popleft()
        delta_buf_square_sum -= delta_buf_squares.popleft()
        delta_buf.append(next_delta)
        delta_buf_sum += next_delta
        delta_buf_squares.append((next_delta - current_mean) * (next_delta - current_mean))
        delta_buf_square_sum += delta_buf_squares[-1]
        prev = datum_key

    return corrected


def graph_data(wave_data, dim=datum.get_dist):
    start = wave_data[0].time
    times = list(map(lambda data: data.time - start, wave_data))
    dims = list(map(dim, wave_data))
    fig, ax = plt.subplots()

    if dim is datum.get_dist:
        ax.set_title("Distance to surface V.S. time")
        ax.set_xlabel("Unix Time [seconds]")
        ax.set_ylabel("Distance [mm]")
    else:
        ax.set_title("Temperature V.S. time")
        ax.set_xlabel("Unix Time [seconds]")
        ax.set_ylabel("Temperature [F]")

    ax.scatter(times, dims, s=3, linewidths=.1, color="teal")
    # plt.plot(times, dists, linestyle='-')


def graph_data_compare(wave_data_sets, dim=datum.get_dist, color_list=[], marker_list=[]):
    fig, ax = plt.subplots()
    if dim is datum.get_dist:
        ax.set_title("Distance to surface V.S. time")
        ax.set_xlabel("Unix Time [seconds]")
        ax.set_ylabel("Distance [mm]")
    else:
        ax.set_title("Temperature V.S. time")
        ax.set_xlabel("Unix Time [seconds]")
        ax.set_ylabel("Temperature [F]")

    for (wave_data, idx) in zip(wave_data_sets, range(len(wave_data_sets))):
        times = [data.time for data in wave_data]
        dims = [dim(data) for data in wave_data]
        marker = marker_list[idx] if idx < len(marker_list) else '.'
        color = color_list[idx] if idx < len(color_list) else "teal"
        ax.scatter(times, dims, s=3, linewidths=.3, marker=marker, color=color)
    # plt.plot(times, dists, linestyle='-')


#condensing in time tends to remove outliers in space, when a box measurement briefly jumps to a large value for a portion of a measurement period
def condense(data, threshhold, key=lambda x:x):
    current = data[0]
    current_size = 1
    condensed = []
    for idx in range(1, len(data)):
        if abs(key(data[idx]) - key(data[idx - 1])) < threshhold:
            current += data[idx]
            current_size += 1
        else:
            if current_size > 0:
                current.apply(lambda x: x / current_size)
                condensed.append(current)
            current = data[idx]
            current_size = 1

    return condensed


def smooth(data, dim=datum.get_dist):
    BUF_SIZE = 6
    avg_buf = deque(map(dim, data[:BUF_SIZE]))
    avg_buf_sum = sum(avg_buf)
    smoothed = []
    for i in range(min(BUF_SIZE, len(data))):
        smoothed.append(data[i])
        dim(smoothed[-1], assign=avg_buf_sum / BUF_SIZE)
        avg_buf_sum -= avg_buf.popleft()
        avg_buf_sum += dim(data[i])
        avg_buf.append(dim(data[i]))
    avg_buf = deque(map(dim, data[:BUF_SIZE]))
    avg_buf_sum = sum(avg_buf)
    for i in range(min(BUF_SIZE, len(data)), len(data)):
        smoothed.append(data[i - 1])
        dim(smoothed[-1], assign=avg_buf_sum / BUF_SIZE)
        avg_buf_sum -= avg_buf.popleft()
        avg_buf_sum += dim(data[i])
        avg_buf.append(dim(data[i]))
    return smoothed


def tare(data):
    d_mean = mean(map(lambda d: d.dist, data))
    for datum in data: datum.dist -= d_mean
    return data


def get_all_diffs(data):
    diffs = []
    for a in range(len(data)):
        for b in range(len(data)):
            if a != b:
                diffs.append(abs(a-b))
    return diffs


def variation_statistics(wave_data_sets, allignment, read_time):
    SEARCH_WIDTH = 15
    wave_data_sets = [tare(wave_data) for wave_data in wave_data_sets]
    diffs = []
    times = []
    non_empty_bucket = True
    time_bucket = (wave_data_sets[0][0].time // 60) * 60
    prev_idx = 0
    while non_empty_bucket:
        non_empty_bucket = False
        min_next_idx = None
        current = []
        for list_idx in range(len(wave_data_sets)):
            for search_idx in range(prev_idx, min(prev_idx + SEARCH_WIDTH, min([len(wave_data) for wave_data in wave_data_sets]))):
                if abs(wave_data_sets[list_idx][search_idx].time - time_bucket) < read_time // 2:
                    current.append(wave_data_sets[list_idx][search_idx].time)
                    non_empty_bucket = True
                    if min_next_idx is None or min_next_idx > search_idx: min_next_idx = search_idx
        current = get_all_diffs(current)
        diffs.extend(current)
        times.extend([time_bucket] * len(current))
        time_bucket += allignment
        prev_idx = min_next_idx

    fig, ax = plt.subplots()
    ax.set_title("Measurement difference by time interval")
    ax.set_xlabel(f"Unix Time [seconds]")
    ax.set_ylabel("Diff [mm]")
    ax.annotate(f"max diff: {max(diffs)}\nmin diff: {min(diffs)}\nmean diff: {mean(diffs)}", xy=(0, 1), xycoords="axes fraction")
    ax.scatter(times, diffs, linewidths=.1)


def get_stilltek(csv_path):
    data = []
    with open(csv_path) as data_file:
        data_file.readline() #header line
        for line in data_file.readlines():
            values = line.split(',')
            m_time = time.strptime(values[0], "20%y-%m-%d %H:%M:%S")
            data.append(datum(
                # mktime expects value in local time, PST. The stilltek data and our data is in UTC
                # so we need to roll back the result of mktime to get a value in UTC
                int(time.mktime(m_time) - 7 * 3600),
                int(-1.0 * float(values[1]) * 304.79999024640034), #millimeters,
                float(values[2]), #farenheight,
                -1
            ))
    return data


#Example command to plot a single data set in D:/Data:
#python plotm.py "D:\Data"
#Example command to compare multiple data sets in "Data" with stilltek data located in "Data/CalPoly Tide Gauge.csv"
#python plotm.py Data -m -s "Data/CalPoly Tide Gauge.csv"
if __name__ == "__main__":
    if len(sys.argv) > 1:
        multiple_flag = "-m" in sys.argv
        file_name_analysis_flag = "-f" in sys.argv
        temperature_analysis = "-t" in sys.argv
        stilltek_path = None
        try:
            stilltek_path = sys.argv[sys.argv.index("-s") + 1]
        except(ValueError): pass
        main(sys.argv[1], multiple_flag, file_name_analysis_flag, temperature_analysis, stilltek_path)

