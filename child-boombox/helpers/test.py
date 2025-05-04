import pandas as pd
from collections import defaultdict
import subprocess

class BarcodeDecoder:
    def __init__(self):
        self.first_pass_median_window_size = 11
        self.second_pass_median_window_size = 5

    def _median_filter(self, data, window_size):
        half = window_size // 2
        filtered = []
        for i in range(len(data)):
            window = []
            for j in range(i - half, i + half + 1):
                if j < 0:
                    window.append(data[0])
                elif j >= len(data):
                    window.append(data[-1])
                else:
                    window.append(data[j])
            window.sort()
            filtered.append(window[half])
        return filtered

    def _normalize_data(self, data):
        min_val = min(data)
        max_val = max(data)
        if max_val == min_val:
            # Avoid division by zero if all values are the same
            return [0.0 for _ in data]
        return [(x - min_val) / (max_val - min_val) for x in data]

    def _restore_index(self, data, window_start, window_idx):
        if window_start < 0:
            return len(data) + window_start + window_idx
        return window_start + window_idx

    def _max_idx(self, data, window_start=0, window_end=None):
        window = data[window_start:window_end]
        max_windowed = max(range(len(window)), key=window.__getitem__)
        return self._restore_index(data, window_start, max_windowed)

    def _min_idx(self, data, window_start=0, window_end=None):
        window = data[window_start:window_end]
        min_windowed = min(range(len(window)), key=window.__getitem__)
        return self._restore_index(data, window_start, min_windowed)

    def _prominence(self, data, left_minimum_idx, maximum_idx, right_minimum_idx):
        return data[maximum_idx] - max(data[left_minimum_idx], data[right_minimum_idx])

    def _find_bit_peak_near_middle(self, data, bit_start, bit_end):
        bit_max_idx = self._max_idx(data, window_start=bit_start, window_end=bit_end)
        bit_max = data[bit_max_idx]
        bit_max_start = bit_max_idx
        bit_max_end = bit_max_idx
        for i in range(bit_start, bit_max_idx):
            if data[i] >= bit_max * 0.9:
                bit_max_start = i
                break

        for i in range(bit_end, bit_max_idx, -1):
            if data[i] >= bit_max * 0.9:
                bit_max_end = i
                break

        bit_middle = bit_start + int((bit_end - bit_start)/2)

        if bit_middle in range(bit_max_start, bit_max_end):
            return bit_middle

        if abs(bit_max_start - bit_middle) < abs(bit_max_end - bit_middle):
            return bit_max_start

        return bit_max_end

    def _compensate_bit_start_plateau(self, data, bit_start, bit_max_idx):
        prominence = data[bit_max_idx] - data[bit_start]
        edge_value = data[bit_start] + 0.1 * prominence

        for i in range(bit_start, bit_max_idx):
            if data[i] >= edge_value:
                minimum_end_edge = i
                break

        minimum_start_edge = 0
        for i in range(bit_start, max(bit_start - bit_max_idx, 0), -1):
            if data[i] >= edge_value:
                minimum_start_edge = i
                break

        minimum_middle = minimum_start_edge + int((minimum_end_edge - minimum_start_edge) / 2)
        return minimum_middle

    def _find_last_bit(self, data):
        n = len(data)
        last_min_boundary = self._max_idx(data, window_start=-int(n * 0.05))
        def find_near_end_local_minimum():
            local_miniumum = self._min_idx(data, window_start=-int(n * 0.2), window_end=last_min_boundary)
            return local_miniumum

        def find_intersections_with_data_to_end(local_minimum):
            local_min_threshold = data[local_minimum] * 1.05
            slopes = []
            for i in range(local_minimum + 1, last_min_boundary):
                if (data[i] <= local_min_threshold and data[i + 1] >= local_min_threshold) or \
                        (data[i] >= local_min_threshold and data[i + 1] <= local_min_threshold):
                    slopes.append(i)
            return slopes

        def find_second_last_min(local_minimum, slopes):
            second_last_min = local_minimum
            if len(slopes) > 2:
                if len(slopes) % 2 == 1:  # last max cut by window
                    second_last_min = min_idx(data, slopes[-2], slopes[-1])
                else:
                    second_last_min = min_idx(data, slopes[-3], slopes[-2])
            return second_last_min

        def compensate_bit_end_signal_end_drop(bit_start, bit_end):
            if data[bit_end] < data[bit_start]:
                for i in range(bit_end, bit_start, -1):
                    if data[i] >= data[bit_start]:
                        return i
            return bit_end

        local_minimum = find_near_end_local_minimum()
        slopes = find_intersections_with_data_to_end(local_minimum)
        bit_start = find_second_last_min(local_minimum, slopes)
        bit_max_idx = self._max_idx(data, window_start=bit_start)
        bit_start = self._compensate_bit_start_plateau(data, bit_start, bit_max_idx)
        bit_end = self._min_idx(data, window_start=bit_max_idx)
        bit_end = compensate_bit_end_signal_end_drop(bit_start, bit_end)

        # sanity checks - bit width, prominence etc
        return (bit_start, self._find_bit_peak_near_middle(data, bit_start, bit_end), bit_end)

    def _find_second_last_bit(self, data, last_bit):
        last_bit_start, last_bit_max_idx, last_bit_end = last_bit
        second_last_bit_end = last_bit_start
        last_bit_width = last_bit_end - last_bit_start
        second_last_bit_max_idx = self._max_idx(data, window_start=last_bit_start - last_bit_width, window_end=second_last_bit_end)
        maximums_width = last_bit_max_idx - second_last_bit_max_idx
        second_last_bit_start = self._min_idx(data, window_start=second_last_bit_max_idx-maximums_width, window_end=second_last_bit_max_idx)

        return (second_last_bit_start, self._find_bit_peak_near_middle(data, second_last_bit_start, second_last_bit_end), second_last_bit_end)

    def _find_previous_bit(self, data, bits):
        (second_last_bit_start, second_last_bit_max_idx, second_last_bit_end) = bits[-2]
        (last_bit_start, last_bit_max_idx, last_bit_end) = bits[-1]
        bit_width = max(second_last_bit_end - second_last_bit_start, last_bit_end - last_bit_start)
        bit_width = int(1.1 * bit_width)
        bit_end = last_bit_start
        bit_start_boundary = max(bit_end - bit_width, 0)
        approximate_bit_max = last_bit_max_idx - (second_last_bit_max_idx - last_bit_max_idx)
        possible_bit_start = self._min_idx(data, window_start=bit_start_boundary, window_end=approximate_bit_max)
        bit_max_idx = self._max_idx(data, window_start=possible_bit_start, window_end=bit_end)
        prominence = data[bit_max_idx] - data[possible_bit_start]
        bit_start = possible_bit_start
        if prominence < 0.1:
            for i in range(bit_start, bit_start_boundary, -1):
                if data[i] <= data[bit_start] - 0.05 or data[i] >= data[bit_start] + 0.05:
                    bit_start = i+1
                    break
            return (bit_start, int(bit_start + (bit_end - bit_start) / 2), bit_end)
        else:
            bit_start = self._compensate_bit_start_plateau(data, possible_bit_start, bit_max_idx)
            return (bit_start, self._find_bit_peak_near_middle(data, bit_start, bit_end), bit_end)


    def decode_barcode(self, data):
        data = self._normalize_data(
            self._median_filter(
                self._median_filter(data, window_size=self.first_pass_median_window_size),
                window_size=self.second_pass_median_window_size))
        n = len(data)
        last_bit = self._find_last_bit(data)
        bits_positions = [last_bit, self._find_second_last_bit(data, last_bit)]

        for i in range(8, 0, -1):
            bits_positions.append(self._find_previous_bit(data, bits_positions))

        decimal_value = 0
        for (bit_start, bit_max_idx, bit_end) in bits_positions:
            bit = 0
            if max(data[bit_start:bit_end]) >= 0.55:
                bit = 1
            decimal_value = decimal_value * 2 + bit

        return decimal_value


# Test z pełnymi danymi
data1 = [147, 147, 148, 159, 160, 162, 160, 173, 172, 174, 185, 184, 184, 187, 184, 186, 185, 185, 186, 184, 173, 175,
         172, 162, 161, 159, 148, 150, 149, 148, 147, 147, 147, 150, 147, 147, 148, 147, 147, 161, 161, 173, 172, 175,
         173, 185, 186, 184, 184, 184, 184, 185, 185, 185, 174, 172, 161, 161, 162, 160, 159, 150, 149, 148, 147, 147,
         147, 149, 148, 148, 147, 147, 161, 161, 161, 160, 172, 172, 173, 172, 175, 186, 184, 185, 184, 184, 184, 186,
         198, 188, 188, 186, 175, 172, 178, 163, 167, 165, 150, 149, 151, 150, 152, 141, 138, 137, 137, 139, 139, 135,
         134, 137, 139, 142, 140, 142, 139, 147, 137, 148, 140, 139, 140, 150, 151, 149, 149, 149, 152, 147, 142, 148,
         139, 142, 139, 138, 136, 137, 136, 139, 136, 139, 138, 138, 136, 137, 138, 127, 136, 137, 138, 137, 136, 142,
         141, 137, 136, 149, 141, 149, 151, 150, 151, 152, 150, 151, 153, 149, 151, 147, 148, 155, 150, 149, 150, 152,
         141, 149, 138, 136, 135, 138, 142, 138, 137, 138, 137, 138, 148, 148, 150, 151, 151, 152, 152, 149, 150, 155,
         162, 160, 161, 164, 176, 174, 175, 185, 189, 193, 188, 201, 199, 200, 197, 197, 202, 197, 199, 200, 203, 200,
         188, 188, 188, 185, 187, 188, 176, 177, 179, 176, 176, 173, 173, 179, 176, 171, 177, 173, 174, 178, 179, 175,
         176, 186, 189, 189, 187, 189, 199, 200, 202, 205, 200, 209, 210, 212, 210, 212, 212, 211, 214, 209, 211, 202,
         199, 198, 201, 201, 188, 191, 188, 188, 190, 188, 177, 175, 174, 175, 172, 176, 173, 175, 165, 173, 164, 162,
         162, 165, 161, 159, 163, 163, 163, 163, 166, 163, 161, 161, 165, 163, 161, 160, 166, 162, 160, 161, 163, 166,
         163, 162, 161, 160, 160, 165, 163, 162, 157, 163, 161, 159, 152, 153, 159, 149, 160, 151, 151, 150, 152, 151,
         162, 162, 163, 161, 161, 160, 164, 172, 175, 168, 175, 176, 175, 185, 188, 190, 201, 201, 209, 200, 203, 211,
         212, 211, 210, 213, 209, 210, 214, 213, 214, 213, 211, 201, 210, 200, 199, 198, 199, 194, 185, 190, 187, 186,
         186, 185, 178, 177, 174, 173, 175, 173, 169, 174, 174, 172, 176, 174, 173, 174, 168, 175, 175, 175, 175, 175,
         175, 177, 186, 187, 188, 187, 200, 198, 198, 207, 202, 203, 200, 200, 202, 213, 210, 206, 201, 211, 210, 210,
         210, 203, 202, 199, 202, 198, 198, 190, 191, 192, 187, 179, 177, 175, 166, 174, 162, 163, 160, 152, 148, 148,
         148, 140, 138, 140]

data2 = [97, 98, 98, 100, 100, 98, 98, 98, 97, 98, 98, 97, 98, 98, 97, 99, 110, 110, 112, 110, 112, 114, 110, 111, 110,
         111, 111, 112, 116, 123, 122, 123, 124, 122, 125, 124, 122, 122, 135, 125, 135, 135, 135, 135, 136, 136, 135,
         125, 127, 122, 123, 124, 122, 123, 121, 122, 112, 112, 110, 111, 110, 110, 110, 111, 110, 112, 110, 111, 112,
         114, 110, 110, 110, 111, 113, 111, 112, 111, 111, 110, 112, 113, 124, 123, 123, 124, 124, 126, 124, 125, 135,
         136, 138, 137, 140, 137, 136, 135, 125, 126, 123, 129, 127, 126, 124, 125, 113, 111, 116, 117, 113, 110, 114,
         111, 111, 114, 115, 116, 111, 116, 117, 111, 114, 113, 113, 113, 111, 123, 115, 124, 124, 126, 128, 129, 124,
         126, 137, 137, 127, 129, 135, 131, 138, 141, 137, 139, 138, 136, 137, 127, 125, 125, 124, 124, 124, 125, 114,
         114, 113, 114, 111, 117, 113, 113, 111, 114, 112, 122, 113, 122, 113, 115, 115, 124, 124, 124, 128, 125, 136,
         131, 136, 136, 135, 144, 151, 148, 152, 161, 161, 159, 163, 162, 167, 172, 172, 180, 176, 188, 190, 186, 192,
         186, 190, 186, 179, 176, 178, 175, 174, 163, 159, 161, 152, 153, 149, 150, 148, 138, 137, 143, 139, 138, 137,
         129, 136, 123, 123, 123, 124, 127, 126, 125, 125, 123, 119, 124, 125, 128, 123, 124, 129, 125, 124, 127, 131,
         136, 138, 136, 136, 138, 137, 138, 138, 138, 140, 137, 134, 136, 139, 138, 136, 137, 136, 137, 131, 137, 139,
         138, 137, 126, 136, 127, 126, 128, 123, 124, 131, 127, 126, 124, 126, 123, 123, 126, 126, 124, 124, 128, 124,
         127, 131, 139, 138, 140, 149, 150, 151, 161, 163, 163, 161, 165, 174, 178, 174, 185, 187, 189, 190, 196, 191,
         190, 186, 188, 175, 177, 180, 172, 164, 165, 162, 163, 161, 152, 148, 153, 148, 147, 151, 148, 148, 148, 149,
         151, 152, 151, 149, 150, 160, 163, 161, 162, 164, 172, 173, 176, 188, 185, 187, 186, 187, 197, 187, 188, 185,
         186, 188, 175, 173, 173, 172, 160, 160, 152, 151, 152, 138, 148, 137, 136, 136, 127, 124, 135, 124, 124, 127,
         123, 123, 123, 124, 125, 125, 127, 137, 135, 137, 129, 134, 135, 130, 141, 137, 138, 139, 140, 138, 137, 138,
         140, 137, 137, 136, 124, 125, 128, 123, 125, 125, 123, 124, 114, 114, 113, 124, 113, 123, 116, 123, 126, 127,
         124, 128, 125, 128, 126, 138, 135, 140, 148, 150, 150, 165, 174, 174, 176, 187, 190, 189, 196, 191, 200, 199,
         189, 188, 178, 173, 168, 165, 160, 164, 152, 148, 143, 138, 135, 139, 128, 126, 125, 124, 121, 123, 123, 123,
         125, 124, 115, 115, 126, 117, 126, 126, 124, 125, 130, 127, 138, 139, 138, 142, 149, 151, 150, 161, 162, 163,
         170, 179, 174, 172, 187, 188, 185, 187, 186, 180, 176, 172, 172, 174, 174, 164, 160, 149, 147, 150, 142, 138,
         136, 136, 136, 125, 124, 128, 114, 112, 113, 114, 110, 102, 100, 102]


decoder = BarcodeDecoder()
assert decoder.decode_barcode(data2) == 872
assert decoder.decode_barcode(data1) == 871
print("ŁO CIĘ KRÓWKO, TEŚCIORY POMYŚLNIE ZAKUŃCZONE")

# Read the CSV file (replace 'filename.csv' with your actual file)
df = pd.read_csv('sampledata_2025-4-26_12.csv')#'sampledata_2025-04-18.csv')

# Sort by NR and SAMPLE_IDX to ensure correct order
df = df.sort_values(by=['NR', 'SAMPLE_IDX'])

# Group DURATION values into lists by NR
nr_duration_dict = defaultdict(list)
for _, row in df.iterrows():
    nr_duration_dict[row['NR']].append(row['DURATION'].item())

#print("data2")
#subprocess.run(["python", "vis.py", f"{data2}"])
#print("data1")
#subprocess.run(["python", "vis.py", f"{data1}"])


# Get the lists in NR order
for nr in sorted(nr_duration_dict.keys()):
    try:
        print(nr)
        subprocess.run(["python", "vis.py", f"{nr_duration_dict[nr]}"])
        #print(decoder.decode_barcode(nr_duration_dict[nr]))
    except Exception as e:
        print(nr_duration_dict[nr])
        raise e