import argparse
import ast
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import numpy as np
from scipy.ndimage import median_filter
import math
from heapq import heappush, heappop
import time
import array
from collections import deque
import bisect

SCALE = 1_000  # N decimal places instead of float normalization

def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size)/window_size, mode='valid')

def exponential_moving_average(data, alpha):
    ema = [data[0]]
    for i in range(1, len(data)):
        ema.append(alpha * data[i] + (1 - alpha) * ema[-1])
    return ema

def alpha_trimmed_mean(data, window_size, trim_percent):
    half_window = window_size // 2
    trimmed_data = []
    for i in range(len(data)):
        start = max(0, i - half_window)
        end = min(len(data), i + half_window + 1)
        window = sorted(data[start:end])
        trim_count = int(len(window) * trim_percent)
        trimmed_window = window[trim_count:-trim_count] if trim_count > 0 else window
        trimmed_data.append(np.mean(trimmed_window))
    return trimmed_data

class DataPreprocessor():
    class DerivativeExtrema:
        def __init__(self, window=7, noise_threshold=int(SCALE * 0.03)):
            self.buffer = []
            self.window = window
            self.noise_threshold = noise_threshold
            self.prev_slope = None

        def add_point(self, y):
            self.buffer.append(y)
            if len(self.buffer) < self.window:
                return None  # Not enough data
            if len(self.buffer) > self.window:
                self.buffer.pop(0)

            # Calculate central derivative
            dy = self.buffer[-1] - self.buffer[-self.window]
            if abs(dy) <= self.noise_threshold:
                return None
            current_slope = dy / (self.window - 1)

            extrema_detected = None
            if self.prev_slope is not None:
                # Detect maxima: slope transitions from positive to negative/zero
                if self.prev_slope > 0 and current_slope <= 0:
                    extrema_detected = "max"
                # Detect minima: slope transitions from negative to positive/zero
                elif self.prev_slope < 0 and current_slope >= 0:
                    extrema_detected = "min"

            # Reset state after detection to avoid stale slopes
            if extrema_detected:
                # Keep last (window-1) points for continuity
                self.buffer = self.buffer[-(self.window - 1):]
                self.prev_slope = None  # Critical reset
            else:
                self.prev_slope = current_slope

            return extrema_detected

    def filter_plateaus(self, data, threshold, min_width_ratio=0.10):
        N = len(data)
        look_ahead = max(1, int(N * min_width_ratio))
        plateaus = []
        i = 0

        # Detect potential plateaus using look-ahead check
        while i < N - look_ahead:
            if abs(data[i] - data[i + look_ahead]) <= threshold:
                # Verify full window and expand
                start, end = self.verify_and_expand(data, i, look_ahead, threshold)
                if end - start + 1 >= look_ahead:
                    plateaus.append((start, end))
                    i = end  # Skip processed plateau
            i += 1

        # Merge overlapping plateaus
        plateaus = self.merge_plateaus(plateaus)

        # Return indexes for data with filtered plateaus
        return self.filtered_indexes(data, plateaus, look_ahead, min_width_ratio)

    def verify_and_expand(self, data, i, look_ahead, threshold):
        """Expand plateau boundaries in both directions"""
        base = data[i]
        start = i
        end = i + look_ahead

        # Verify initial window
        for j in range(i, end + 1):
            if abs(data[j] - base) > threshold:
                return (0, 0)  # Invalid plateau

        # Expand left
        while start > 0 and abs(data[start - 1] - base) <= threshold:
            start -= 1

        # Expand right
        while end < len(data) - 1 and abs(data[end + 1] - base) <= threshold:
            end += 1

        return (start, end)

    def merge_plateaus(self, plateaus):
        """Merge overlapping/adjacent plateaus"""
        if not plateaus:
            return []

        plateaus.sort()
        merged = [list(plateaus[0])]

        for current in plateaus[1:]:
            last = merged[-1]
            if current[0] <= last[1] + 1:
                last[1] = max(last[1], current[1])
            else:
                merged.append(list(current))

        return [tuple(p) for p in merged]

    def filtered_indexes(self, data, plateaus, target_width, min_width_ratio):
        """Downsample plateaus to target width"""
        result = []
        prev_end = 0
        plateau_reduction = 0

        for start, end in plateaus:
            plateau_len = end - start + 1
            if plateau_len > target_width:
                plateau_reduction += plateau_len - target_width

        compensated_target_width = int((len(data) - plateau_reduction) * min_width_ratio)

        for start, end in plateaus:
            plateau_len = end - start + 1
            if plateau_len > compensated_target_width:
                plateau_reduction += plateau_len - compensated_target_width

        compensated_target_width = int((len(data) - plateau_reduction) * min_width_ratio)

        for start, end in plateaus:
            # Add non-plateau data
            result.extend(range(prev_end, start))

            # Process plateau
            plateau_len = end - start + 1
            if plateau_len > compensated_target_width:
                # Calculate evenly spaced indices
                step = (plateau_len - 1) / (compensated_target_width - 1)
                indices = [start + int(i * step) for i in range(compensated_target_width)]
                result.extend(i for i in indices)
            else:
                result.extend(range(start, end + 1))

            prev_end = end + 1

        # Add remaining data
        result.extend(range(prev_end, len(data)))
        return result

    def normalize_data(self, data):
        min_val = min(data)
        max_val = max(data)
        length = len(data)
        denom = max_val - min_val  # Single division calculation
        return [math.floor(SCALE * ((x - min_val) / denom)) for x in data]

    def normalize_data_inplace(self, data):
        min_val = min(data)
        max_val = max(data)
        length = len(data)
        denom = max_val - min_val  # Single division calculation
        for i in range(len(data)):
            data[i] = math.floor(SCALE * ((data[i] - min_val) / denom))

        return None

    def min_filter(self, data, window_size=5):
        if window_size % 2 == 0:
            window_size += 1
        half = window_size // 2
        extended_data = [data[0]] * half + data + [data[-1]] * half

        return [min(window) for window in (extended_data[i:i + window_size]
                               for i in range(len(data)))]

    def max_filter(self, data, window_size=5):
        if window_size % 2 == 0:
            window_size += 1
        half = window_size // 2
        extended_data = [data[0]] * half + data + [data[-1]] * half

        return [max(window) for window in (extended_data[i:i + window_size]
                               for i in range(len(data)))]

    def min_max_filter(self, data, window_size=5):
        extrema = self.DerivativeExtrema()

        if window_size % 2 == 0:
            window_size += 1
        half = window_size // 2
        extended_data = [data[0]] * half + data + [data[-1]] * half

        result = []
        mode = "max"
        for i in range(len(data)):
            window = extended_data[i:i + window_size]
            min_val = min(window)
            max_val = max(window)
            val = extrema.add_point(max_val)
            if val is not None:
                if val == "max":
                    mode = "min"
                else:
                    mode = "max"
            if min_val <= int(SCALE*0.1) or mode=="min":
                result.append(min_val)
            else:
                result.append(max_val)
        return result

    def min_max_filter_inplace(self, data, window_size=5):
        extrema = self.DerivativeExtrema()

        if window_size % 2 == 0:
            window_size += 1
        window_half = window_size // 2

        window = []
        mode = "max"

        for idx in range(len(data)):
            window.append(data[idx])
            if len(window) > window_size:
                window.pop(0)

            min_val = min(window)
            max_val = max(window)
            val = extrema.add_point(max_val)
            if val is not None:
                if val == "max":
                    mode = "min"
                else:
                    mode = "max"

            if len(window) == window_size:
                if idx == window_size:
                    for i in range(0, window_half + 1):
                        wmin = min(window[i:i+window_size])
                        data[i] = wmin
                elif idx == len(data) - 1:
                    for i in range(len(data) - window_half - 1, len(data)):
                        window_idx = i - (len(data) - window_size)
                        wmin = min(window[window_idx-window_half:])
                        data[i] = wmin
                if min_val <= int(SCALE * 0.1) or (mode == "min" and max_val <= int(SCALE * 0.7)):
                    data[idx - window_half] = min_val
                else:
                    data[idx - window_half] = max_val

        return None

    def median_filter_bisect(self, data, window_size=11):
        if window_size % 2 == 0:
            window_size += 1
        window_half = window_size // 2
        len_data = len(data)
        data0, data_end = data[0], data[-1]

        result = [0] * len_data  # Pre-allocate
        window = []

        for idx in range(len(data)):
            bisect.insort_left(window, data[idx])
            if len(window) > window_size:
                window.pop(window.index(data[idx-window_size]))

            if len(window) == window_size:
                if idx == window_size:
                    extended_window = [data0] * window_half + data[0:window_size]
                    for i in range(0, window_half + 1):
                        extended_window_slice = extended_window[i:i + window_size]
                        extended_window_slice.sort()
                        result[i] = extended_window_slice[window_half]
                elif idx == len(data) - 1:
                    extended_window = data[-window_size:] + [data_end] * window_half
                    window_idx = 0
                    for i in range(len(data) - window_half - 1, len(data)):
                        extended_window_slice = extended_window[window_idx:window_idx + window_size]
                        extended_window_slice.sort()
                        result[i] = extended_window_slice[window_half]
                        window_idx += 1
                result[idx - window_half] = window[window_half]
        return result

    def median_filter(self, data, window_size=11):
        def insort_left(a, x):
            # Find the index where to insert x to keep a sorted (left-most position)
            lo = 0
            hi = len(a)
            while lo < hi:
                mid = (lo + hi) // 2
                if a[mid] < x:
                    lo = mid + 1
                else:
                    hi = mid
            a.insert(lo, x)

        if window_size % 2 == 0:
            window_size += 1
        window_half = window_size // 2
        len_data = len(data)
        data0, data_end = data[0], data[-1]

        result = [0] * len_data  # Pre-allocate
        window = []

        for idx in range(len(data)):
            insort_left(window, data[idx])
            if len(window) > window_size:
                window.pop(window.index(data[idx-window_size]))

            if len(window) == window_size:
                if idx == window_size:
                    extended_window = [data0] * window_half + data[0:window_size]
                    for i in range(0, window_half + 1):
                        extended_window_slice = extended_window[i:i + window_size]
                        extended_window_slice.sort()
                        result[i] = extended_window_slice[window_half]
                elif idx == len(data) - 1:
                    extended_window = data[-window_size:] + [data_end] * window_half
                    window_idx = 0
                    for i in range(len(data) - window_half - 1, len(data)):
                        extended_window_slice = extended_window[window_idx:window_idx + window_size]
                        extended_window_slice.sort()
                        result[i] = extended_window_slice[window_half]
                        window_idx += 1
                result[idx - window_half] = window[window_half]
        return result

    def median_filter_inplace(self, data, window_size=11):
        def insort_left(a, x):
            # Find the index where to insert x to keep a sorted (left-most position)
            lo = 0
            hi = len(a)
            while lo < hi:
                mid = (lo + hi) // 2
                if a[mid] < x:
                    lo = mid + 1
                else:
                    hi = mid
            a.insert(lo, x)

        if window_size % 2 == 0:
            window_size += 1
        window_half = window_size // 2
        len_data = len(data)
        data0, data_end = data[0], data[-1]

        window = []
        window_raw = []

        for idx in range(len(data)):
            window_raw.append(data[idx])
            insort_left(window, data[idx])
            if len(window) > window_size:
                oldest_window_val = window_raw.pop(0)
                window.pop(window.index(oldest_window_val))


            if len(window) == window_size:
                if idx == window_size:
                    extended_window = [data0] * window_half + data[0:window_size]
                    for i in range(0, window_half + 1):
                        extended_window_slice = extended_window[i:i + window_size]
                        extended_window_slice.sort()
                        data[i] = extended_window_slice[window_half]
                elif idx == len(data) - 1:
                    extended_window = data[-window_size:] + [data_end] * window_half
                    window_idx = 0
                    for i in range(len(data) - window_half - 1, len(data)):
                        extended_window_slice = extended_window[window_idx:window_idx + window_size]
                        extended_window_slice.sort()
                        data[i] = extended_window_slice[window_half]
                        window_idx += 1
                data[idx - window_half] = window[window_half]
        return None


    def dynamic_median_filter(self, data, var_threshold=SCALE*0.7):
        result = data.copy()
        n = len(data)
        if n == 0:
            return result

        current_window = []
        sum_x = 0.0
        sum_x2 = 0.0
        start_idx = 0

        for i in range(n):
            x = data[i]
            current_window.append(x)
            sum_x += x
            sum_x2 += x ** 2
            window_len = len(current_window)

            # Calculate variance
            if window_len >= 2:
                var = (sum_x2 - (sum_x ** 2) / window_len) / (window_len - 1)
            else:
                var = 0.0

            if var > var_threshold:
                # Reject current sample and process window
                rejected = current_window.pop()
                sum_x -= rejected
                sum_x2 -= rejected ** 2
                window_len -= 1

                if window_len >= 15:
                    median = sorted(current_window)[window_len // 2]
                    result[start_idx:start_idx + window_len] = [median] * window_len

                # Start new window with rejected sample
                start_idx = i
                current_window = [rejected]
                sum_x = rejected
                sum_x2 = rejected ** 2

        # Process final window
        window_len = len(current_window)
        if window_len >= 15:
            median = sorted(current_window)[window_len // 2]
            result[start_idx:start_idx + window_len] = [median] * window_len

        return result

    def filter_inplace(self, data, filtered_idx):
        write = 0
        for i in filtered_idx:
            data[write] = data[i]
            write += 1
        # Remove extra elements from the end
        del data[write:]

        return None

    def dynamic_median_filter_inplace(self, data, var_threshold=SCALE*0.7):
        n = len(data)
        if n == 0:
            return

        current_window = []
        sum_x = 0.0
        sum_x2 = 0.0
        start_idx = 0

        for i in range(n):
            x = data[i]
            current_window.append(x)
            sum_x += x
            sum_x2 += x ** 2
            window_len = len(current_window)

            # Calculate variance
            if window_len >= 2:
                var = (sum_x2 - (sum_x ** 2) / window_len) / (window_len - 1)
            else:
                var = 0.0

            if var > var_threshold:
                # Reject current sample and process window
                rejected = current_window.pop()
                sum_x -= rejected
                sum_x2 -= rejected ** 2
                window_len -= 1

                if window_len >= 15:
                    median = sorted(current_window)[window_len // 2]
                    data[start_idx:start_idx + window_len] = [median] * window_len

                # Start new window with rejected sample
                start_idx = i
                current_window = [rejected]
                sum_x = rejected
                sum_x2 = rejected ** 2

        # Process final window
        window_len = len(current_window)
        if window_len >= 15:
            median = sorted(current_window)[window_len // 2]
            data[start_idx:start_idx + window_len] = [median] * window_len

        return None

    def smoothen(self, data):
        normalized = self.normalize_data(self.median_filter(data, 7 + 2 * int(((len(data) - 500) / 300))))
        filtered_idx = self.filter_plateaus(normalized, threshold=int(SCALE*0.02))
        filtered_data = [data[i] for i in filtered_idx]
        window_size = max(5, int(len(filtered_data) / 100))
        result = self.dynamic_median_filter(self.min_max_filter(self.normalize_data(filtered_data), window_size))
        return result

    def smoothen_light(self, data):
        normalized = self.normalize_data(self.median_filter(data, 7 + 2 * int(((len(data) - 500) / 300))))
        filtered_idx = self.filter_plateaus(normalized, threshold=int(SCALE*0.02))
        filtered_data = [data[i] for i in filtered_idx]
        result = self.normalize_data(filtered_data)
        return result

    def smoothen_inplace(self, data):
        normalized = self.normalize_data(self.median_filter(data, 7 + 2 * int(((len(data) - 500) / 300))))
        filtered_idx = self.filter_plateaus(normalized, threshold=int(SCALE*0.02))
        self.filter_inplace(data, filtered_idx)
        window_size = max(5, int(len(filtered_idx) / 150))
        self.normalize_data_inplace(data)
        window_size = max(5, int(len(data) / 100))
        self.min_max_filter_inplace(data, window_size=window_size)
        self.median_filter_inplace(data, window_size=5)
        self.dynamic_median_filter_inplace(data)
        return None

parser = argparse.ArgumentParser(description="Plot a time series from a Python list string.")
parser.add_argument('data', type=str, help='Time series data as a Python list, e.g. "[1,2,3,4]"')
args = parser.parse_args()

data = ast.literal_eval(args.data)

# --- Parameters ---
window_length = 11  # Must be odd and <= len(data)
polyorder = 2
alpha = 0.3
trim_percent = 0.1

# --- Apply Filters ---
# savgol_smoothed = savgol_filter(data, window_length=window_length, polyorder=polyorder)
# moving_avg = moving_average(data, window_length)
# ema = exponential_moving_average(data, alpha)
# median_smoothed = median_filter_custom2(data, window_length)
# alpha_smooth = alpha_trimmed_mean(data, window_length, trim_percent)

def pairwise(t):
    it = iter(t)
    return zip(it,it)

def window(arr, k):
    for i in range(len(arr)-k+1):
        yield arr[i:i+k]

class BarcodeDecoder():
    def expand_left(self, data, left, window_size, window_size_threshold, shrink_edge_plateaus):
        expanded_left = left
        last_changed = left - 1
        for i in range(max(0, left - 1), 0, -1):
            if data[i] > data[i + 1]:
                if data[i] > data[i+1] + int(SCALE*0.025):
                    break
                last_changed = i
                if min(data[max(0, i - window_size):i - 1]) >= data[i]:
                    break
            elif data[i] < data[i+1]:
                last_changed = i
            if data[i] < window_size_threshold and (last_changed - i) > window_size:
                break
            expanded_left = i

        if shrink_edge_plateaus:
            for i in range(expanded_left, left):
                if data[i + 1] > data[i]:
                    break
                expanded_left = i
        return expanded_left

    def expand_right(self, data, right, window_size, window_size_threshold, shrink_edge_plateaus):
        expanded_right = right
        last_changed = right + 1
        for i in range(min(right + 1, len(data) - 1), len(data)):
            if data[i] > data[i - 1]:
                if data[i] > data[i - 1] + int(SCALE*0.025):
                    break
                last_changed = i
                if min(data[i + 1:min(i + window_size, len(data))]) >= data[i]:
                    break
            elif data[i] < data[i - 1]:
                last_changed = i
            if data[i] < window_size_threshold and (i - last_changed) > window_size:
                break
            expanded_right = i

        if shrink_edge_plateaus:
            for i in range(expanded_right, right, -1):
                if data[i - 1] > data[i]:
                    break
                expanded_right = i
        return expanded_right

    def extend_bit_range(self, data, left, right, window_size_threshold, shrink_edge_plateaus):
        window_size = int(len(data)*0.025)
        return (self.expand_left(data, left, window_size, window_size_threshold, shrink_edge_plateaus),
                self.expand_right(data, right, window_size, window_size_threshold, shrink_edge_plateaus))

    def find_intersections(self, data, threshold, locked_idx_pairs):
        def is_index_locked(index):
            for start, end in locked_idx_pairs:
                if start <= index <= end:
                    return (True, end)
            return (False, None)

        intersections = []
        idx = 1
        while idx < len(data):
            (locked, lock_end) = is_index_locked(idx)
            if locked:
                idx = lock_end + 1
                continue
            if data[idx] >= threshold and data[idx-1] < threshold or \
                data[idx] <= threshold and data[idx-1] > threshold:
                intersections.append(idx)
            idx += 1

        return intersections

    def bit_width(self, bit):
        [left, right] = bit
        return right - left

    def adjust_boundaries(self, data, bits):
        bits = sorted(bits)

        for [previous, next] in window(bits, 2):
            [_, prev_right] = previous
            [next_left, _] = next
            if prev_right > next_left:
                diff = prev_right - next_left
                previous[1] = prev_right - int(diff / 2)
                next[0] = previous[1] + 1
            elif next_left > prev_right:
                diff = next_left - prev_right
                min_surrounding_bit_width = min(self.bit_width(previous), self.bit_width(next))
                if diff > 1 and (diff < 0.2 * min_surrounding_bit_width or \
                        (diff < 0.8 * min_surrounding_bit_width and min(data[prev_right:next_left]) == max(data[prev_right:next_left]))):
                    previous[1] = prev_right + int(diff / 2)
                    next[0] = previous[1] + 1

        return bits

    def find_bits(self, data):
        def find_confident_ones_bits():
            bits = []
            intersections = self.find_intersections(data, int(SCALE*0.75), [])
            for (left, right) in pairwise(intersections):
                (expanded_left, expanded_right) = self.extend_bit_range(data, left, right, int(SCALE*0.6), shrink_edge_plateaus=True)
                bits.append([expanded_left, expanded_right])

            intersections = self.find_intersections(data, int(SCALE*0.58), bits)
            for (left, right) in pairwise(intersections):
                (expanded_left, expanded_right) = self.extend_bit_range(data, left, right, int(SCALE*0.6), shrink_edge_plateaus=True)
                bits.append([expanded_left, expanded_right])
            return bits

        def find_starting_bits(first_bit):
            bits = []
            [first_left, _] = first_bit
            if first_left > 0 and max(data[0:first_left]) > data[first_left] * 1.1:
                threshold = 0.7 * max(data[0:first_left])
                intersections = self.find_intersections(data, threshold, [[first_left, len(data)]])
                for (left, right) in pairwise(intersections):
                    (expanded_left, expanded_right) = self.extend_bit_range(data, left, right, 0.3 * max(data[0:first_left]),
                                                                       shrink_edge_plateaus=True)
                    bits.append([expanded_left, expanded_right])
            return bits

        def find_certain_middle_bits(data, prev_right, next_left, min_bit_width):
            middle_bits = []
            prominence = max(data[prev_right+1: next_left-1]) - max(data[prev_right], data[next_left])
            if prominence > int(SCALE*0.05):
                threshold = max(data[prev_right], data[next_left]) + prominence * 0.5
                intersections = self.find_intersections(data, threshold, [[0, prev_right], [next_left, len(data)]])
                for (left, right) in pairwise(intersections):
                    (expanded_left, expanded_right) = self.extend_bit_range(data, left, right, min(data[prev_right], data[next_left]) * 1.1, shrink_edge_plateaus=False)
                    middle_bits.append([expanded_left, expanded_right])
            return middle_bits

        def expand_surrounding_bits_and_find_middle_bits(prev_right, next_left):
            expanded_prev_right = self.expand_right(data, prev_right,
                                               window_size=int(len(data) * 0.025),
                                               window_size_threshold=min(data[prev_right:next_left]) * 1.1,
                                               shrink_edge_plateaus=True)
            expanded_next_left = self.expand_left(data, next_left,
                                             window_size=int(len(data) * 0.025),
                                             window_size_threshold=min(data[prev_right:next_left]) * 1.1,
                                             shrink_edge_plateaus=True)
            if expanded_prev_right < expanded_next_left and expanded_next_left - expanded_prev_right > int(
                    0.8 * min_bit_width):
                return find_certain_middle_bits(data, expanded_prev_right, expanded_next_left, min_bit_width)
            return []

        def merge_too_short_bits(middle_bits, min_bit_width, max_bit_width):
            i = 0
            while i < len(middle_bits) - 1:
                mid_bit_prev = middle_bits[i]
                mid_bit_next = middle_bits[i + 1]
                bit_width_prev = self.bit_width(mid_bit_prev)
                bit_width_next = self.bit_width(mid_bit_next)
                if bit_width_prev < 0.8 * min_bit_width and bit_width_next < 0.8 * min_bit_width and bit_width_prev + bit_width_next <= max_bit_width:
                    [mid_bit_prev_left, mid_bit_prev_right] = mid_bit_prev
                    [mid_bit_next_left, mid_bit_next_right] = mid_bit_next
                    if mid_bit_next_left - mid_bit_prev_right <= 1 or (
                            min(data[mid_bit_prev_right:mid_bit_next_left]) ==
                            max(data[mid_bit_prev_right:mid_bit_next_left])):
                        joined_bit = [mid_bit_prev_left, mid_bit_next_right]
                        mid_bits[i:i + 2] = [joined_bit]
                        continue
                i += 1
            return middle_bits

        def find_bits_between(previous, next, min_bit_width, max_bit_width):
            middle_bits = []
            [_, prev_right] = previous
            [next_left, _] = next
            if next_left > prev_right and next_left - prev_right > int(0.8 * min_bit_width):
                middle_bits = find_certain_middle_bits(data, prev_right, next_left, min_bit_width)
                if len(middle_bits) == 0:
                    middle_bits = expand_surrounding_bits_and_find_middle_bits(prev_right, next_left)
            return middle_bits

        def find_plateau_bits(found_bits, min_bit_width, max_bit_width):
            plateau_bits = []
            non_bit_spaces = []
            for [previous, next] in window(found_bits, 2):
                [_, prev_right] = previous
                [next_left, _] = next
                if next_left - prev_right > 1:
                    non_bit_spaces.append((next_left - prev_right, prev_right + 1, next_left))
            non_bit_spaces = sorted(non_bit_spaces, key=lambda x: x[0], reverse=True)
            for i in range(0, 10 - len(found_bits)):
                for idx, non_bit_space in enumerate(non_bit_spaces):
                    (width, left, right) = non_bit_space
                    if width >= 0.9 * min_bit_width and width <= 1.1 * max_bit_width:
                        plateau_bits.append([left, right])
                        non_bit_spaces.pop(idx)
                        break

            return plateau_bits


        all_bits = find_confident_ones_bits()
        all_bits = self.adjust_boundaries(data, all_bits)
        all_bits.extend(find_starting_bits(all_bits[0]))
        all_bits = self.adjust_boundaries(data, all_bits)

        for [previous, next] in window(all_bits, 2):
            min_bit_width = min(self.bit_width(bit) for bit in all_bits)
            max_bit_width = max(self.bit_width(bit) for bit in all_bits)
            mid_bits = find_bits_between(previous, next, min_bit_width, max_bit_width)
            mid_bits = merge_too_short_bits(mid_bits, min_bit_width, max_bit_width)
            all_bits.extend(mid_bits)

        all_bits = self.adjust_boundaries(data, all_bits)

        if len(all_bits) < 10:
            min_bit_width = min(self.bit_width(bit) for bit in all_bits)
            max_bit_width = max(self.bit_width(bit) for bit in all_bits)
            all_bits.extend(find_plateau_bits(all_bits, min_bit_width, max_bit_width))

        return tuple(r for r in reversed(sorted(all_bits)))

    def decode_barcode(self, data, bits):
        number = 0
        binary_number = ""
        for idx, (left, right) in enumerate(bits):
            bit_height_diff = max(data[left:right]) - min(data[left:right + 1])
            bit = 1 if bit_height_diff > int(SCALE*0.4) or max(data[left:right]) > int(SCALE*0.65) else 0
            if idx == 0 or idx == 1:
                if bit != 1:
                    raise Exception("First 2 bits should be always 1")
            binary_number += str(bit)
            number = number*2 + bit
        return number


start = round(time.time() * 1000)
preprocessor = DataPreprocessor()
smoothened_light = preprocessor.smoothen_light(data)
smoothened = preprocessor.smoothen(data)
preprocessor.smoothen_inplace(data)
smoothened_inplace = data
decoder = BarcodeDecoder()
try:
    bit_boundaries = decoder.find_bits(smoothened_inplace)
except Exception as e:
    bit_boundaries = None
    print("OH NO", e)
end = round(time.time() * 1000)
try:
    print(f"Processing time: {end - start} ms, decoded number: {decoder.decode_barcode(smoothened_inplace, bit_boundaries)}")
except Exception as e:
    print("OH NO", e)

if True:# or len(bit_boundaries) != 10:
    plt.figure(figsize=(12, 6))
    #plt.plot(smoothened, color='red', linewidth=1, label=f'Smoothened')
    plt.plot(smoothened_inplace, color='green', linewidth=2, label='Inplace smooth')
    plt.plot(smoothened_light, color='red', linewidth=1, label=f'Light smooth')
    if bit_boundaries is not None:
        cmap = plt.colormaps['hsv']
        N = len(bit_boundaries)
        last_right_boundary = 0
        for idx, (boundary_left, boundary_right) in enumerate(bit_boundaries):
            if idx % 2 == 1:
                plt.axvspan(xmin=boundary_left, xmax=boundary_right, ymin=0.99, color=cmap(idx/N), label=f'bit {idx}')
            else:
                plt.axvspan(xmin=boundary_left, xmax=boundary_right, ymax=0.01, color=cmap(idx / N), label=f'bit {idx}')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.title('Raw Data vs. Smoothed Data')
    plt.legend()
    plt.grid(True)
    plt.show()