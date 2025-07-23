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

def pairwise(t):
    it = iter(t)
    return zip(it,it)

def window(arr, k):
    for i in range(len(arr)-k+1):
        yield arr[i:i+k]

parser = argparse.ArgumentParser(description="Plot a time series from a Python list string.")
parser.add_argument('data', type=str, help='Time series data as a Python list, e.g. "[1,2,3,4]"')
args = parser.parse_args()

data = ast.literal_eval(args.data)
bit_boundaries = None

if True:# or len(bit_boundaries) != 10:
    plt.figure(figsize=(12, 6))
    #plt.plot(smoothened, color='red', linewidth=1, label=f'Smoothened')
    plt.plot(data, color='green', linewidth=2, label='Raw data')
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