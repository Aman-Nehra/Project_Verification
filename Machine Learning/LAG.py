'''
This is the Lisan-Al-Gaib algorithm. The algorithm is used to cluster spice points into K clusters.
The spice points are in 2D space. The spice points are provided as input. The spice points are stored in a file.
The function LAG() returns the final spice centers and the labels for each spice point after the clustering.
'''
import time
import numpy as np
import matplotlib.pyplot as plt

def update_centers(data, labels, K):
    labels_one_hot = np.zeros((len(data), K))
    labels_one_hot[np.arange(len(data)), labels] = 1
    count = np.sum(labels_one_hot, axis=0).reshape(K,1)
    count[count==0] = 1
    centers = np.dot(labels_one_hot.T, data)/count
    return centers

def LAG(data_path:str, K:int):
    data = np.loadtxt(data_path, delimiter=',').reshape(-1,2)
    centers = data[np.random.choice(len(data), K, replace=False)]
    labels = np.zeros(len(data), dtype=int)

    start_time = time.time()

    while True:
        distances = np.linalg.norm(data[:, np.newaxis] - centers, axis=2)
        labels_new = np.argmin(distances, axis=1)
        centers = update_centers(data, labels_new, K)
        if np.all(labels == labels_new): break
        else: labels = labels_new
 
    end_time = time.time()
    return centers, labels, end_time - start_time 