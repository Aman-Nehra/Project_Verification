'''
Lab 7: ADVANCED PYTHON
Activity 1 : LISAN-AL-GAIB (Dune Reference)
Author : Sabyasachi Samantaray

Here we will be dealing only with 2D data to aid visualisation

In this activity, your aim will be the following.
1) Implement the kmeans algorithm completely and correctly.
2) Implement all TODOs without using any loops
'''

### TODO 1: Importing the necessary libraries - numpy, matplotlib and time
### This TODO is already done
import time # to time the execution
import numpy as np
import matplotlib.pyplot as plt
### TODO 1

### TODO 2
### Load data from data_path
### Check the input file spice_locations.txt to understand the Data Format
### Return : np array of size Nx2
def load_data(data_path):
    data = np.loadtxt(data_path,delimiter=",").reshape(-1,2)
    return data 

### TODO 3.1
### If init_centers is None, initialize the centers by selecting K data points at random without replacement
### Else, use the centers provided in init_centers
### Return : np array of size Kx2
def initialise_centers(data, K, init_centers=None):
    if init_centers == None :
        rng = np.random.default_rng()
        arr = rng.choice(data,size=K,replace=False)
        return arr
    else :
        return init_centers

### TODO 3.2
### Initialize the labels to all ones to size (N,) where N is the number of data points
### Return : np array of size N
def initialise_labels(data):
    return np.ones(data.shape[0])

### TODO 4.1 : E step
### For Each data point, find the distance to each center
### Return : np array of size NxK
def calculate_distances(data, centers):
    # N*2 , K*2 
    # Calculate for x , y , then sqrt(x**2 + y**2)
    center_transpose = np.transpose(centers)
    # sqrt(xi-ki**2 + yi-ki**2)
    return np.sqrt((data[:,0].reshape(data.shape[0],1) - center_transpose[0,:])**2 +  (data[:,1].reshape(data.shape[0],1)-center_transpose[1,:])**2 ) 

### TODO 4.2 : E step
### For Each data point, assign the label of the nearest center
### Return : np array of size N
def update_labels(distances):
    # min_distance = np.min(distances,axis=1)
    # min_distance_reshaped = min_distance.reshape(distances.shape[0],1) + np.zeros(distances.shape)
    # labels = np.where(distances == min_distance_reshaped)
    # return labels[1] # It is returning a 1*N matrix i.e (N,) , that is it returns the column where the condition matches
    labels = np.argmin(distances,axis = 1)
    return labels 



### TODO 5 : M step
### Update the centers to the mean of the data points assigned to it
### Return : np array of size Kx2
def update_centers(data, labels, K):
    freq = np.bincount(labels)
    print(freq)
    mean_x = np.bincount(labels, weights = data[:,0]) / freq
    mean_y = np.bincount(labels, weights = data[:,1]) / freq
    return np.column_stack((mean_x,mean_y))

### TODO 6 : Check convergence
### Check if the labels have changed from the previous iteration
### Return : True / False
def check_termination(labels1, labels2):
    if np.array_equal(labels1,labels2) : 
        return True
    else :
        return False

### DON'T CHANGE ANYTHING IN THE FOLLOWING FUNCTION
def kmeans(data_path:str, K:int, init_centers):
    '''
    Input :
        data (type str): path to the file containing the data
        K (type int): number of clusters
        init_centers (type numpy.ndarray): initial centers. shape = (K, 2) or None
    Output :
        centers (type numpy.ndarray): final centers. shape = (K, 2)
        labels (type numpy.ndarray): label of each data point. shape = (N,)
        time (type float): time taken by the algorithm to converge in seconds
    N is the number of data points each of shape (2,)
    '''
    data = load_data(data_path)    
    centers = initialise_centers(data, K, init_centers)
    labels = initialise_labels(data)


    start_time = time.time() # Time stamp 

    while True:
        # visualise("spice_locations2.txt",labels,centers)
        distances = calculate_distances(data, centers)
        labels_new = update_labels(distances)
        centers = update_centers(data, labels_new, K)
        if check_termination(labels, labels_new): break
        else: labels = labels_new
 
    end_time = time.time() # Time stamp after the algorithm ends
    return centers, labels, end_time - start_time 


### FILL THE LINES BELOW THE TODO COMMENTS
def visualise(data_path, labels, centers):
    data = load_data(data_path)

    # Scatter plot of the data points
    plt.scatter(data[:, 0], data[:, 1], c=labels, s=50, cmap='viridis')
    plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)

    ### TODO 7 : In conclusion
    ### Set title as 'K-means clustering'
    plt.title("K-means clustering")
    ### Set xlabel as 'Longitude'
    plt.xlabel("Longitude")
    ### Set ylabel as 'Latitude'
    plt.ylabel("Latitude")
    ### Save the plot as 'kmeans.png'
    plt.savefig("kmeans.png")
    plt.show()
    ## DO NOT CHANGE THE FOLLOWING LINE
    return plt

center , labels , _ = kmeans("spice_locations2.txt",4,None)
visualise("spice_locations2.txt",labels,center)