import numpy as np
import LAG
import matplotlib.pyplot as plt

np.random.seed(0)

## ---------------------------------- ##
# Do not change anything above this line
## ---------------------------------- ##

def mean_intra_cluster_distance(spicepoints, labels):
    # spicepoints.shape = (Nx2)  ==> np.ndarray
    # labels.shape = (N,) ==> np.ndarray

    # a.shape = (N,) ==> np.ndarray
    # a(i) = mean intra-cluster distance corresponding to spice point i
    ## TODO1 : Calculate a
    # a = np.random.rand(spicepoints.shape[0]) 
    # a is set (n,) random variable 
    freq = np.bincount(labels) # Count frequency in each bin from 0 to K-1 
    labels_squared = labels**2 # x_i**2 , y_i**2 
    # First the calculate the distances between points 
    # Once braodcast the transpose label , and once broadcast the normal label , where they are equal , replace by distance value , else by 0
    # Now sum along axis 1 , sum of distances , Now to calculate the number of observation , do the following
    # use the count_nonxero function and then divide by that array to obtain the result wished 
    # To calculate the distance 

    # Number of Data Points  
    N = spicepoints.shape[0]

    spicepoints_vertical_stacked = spicepoints.reshape(N,1,2)

    # Distance between points N*N matrix 
    distance = np.sqrt(np.sum((spicepoints_vertical_stacked-spicepoints)**2 , axis = 2))

    # Tranposed labels
    transpose_label_broadcasted = (labels.reshape(N,1)) + np.zeros((N,N))
    # Labels broadcasted
    label_broadcasted = labels.reshape((1,N)) + np.zeros((N,N))
    intra_dis = np.where(label_broadcasted == transpose_label_broadcasted , distance , 0)

    # print(intra_dis[0,:])
    non_zerocount = np.count_nonzero(intra_dis,axis=1)

    # print(non_zerocount)
    non_zerocount[non_zerocount == 0] = 1
    mean_intra_dis = np.sum(intra_dis,axis = 1)/non_zerocount
    return mean_intra_dis.reshape(N,-1)

def generate_plot(data_path:str, max_K:int):
    ## TODO2 : Generate the plot
    ## The file data_path contains the spicepoints with dimensions separated by commas
    ## Check the sample data provided in the spicepoints.csv file (data shape 300x2)
    ## use the LAG function defined in LAG.py to generate labels -- module LAG is imported above
    data = np.loadtxt(data_path, delimiter=',').reshape(-1,2)
    for K in range(2,max_K+1) : 
        _, labels,_ = LAG.LAG(data_path,K)
        distance = mean_intra_cluster_distance(data,labels)
        plt.hist(distance,bins=12,alpha=0.5,label=f"K = {K}")
        plt.legend(loc = "lower center")
    # plt.figure()

    plt.xlabel("mean intra-cluster distance")
    plt.ylabel("frequency")
    plt.savefig("kmeans_hist.png")

    ## DON'T change the following line: You are supposed to return plt object after plotting and saving the figure
    return plt

if __name__ == "__main__":
    # You can test your implementation here
    # To check the mean_intra_cluster_distance
    # We provide some sample data in testcases
    # subfolder sample1 contains 
        # sample1.txt => spicepoints
        # sample1_labels.txt => labels
        # sample1_a.txt => output of mean_intra_cluster_distance
    labels = np.loadtxt("./testcases/sample1/sample1_labels.txt").astype(dtype = int)
    spicepoints = np.loadtxt("./testcases/sample1/sample1.txt",delimiter=",").reshape(-1,2)

    a1 = mean_intra_cluster_distance(spicepoints,labels) 

    spicepoints1 = np.loadtxt("./testcases/sample2/sample2.txt",delimiter=",").reshape(-1,2)
    labels1 = np.loadtxt("./testcases/sample2/sample2_labels.txt").astype(dtype = int)

    a2 = mean_intra_cluster_distance(spicepoints1,labels1)


    generate_plot("spicepoints.txt", 6)
