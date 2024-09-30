# Aman Nehra - 23b1051
# Haresh Gupta - 23b0931
# Sarthak Dhanajay Thakare - 23b0933
# Question Number 2 , Task C

import numpy as np
import matplotlib.pyplot as plt 
import scipy.stats as sp
 
# ************************ Making the sample function ********************
def sample(loc , scale , N) :
    
    # This calucaltes the Y = F_X(X) that is a uniform distributed random variable
    CDF = np.random.rand(N,)

    # This calculates the X = F_X^-1(Y) which yields the random variable that follows teh Gaussian Distribution
    # Since the norm.ppf(CDF) generates the X for a standard normal distribution we need to adjust scaling
    # that is n=multiply it by sigma and add mean , to get the desired random variable X 
    X = scale*sp.norm.ppf(CDF) + loc 

    # returning the sampled distribution 
    return X 



# ************************ Sampling the Gaussian for the given paremeters *******************
N = int(1e5) 
mean = [0,0,0,-2]
var = [0.2,1,5,0.5]

# This stores the result
result = np.empty((4,N))

for i in range(len(mean)) :

    result[i,:] = sample(mean[i],np.sqrt(var[i]),N)
    

# ******************** Plotting the samples *****************
colors = ['blue','red','yellow','green']

# Iterating so to define color for every bar
color_bar = []
for i in range(len(colors)) :
    temp = []
    for j in range(int(N/100)) :
        temp.append(colors[i])
    color_bar.append(temp)

labels = []

count = np.empty((4,int(N/100))) 
bin_values = np.empty((4,int(N/100)+1)) 
max_count = 1 

for i in range(len(colors)) :
    # The x data for each sampling
    X = result[i,:].reshape(N,)
    count[i,:] , bin_values[i,:] = np.histogram(X , bins = int(N/100) , density = True)
    # max_count for Performing  normalisation
    max_count = max(max_count,count[i,:].max())

# Normalized
count = count/max_count

for i in range(len(colors)) :

    # Label for each sampling
    label_i = [f"μ = {mean[i]} , σ² = {var[i]}"]

    # Plotting bar graph for this
    plt.bar(bin_values[i,:-1],count[i,:], width=np.diff(bin_values[i,:]) ,color = color_bar[i] , alpha = 0.5)
    # plt.hist(x = , bins=int(N/10) , color=colors[i] , alpha = 0.6,density=True)

    # Writing x and y label 
    plt.xlabel("X")
    plt.ylabel("p(x)")
    labels.append(label_i)

plt.tight_layout()
plt.legend(labels , loc = "upper right")
plt.show()
plt.savefig("./images/2c.png")
