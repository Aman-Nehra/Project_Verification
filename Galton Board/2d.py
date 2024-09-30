# Aman Nehra - 23b1051
# Haresh Gupta - 23b0931
# Sarthak Dhanajay Thakare - 23b0933
# Question Number 2 , Task D

import numpy as np
import matplotlib.pyplot as plt 

def Compute_Count_InEachBin(h,N) :
    
    # args passed : h ->  depth of the board 
    #               N ->  Number of balls for which trials is done

    # Now generating N random numbers for each ball "h" times to simulate all h movements 
    num = np.random.rand(N,h)

    # calculating movement by summing up 
    movement = np.where(num<0.5,-1,1)

    # Adding all the movement to get the final position
    final_pos = np.sum(movement,axis=1)

    return final_pos


# ***************** Now poltting it in form of bar graph ****************** #

N = 10000
h = [10,50,100]

for i in range(len(h)) :
    
    final_pos = Compute_Count_InEachBin(h[i],N)

    # To count the frequency of ball in each X s.t. X belongs to {-h,-h+1 ,.... h-1,h}
    bin_used = np.arange(-h[i]-0.5,h[i]+1.5,1) 
    count , _ = np.histogram(final_pos,bins=bin_used)
    # Normalizing the count
    count = (count/np.sum(count))

    # calculates position for each bar that is {-h,-h+1....h-1,h}
    bin_pos = np.arange(-h[i],h[i]+1,1)
    
    plt.bar(bin_pos, count, width = 1)  # width such that distribution appears uniform
    plt.xticks(np.linspace(-h[i], h[i], 5).astype(int))
    plt.xlabel("pocket")
    plt.ylabel("normalized count")
    plt.title(f"Distribution for h={h[i]}")
    plt.show()
    name = "./images/2d" + str(i+1) + ".png"
    plt.savefig(name)


