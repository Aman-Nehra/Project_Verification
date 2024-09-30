import numpy as np
import matplotlib.pyplot as plt

# Hypothesis function (linear model)
def predict(X, theta):
    return X.dot(theta)

# Cost function (Mean Squared Error)
def compute_cost(X, y, theta):
    m = len(y)
    predictions = predict(X, theta)
    cost = (1/(2*m)) * np.sum((predictions - y)**2)
    return cost

# Gradient Descent function
def gradient_descent(X, y, theta, alpha, iterations):
    m = len(y)
    cost_history = []

    for _ in range(iterations):
        predictions = predict(X, theta)
        errors = predictions - y
        theta -= (alpha/m) * X.T.dot(errors)
        cost_history.append(compute_cost(X, y, theta))

    return theta, cost_history

# Data Generation (Simple example with one feature)
X = np.random.rand(100, 1) * 10  # 100 random points between 0 and 10
y = 5 + 2 * X + np.random.randn(100, 1)  # Linear relation with some noise

# Add intercept term (X0 = 1) to the data
X_b = np.c_[np.ones((100, 1)), X]

# Initial theta values
theta = np.random.randn(2, 1)

# Parameters
alpha = 0.01
iterations = 1000

# Perform gradient descent
theta_opt, cost_history = gradient_descent(X_b, y, theta, alpha, iterations)

# Plot the data and the fitted line
plt.scatter(X, y, color='blue')
plt.plot(X, predict(X_b, theta_opt), color='red', label='Fitted line')
plt.xlabel('X')
plt.ylabel('y')
plt.legend()
plt.show()

# Cost history plot
plt.plot(cost_history)
plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('Cost Function History')
plt.show()
