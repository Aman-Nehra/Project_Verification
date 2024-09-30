import numpy as np
import matplotlib.pyplot as plt

# Hypothesis function for time series (linear model)
def predict(X, theta):
    return np.dot(X, theta)

# Cost function
def compute_cost(X, y, theta):
    m = len(y)
    predictions = predict(X, theta)
    cost = (1/(2*m)) * np.sum((predictions - y)**2)
    return cost

# Gradient Descent
def gradient_descent(X, y, theta, alpha, iterations):
    m = len(y)
    cost_history = []

    for i in range(iterations):
        predictions = predict(X, theta)
        theta -= (alpha/m) * np.dot(X.T, (predictions - y))
        cost_history.append(compute_cost(X, y, theta))

    return theta, cost_history

# Simulated time series data
time = np.arange(1, 101).reshape(-1, 1)
y = 5 + 0.5 * time + np.random.randn(100, 1)  # True trend: y = 5 + 0.5*t + noise

# Add intercept term
X_b = np.c_[np.ones((100, 1)), time]

# Initial theta values
theta = np.random.randn(2, 1)

# Hyperparameters
alpha = 0.01
iterations = 1000

# Perform gradient descent
theta_opt, cost_history = gradient_descent(X_b, y, theta, alpha, iterations)

# Plot time series data and regression line
plt.plot(time, y, 'b.', label="Actual Data")
plt.plot(time, predict(X_b, theta_opt), 'r-', label="Fitted Line")
plt.xlabel('Time')
plt.ylabel('Value')
plt.legend()
plt.title('Time Series Regression')
plt.show()

# Display optimized theta values
print("Optimized Parameters for Time Series Regression:", theta_opt)
