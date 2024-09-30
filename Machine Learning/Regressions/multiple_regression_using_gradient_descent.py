import numpy as np

# Prediction function (h_theta = X * theta)
def predict(X, theta):
    return X.dot(theta)

# Cost function for multiple regression (Mean Squared Error)
def compute_cost(X, y, theta):
    m = len(y)
    predictions = predict(X, theta)
    cost = (1/(2*m)) * np.sum((predictions - y)**2)
    return cost

# Gradient Descent for multiple regression
def gradient_descent(X, y, theta, alpha, iterations):
    m = len(y)
    cost_history = []

    for _ in range(iterations):
        predictions = predict(X, theta)
        theta -= (alpha/m) * X.T.dot(predictions - y)
        cost_history.append(compute_cost(X, y, theta))

    return theta, cost_history

# Generating multiple feature data (3 features)
np.random.seed(42)
X = np.random.rand(100, 3)  # 100 samples, 3 features
y = 3 + 1.5 * X[:, 0] + 2 * X[:, 1] - 2.5 * X[:, 2] + np.random.randn(100)  # True relationship

# Add intercept term to X
X_b = np.c_[np.ones((100, 1)), X]

# Initial theta (for 4 parameters including intercept)
theta = np.random.randn(4)

# Parameters
alpha = 0.01
iterations = 1000

# Running Gradient Descent
theta_opt, cost_history = gradient_descent(X_b, y, theta, alpha, iterations)

# Display optimized theta values
print("Optimized Theta Values:", theta_opt)

# Optional: Check final cost
print("Final Cost:", cost_history[-1])
