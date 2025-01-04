import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from statsmodels.tsa.arima.model import ARIMA
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LinearRegression

# Things to process in data 
# Reports null for 20 min every week, So that 20 their would be a jump observed of 40 minutes in the data
# Data only from 5am to 11:59pm 
# More demand on Holdiday 
# All those number plate differe by one character are same 

# Function to calculate MASE (Mean Absolute Scaled Error)
def mase(y_true, y_pred, y_train):
    naive_forecast = np.roll(y_train, shift=1)[1:]
    mae_naive = np.mean(np.abs(y_train[1:] - naive_forecast))
    return np.mean(np.abs(y_true - y_pred)) / mae_naive

# Function to calculate MAPE (Mean Absolute Percentage Error)
def mape(y_true, y_pred):
    return np.mean(np.abs((y_true - y_pred) / y_true)) * 100

# Function to plot the time series
def plot_series(train, test, forecast, title):
    plt.figure(figsize=(10, 6))
    plt.plot(train.index, train, label="Train Data")
    plt.plot(test.index, test, label="Test Data", color='orange')
    plt.plot(test.index, forecast, label="Forecast", color='green')
    plt.title(title)
    plt.legend()
    plt.show()

# Load the parking lot data
df = pd.read_csv('./parkingLot (1).csv')
df = df.ffill()

# Convert timestamp to datetime and filter out times when the mall is closed (12 AM - 5 AM)
df['timestamp'] = pd.to_datetime(df['timestamp'])
df = df[(df['timestamp'].dt.hour >= 5) & (df['timestamp'].dt.hour < 24)]

# Separate camera data for entry (camera ID 001) and exit (camera ID 002)
df_entry = df[df['camera_id'] == 1].copy()
df_exit = df[df['camera_id'] == 2].copy()

# ----------------------------- PART A -----------------------------
# Forecast total number of vehicles entering per day
df_entry['date'] = df_entry['timestamp'].dt.date
vehicles_per_day = df_entry.groupby('date').size().asfreq('D')

# Fill missing values with 0
vehicles_per_day = vehicles_per_day.fillna(0)

# Split data into training and testing sets
train_size = int(len(vehicles_per_day) * 0.8)
train_data, test_data = vehicles_per_day[:train_size], vehicles_per_day[train_size:]

# Ensure the index has a frequency for ARIMA
train_data.index = pd.date_range(start=train_data.index[0], periods=len(train_data), freq='D')
test_data.index = pd.date_range(start=train_data.index[-1] + pd.Timedelta(days=1), periods=len(test_data), freq='D')

# ARIMA model for forecasting vehicles entering
arima_model = ARIMA(train_data, order=(11, 0, 7))
arima_result = arima_model.fit()
arima_forecast = arima_result.forecast(steps=len(test_data))

# Polynomial Regression for forecasting vehicles entering
degree = 4  # You can change the degree of the polynomial
X_train = np.arange(len(train_data)).reshape(-1, 1)
X_test = np.arange(len(train_data), len(train_data) + len(test_data)).reshape(-1, 1)
poly_model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
poly_model.fit(X_train, train_data)
poly_forecast = poly_model.predict(X_test)

# Plot ARIMA forecast
plot_series(train_data, test_data, arima_forecast, "ARIMA Forecast of Vehicles Entering Per Day")

# Plot Polynomial Regression forecast
plot_series(train_data, test_data, poly_forecast, f"Polynomial Regression (Degree {degree}) Forecast of Vehicles Entering Per Day")

# Evaluate models for Part A
print("Part A: Forecast Total Number of Vehicles Entering Per Day")
print("ARIMA MASE:", mase(test_data, arima_forecast, train_data))
print("ARIMA MAPE:", mape(test_data, arima_forecast))
print(f"Polynomial Regression (Degree {degree}) MASE:", mase(test_data, poly_forecast, train_data))
print(f"Polynomial Regression (Degree {degree}) MAPE:", mape(test_data, poly_forecast))


# ----------------------------- PART B -----------------------------
# Forecast average time spent in the mall per day

# Calculate time spent by each vehicle by matching entry and exit times
df_entry_exit = pd.merge(df_entry, df_exit, on='vehicle_no', suffixes=('_entry', '_exit'))
df_entry_exit['time_spent'] = (df_entry_exit['timestamp_exit'] - df_entry_exit['timestamp_entry']).dt.total_seconds() / 60.0

# Group by date to get average time spent per day
avg_time_per_day = df_entry_exit.groupby(df_entry_exit['timestamp_entry'].dt.date)['time_spent'].mean().asfreq('D').fillna(0)

# Split data into training and testing sets for time spent
train_size_time = int(len(avg_time_per_day) * 0.8)
train_time_data, test_time_data = avg_time_per_day[:train_size_time], avg_time_per_day[train_size_time:]

# Ensure the index has a frequency for ARIMA
train_time_data.index = pd.date_range(start=train_time_data.index[0], periods=len(train_time_data), freq='D')
test_time_data.index = pd.date_range(start=train_time_data.index[-1] + pd.Timedelta(days=1), periods=len(test_time_data), freq='D')

# ARIMA model for forecasting average time spent
arima_time_model = ARIMA(train_time_data, order=(11, 2, 12))
arima_time_result = arima_time_model.fit()
arima_time_forecast = arima_time_result.forecast(steps=len(test_time_data))

# Polynomial Regression for forecasting average time spent
X_train_time = np.arange(len(train_time_data)).reshape(-1, 1)
X_test_time = np.arange(len(train_time_data), len(train_time_data) + len(test_time_data)).reshape(-1, 1)
poly_time_model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
poly_time_model.fit(X_train_time, train_time_data)
poly_time_forecast = poly_time_model.predict(X_test_time)

# Plot ARIMA forecast for time spent
plot_series(train_time_data, test_time_data, arima_time_forecast, "ARIMA Forecast of Average Time Spent Per Day")

# Plot Polynomial Regression forecast for time spent
plot_series(train_time_data, test_time_data, poly_time_forecast, f"Polynomial Regression (Degree {degree}) Forecast of Average Time Spent Per Day")

# Evaluate models for Part B
print("\nPart B: Forecast Average Time Spent Per Day")
print("ARIMA MASE:", mase(test_time_data, arima_time_forecast, train_time_data))
print("ARIMA MAPE:", mape(test_time_data, arima_time_forecast))
print(f"Polynomial Regression (Degree {degree}) MASE:", mase(test_time_data, poly_time_forecast, train_time_data))
print(f"Polynomial Regression (Degree {degree}) MAPE:", mape(test_time_data, poly_time_forecast))
