#!/usr/bin/env python
# coding: utf-8

# %%


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import statsmodels.api as sm
from sktime.forecasting.arima import ARIMA
from statsmodels.tsa.stattools import adfuller
from sktime.forecasting.model_selection import temporal_train_test_split
from statsmodels.tsa.stattools import acf, pacf
from sktime.transformations.series.difference import Differencer 
from statsmodels.tsa.seasonal import seasonal_decompose

data = pd.read_csv('AirtrafficA4.csv')



# Map month names to numbers and combine Year and Month into a datetime index
month_mapping = {
    'JAN': 1, 'FEB': 2, 'MAR': 3, 'APR': 4,
    'MAY': 5, 'JUNE': 6, 'JULY': 7, 'AUG': 8,
    'SEP': 9, 'OCT': 10, 'NOV': 11, 'DEC': 12
}
data['MONTH'] = data['MONTH'].str.strip().map(month_mapping)

# Create a proper datetime column using the first day of each month
data['DATE'] = pd.to_datetime(data[['YEAR', 'MONTH']].assign(DAY=1))

# Convert dates to string in 'MM-YYYY' format for x-axis labels
data['DATE_STR'] = data['DATE'].dt.strftime('%m-%Y')
data.dropna(subset=['PASSENGERS CARRIED'], inplace=True)
# Set the DATE as index for the DataFrame
data.set_index('DATE', inplace=True)
# Keep only the 'PASSENGERS CARRIED' column
data = data[['PASSENGERS CARRIED']]
# Ensure the DataFrame is sorted by the DATE index
data.sort_index(inplace=True)


# In[109]:


# Convert index back to string dates for x-axis labels
def process_for_plot(data):
    dates_str = data.index.strftime('%m-%Y').tolist()
    data['PASSENGERS CARRIED'] = pd.to_numeric(data['PASSENGERS CARRIED'].str.replace(',', ''), errors='coerce')
    passengers_carried = data['PASSENGERS CARRIED'].to_numpy()
    return dates_str, passengers_carried
# dates_str , passengers_carried = process_for_plot(data)


# In[115]:


def check_stationary(data):
    result = adfuller(data)
    print('ADF Statistic: %f' % result[0])
    print('p-value: %f' % result[1])
    print('Critical Values:')
    for key, value in result[4].items():
        print('\t%s: %.3f' % (key, value))
    if (result[1] <= 0.05) & (result[4]['5%'] > result[0]):
        print("\u001b[32mStationary\u001b[0m")
    else:
        print("\x1b[31mNon-stationary\x1b[0m")
check_stationary(data)


# In[111]:


def plot_data(data):
    # Plot the data

    dates_str , passengers_carried = process_for_plot(data)

    plt.figure(figsize=(12, 6))
    plt.plot(dates_str, passengers_carried, label='Passengers Carried', color='blue',marker='o',ms=4,mfc='red')


    # Rotate x-axis labels for clarity
    plt.xticks(rotation=45)

    # Manually set the number of x-axis ticks to avoid crowding
    step = max(1, len(dates_str) // 10)
    plt.xticks(np.arange(0, len(dates_str), step), dates_str[::step])

    # Titles and labels
    plt.title('Passengers Carried Over Time')
    plt.xlabel('Date')
    plt.ylabel('Number of Passengers')
    plt.legend()

    # Adjust layout to prevent label cut-off
    plt.tight_layout()

    # Save and display the plot
    plt.savefig('airtraffic.png')
    plt.show()

plot_data(data)


# In[116]:


def plotting_diffrenced_data(data,d):
    # # Example usage
    # passengers_carried = np.diff(passengers_carried)
    # # # Adjust the dates to match the length of the differenced data
    # dates_str = dates_str[1:]  # Adjust the dates to account for d

    # p_value = sm.tsa.stattools.adfuller(passengers_carried)[1]
    # print(f"The p-value for the ADF test is {p_value:.4f}")

    # d = 1 # Differencing order 

    # # # Plot the differenced data
    # plt.figure(figsize=(12, 6))
    # plt.plot(dates_str, passengers_carried, label=f'Passengers Carried (d={d})', color='blue', marker='o', ms=4, mfc='red')

    # # Rotate x-axis labels for clarity
    # plt.xticks(rotation=45)

    # # Manually set the number of x-axis ticks to avoid crowding
    # step = max(1, len(dates_str) // 10)
    # plt.xticks(np.arange(0, len(dates_str), step), dates_str[::step])

    # # Titles and labels
    # plt.title(f'Passengers Carried Over Time (Differenced, d={d})')
    # plt.xlabel('Date')
    # plt.ylabel('Number of Passengers')
    # plt.legend()

    # # Adjust layout to prevent label cut-off
    # plt.tight_layout()
    # # Save and display the plot
    # plt.show()

    Difier = Differencer(lags=d,na_handling="drop_na")
    diifference_data = Difier.fit_transform(data)
    
    plot_data(diifference_data)
    check_stationary(diifference_data)

    return diifference_data

plotting_diffrenced_data(data,1)
# plot_data(dates_str, passengers_carried)


# In[34]:


# Plotting ACF and PACF 
def acf_pacf(data,lags=None) :
    return acf(data,nlags=lags) , pacf(data,nlags=lags)


# In[35]:


def plot_acf_pacf(data,lags=None) : 
    fig , ax = plt.subplots(1,2,figsize=(10,5))
    acf_nlags , pacf_nlags = acf_pacf(data,lags)

    x_axis_acf = np.arange(0,acf_nlags.shape[0],1)
    x_axis_pacf = np.arange(0,pacf_nlags.shape[0],1)

    # Add markers at the top of ACF bars
    for i, value in enumerate(acf_nlags):
        ax[0].plot(i, value, 'bo')  # 'ro' means red circle markers

    ax[0].bar(x_axis_acf,acf_nlags,width=0.2)
    ax[0].grid(True)
    ax[0].set_xlabel("lags")
    ax[0].set_ylabel("ACF")
    ax[0].set_title('Autocorrelation Function (ACF)')

    # Add markers at the top of ACF bars
    for i, value in enumerate(pacf_nlags):
        ax[1].plot(i, value, 'bo')  # 'ro' means red circle markers

    ax[1].bar(x_axis_pacf,pacf_nlags,width=0.2)
    ax[1].grid(True)
    ax[1].set_xlabel("lags")
    ax[1].set_ylabel("PACF")
    ax[1].set_title('Partial Autocorrelation Function (PACF)')

    plt.show()

    return acf_nlags , pacf_nlags


# In[36]:


lags = 40
acf_nlags , pacf_nlags = plot_acf_pacf(passengers_carried,lags)
print(f"ACF for {lags} lags = {acf_nlags}")
print(f"PACF for {lags} lags = {pacf_nlags}")


# In[39]:


"""
This Plot demonstrates that there is no significant sesonality in the data 
"""


# Ensure the 'DATE' column is a DatetimeIndex
data.index = pd.to_datetime(data.index)

data['PASSENGERS CARRIED'] = data['PASSENGERS CARRIED'].ffill()
# Set the frequency to 'MS' (Month Start) since it's monthly data
data = data.asfreq('MS')

# Decompose the time series into trend, seasonal, and residual components
result = seasonal_decompose(data['PASSENGERS CARRIED'], model='multiplicative')

# Plot the results
result.plot()
plt.suptitle('Seasonal Decomposition of Air Passengers Time Series')
plt.tight_layout()
plt.show()

data_without_seasonal = data['PASSENGERS CARRIED'] / result.seasonal
# Plot the original data without the seasonal component
plt.plot(data, label='Original Time Series', color='blue')
plt.plot(data_without_seasonal, label='Original Data without Seasonal Component', color='green')
plt.title('Air Passengers Time Series with and without Seasonal Component')
plt.xlabel('Year')
plt.ylabel('Number of Passengers')
plt.legend()
plt.show()


# In[40]:


def MAE(y_true, y_pred):
    return np.mean(np.abs(y_true - y_pred))

def MAPE(y_true, y_pred):
    return np.mean(np.abs((y_true - y_pred) / y_true)) * 100


# In[43]:


def fit_arima_model(data, p, d, q):
    # Split the data into train and test sets (80% train, 20% test)
    train, test = temporal_train_test_split(data, test_size=int(len(data) * 0.2))
    # Define forecast horizon for the test set length
    fh = np.arange(1, len(test) + 1)
    
    print(test , train)
    # Fit the ARIMA model
    model = ARIMA(order=(p, d, q))
    model.fit(train)
    print(model.summary())
    # Forecast for the test set
    y_pred = model.predict(fh=fh)
    
    # Calculate mae and MAPE 
    mae = MAE(test, y_pred)
    mape = MAPE(test, y_pred)

    # Plot the forecast
    # Plot the training data, test data, and forecast
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(train)), train, label='Training Data', color='blue')
    plt.plot(range(len(train), len(train) + len(test)), test, label='Test Data', color='green')
    plt.plot(range(len(train), len(train) + len(y_pred)), y_pred, label='Forecast', color='red')
    plt.legend()
    plt.show()
    
    return mae,mape

# Fit the ARIMA model and visualize the forecast
mae ,mape = fit_arima_model(passengers_carried, 5, 1, 0)
print(f"MAE: {mae:.2f}")
print(f"MAPE: {mape:.2f}%")


# In[ ]:




