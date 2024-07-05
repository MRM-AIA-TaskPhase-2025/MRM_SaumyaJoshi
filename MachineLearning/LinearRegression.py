import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error

df = pd.read_csv('winequality-red.csv')

X = df.drop('quality', axis=1).values
y = df['quality'].values

X = (X - np.mean(X, axis=0)) / np.std(X, axis=0)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

def initialize_parameters(n_features):
    weights = np.zeros(n_features)
    bias = 0
    return weights, bias

def compute_cost(X, y, weights, bias):
    n_samples = len(y)
    y_predicted = np.dot(X, weights) + bias
    cost = (1 / (2 * n_samples)) * np.sum((y_predicted - y) ** 2)
    return cost

def compute_gradients(X, y, weights, bias):
    n_samples = len(y)
    y_predicted = np.dot(X, weights) + bias
    dw = (1 / n_samples) * np.dot(X.T, (y_predicted - y))
    db = (1 / n_samples) * np.sum(y_predicted - y)
    return dw, db

def gradient_descent(X, y, weights, bias, learning_rate, n_iterations, tolerance):
    previous_cost = float('inf')

    for i in range(n_iterations):
        dw, db = compute_gradients(X, y, weights, bias)
        weights -= learning_rate * dw
        bias -= learning_rate * db

        cost = compute_cost(X, y, weights, bias)

        if abs(previous_cost - cost) < tolerance:
            print(f"Convergence reached at iteration {i}")
            break

        previous_cost = cost

        if i % 100 == 0 or i == n_iterations - 1:
            print(f"Iteration {i}: Cost {cost}, Weights {weights}, Bias {bias}")

    return weights, bias

def predict(X, weights, bias):
    return np.dot(X, weights) + bias

n_features = X_train.shape[1]
weights, bias = initialize_parameters(n_features)

learning_rate = 0.01
n_iterations = 1000
tolerance = 1e-6

weights, bias = gradient_descent(X_train, y_train, weights, bias, learning_rate, n_iterations, tolerance)

y_pred = predict(X_test, weights, bias)

if np.isnan(y_pred).any():
    print("NaN values found in predictions!")
else:
    mse = mean_squared_error(y_test, y_pred)
    print(f"Mean Squared Error: {mse}")
    rmse = np.sqrt(mse)
    print(f"Root Mean Squared Error: {rmse}")

'''def predict_custom_features(custom_features, mean_train, std_train, weights, bias):
    custom_features_normalized = (custom_features - mean_train) / std_train
    return predict(custom_features_normalized, weights, bias)

custom_features = np.array([])
mean_train = np.mean(X_train, axis=0)
std_train = np.std(X_train, axis=0)
predicted_quality = predict_custom_features(custom_features, mean_train, std_train, weights, bias)
print(f"Predicted Quality: {predicted_quality}")'''
