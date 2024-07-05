import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error

df = pd.read_csv('winequality-red.csv')

X = df.drop('quality', axis=1).values
y = df['quality'].values

X = (X - np.mean(X, axis=0)) / np.std(X, axis=0)

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

class LinearRegressionScratch:
    def __init__(self, learning_rate=0.01, n_iterations=1000):
        self.learning_rate = learning_rate
        self.n_iterations = n_iterations
        self.weights = None
        self.bias = None

    def fit(self, X, y):
        n_samples, n_features = X.shape
        
        self.weights = np.zeros(n_features)
        self.bias = 0
        
        for i in range(self.n_iterations):
            y_predicted = np.dot(X, self.weights) + self.bias
            
            dw = (1 / n_samples) * np.dot(X.T, (y_predicted - y))
            db = (1 / n_samples) * np.sum(y_predicted - y)
            
            self.weights -= self.learning_rate * dw
            self.bias -= self.learning_rate * db

            if i % 100 == 0 or i == self.n_iterations - 1:
                cost = (1 / (2 * n_samples)) * np.sum((y_predicted - y) ** 2)
                print(f"Iteration {i}: Cost {cost}, Weights {self.weights}, Bias {self.bias}")
                if np.isnan(cost):
                    print("NaN value encountered during training. Exiting...")
                    break

    def predict(self, X):
        return np.dot(X, self.weights) + self.bias

model = LinearRegressionScratch(learning_rate=0.01, n_iterations=1000)
model.fit(X_train, y_train)

y_pred = model.predict(X_test)

if np.isnan(y_pred).any():
    print("NaN values found in predictions!")

if not np.isnan(y_pred).any():
    mse = mean_squared_error(y_test, y_pred)
    print(f"Mean Squared Error: {mse}")
    rmse = np.sqrt(mse)
    print(f"Root Mean Squared Error: {rmse}")
else:
    print("Cannot calculate Mean Squared Error due to NaN values in predictions.")


'''def predict_custom_features(model, feature_values):
    # Normalize the input features using the mean and std of the training set
    feature_values = (feature_values - np.mean(X_train, axis=0)) / np.std(X_train, axis=0)
    quality_prediction = model.predict(np.array([feature_values]))
    return quality_prediction[0]

# Example custom feature input (should match the number of features in the dataset)
custom_features = np.array([])
predicted_quality = predict_custom_features(model, custom_features)
print(f"Predicted Quality: {predicted_quality}") 