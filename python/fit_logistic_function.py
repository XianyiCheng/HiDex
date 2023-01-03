import numpy as np
from sklearn.linear_model import LinearRegression

# main function

def fit_logistic_function(x, y):
    # fit a logistic function to the data
    # x: input data
    # y: output data
    # y = 1/(1+exp(kx+b))
    # return: k, b
    y_linear = np.log(1/y - 1)
    # print(y_linear)
    clf = LinearRegression()
    clf.fit(x, y_linear)
    k = clf.coef_
    b = clf.intercept_
    return k, b

# main function
if __name__ == "__main__":
    x_path_size = np.array([[10],[5], [1]])
    y_path_size = np.array([[0.2],[0.5], [0.99]])
    k, b = fit_logistic_function(x_path_size, y_path_size)
    print("path size")
    print("k = ", k)
    print("b = ", b)

    x_dist = np.array([[1],[2], [3]])
    y_dist = np.array([[0.99],[0.5], [0.1]])
    k, b = fit_logistic_function(x_dist, y_dist)
    print("distance")
    print("k = ", k)
    print("b = ", b)

    x_finger = np.array([[0.99], [0.8], [0.5],[0.2], [0.1],[0.0]])
    y_finger = np.array([[0.001], [0.05], [0.2],[0.7],[0.9],[0.999]])
    k, b = fit_logistic_function(x_finger, y_finger)
    print("finger")
    print("k = ", k)
    print("b = ", b)
    print(1/(1+np.exp(k*x_finger+b)))