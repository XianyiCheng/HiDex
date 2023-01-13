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

def logistic_function(x, k, b):
    y = 1/(1+np.exp(k*x+b))
    return y

# main function
if __name__ == "__main__":
    x = np.array([[0.0],[1.0], [2.0]])
    y = np.array([[1.0],[0.5], [0.999]])
    k, b = fit_logistic_function(x, y)
    print("y = 1/(1+exp(k*x+b))")
    print("k = ", k)
    print("b = ", b)
    print("y = ", logistic_function(np.array(range(0,2,0.1)), k, b))