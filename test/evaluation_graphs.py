import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import numpy as np
import copy

def moreDimensionalApproximation(data_x, data_y, degree=2):
    # Interpolate cubic function.
    coefficients = np.polyfit(data_x, data_y, degree)
    foo = np.poly1d(coefficients)
    N = 100
    # Return linspace array.
    y = np.array([])
    x = np.linspace(min(data_x), max(data_x), N)
    for i in range(0, N):
        result = foo(x[i])
        y = np.append(y,result)
    # Convert to list.
    list_x = [x[i] for i in range(0,N)]
    list_y = [y[i] for i in range(0,N)]
    return list_x, list_y


###################################################################################
# Plot regularisation plot.
# Acquiring data.
fully_connected = [[0.01, 100, 61, 89.7],
                   [0.1, 100, 93.75, 92.5],
                   [0.001, 98.43, 75, 87.8]]
conv_net = [[0.001, 100, 65.62, 93.45],
            [0.01, 100, 92.18, 93.45], 
            [0.1, 100, 95.31, 96.26]]
# Sort data.
reg_fc = []; train_fc = []; test_fc = []; vali_fc = [];
for element in fully_connected:
    reg_fc.append(element[0])
    train_fc.append(element[1])
    test_fc.append(element[2])
    vali_fc.append(element[3])
reg_cn = []; train_cn = []; test_cn = []; vali_cn = [];
for element in conv_net:
    reg_cn.append(element[0])
    train_cn.append(element[1])
    test_cn.append(element[2])
    vali_cn.append(element[3])
# Plot figure.
fig = plt.figure()
ax = plt.gca()
ax.grid(True)
xs,ys = moreDimensionalApproximation(reg_fc, train_fc, degree=1)
train, = plt.plot(xs, ys, label='train')
xs,ys = moreDimensionalApproximation(reg_fc, test_fc, degree=1)
test, = plt.plot(xs, ys, label='test')
xs,ys = moreDimensionalApproximation(reg_fc, vali_fc, degree=1)
validation, = plt.plot(xs, ys, label='validation')
plt.legend(handles=[train, test, validation])
plt.xlabel("Regularisation")
plt.ylabel("Accuracy [%]")
plt.savefig("Images/Regularisation_FC.png")
fig = plt.figure()
ax = plt.gca()
ax.grid(True)
xs,ys = moreDimensionalApproximation(reg_cn, train_cn, degree=1)
train, = plt.plot(xs, ys, label='train')
xs,ys = moreDimensionalApproximation(reg_cn, test_cn, degree=1)
test, = plt.plot(xs, ys, label='test')
xs,ys = moreDimensionalApproximation(reg_cn, vali_cn, degree=1)
validation, = plt.plot(xs, ys, label='validation')
plt.legend(handles=[train, test, validation])
plt.xlabel("Regularisation")
plt.ylabel("Accuracy [%]")
plt.savefig("Images/Regularisation_CN.png")

###################################################################################
# Plot computational cost plot.
# Acquiring data.
fully_connected = [[100, 61, 89.7, 0.65],
                   [100, 93.75, 92.5, 0.65], 
                   [98.43, 75, 87.8, 0.697],
                   [100, 87.5, 94.4, 0.92],
                   [100, 81.25, 91.5, 0.58], 
                   [93.75, 65.63, 87.7, 0.89],
                   [100, 100, 93.45, 0.897], 
                   [95.31, 62.5, 86.91, 0.8915], 
                   [100, 96.88, 94.39, 0.995], 
                   [98.44, 95.31, 92.5, 0.97], 
                   [100, 92.19, 92.52, 0.89],  
                   [100, 90.6, 91.5, 1.01],
                   [100, 89.06, 86.9, 0.965], 
                   [100, 93.7, 90.65, 1.15], 
                   [100, 82.25, 90.65, 0.9155], 
                   [95.31, 65.62, 88.7, 0.9055]] 
conv_net = [[100, 81.25, 93.46, 4.88], 
            [100, 92.18, 95.32, 4.52],
            [100, 92.18, 93.45, 4.54], 
            [100, 85.93, 94.39, 4.64], 
            [100, 95.31, 96.26, 8.851], 
            [100, 84.37, 93.45, 2.355], 
            [100, 64.62, 93.45, 2.458]]
# Sort data.
comp_fc = []; train_fc = []; test_fc = []; vali_fc = [];
for element in fully_connected:
    comp_fc.append(element[3])
    train_fc.append(element[0])
    test_fc.append(element[1])
    vali_fc.append(element[2])
comp_cn = []; train_cn = []; test_cn = []; vali_cn = [];
for element in conv_net:
    comp_cn.append(element[3])
    train_cn.append(element[0])
    test_cn.append(element[1])
    vali_cn.append(element[2])
# Plot figure.
fig = plt.figure()
ax = plt.gca()
ax.grid(True)
xs,ys = moreDimensionalApproximation(comp_fc, train_fc)
train, = plt.plot(xs, ys, label='train')
xs,ys = moreDimensionalApproximation(comp_fc, test_fc)
test, = plt.plot(xs, ys, label='test')
xs,ys = moreDimensionalApproximation(comp_fc, vali_fc)
validation, = plt.plot(xs, ys, label='validation')
plt.legend(handles=[train, test, validation])
plt.xlabel("Computational Time [s] on 108 images")
plt.ylabel("Accuracy [%]")
plt.savefig("Images/CompCost_FC.png")
fig = plt.figure()
ax = plt.gca()
ax.grid(True)
xs,ys = moreDimensionalApproximation(comp_cn, train_cn)
train, = plt.plot(xs, ys, label='train')
xs,ys = moreDimensionalApproximation(comp_cn, test_cn)
test, = plt.plot(xs, ys, label='test')
xs,ys = moreDimensionalApproximation(comp_cn, vali_cn)
validation, = plt.plot(xs, ys, label='validation')
plt.legend(handles=[train, test, validation])
plt.xlabel("Computational Cost [s] on 108 images")
plt.ylabel("Accuracy [%]")
plt.savefig("Images/CompCost_CN.png")

