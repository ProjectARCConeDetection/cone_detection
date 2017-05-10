import tensorflow as tf

#Definition of neural nets.
def conv_net(x, image_height, image_width, n_classes, dropout): 
    # Reshape input picture.
    # x = tf.reshape(x, shape=[-1,image_height,image_width,3])
    # First Convolutional Layer.
    conv1 = tf.contrib.layers.conv2d(inputs = x, 
                                    num_outputs = 32, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = None)
    pool1 = tf.contrib.layers.max_pool2d(inputs = conv1, kernel_size = [2,2], stride = [1,1])
    # Second Convolutional Layer.
    conv2 = tf.contrib.layers.conv2d(inputs = pool1, 
                                    num_outputs = 64, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = None)
    pool2 = tf.contrib.layers.max_pool2d(inputs = conv2, kernel_size = [2,2], stride = [1,1])
    # Fully connected layer with dropout.
    pool2_flat = tf.contrib.layers.flatten(pool2, [-1, 7*7*64])
    fc = tf.contrib.layers.fully_connected(inputs = pool2_flat, 
                                           num_outputs = 1024, 
                                           activation_fn = tf.nn.relu)
    fc = tf.nn.dropout(fc, dropout)
    # Output layer.
    output = tf.contrib.layers.fully_connected(inputs = fc, 
                                               num_outputs = n_classes, 
                                               activation_fn = tf.nn.relu)
    return tf.nn.softmax(output)


def easy_dense(x, image_height, image_width, n_classes): 
    # Reshape input picture.
    x = tf.reshape(x, shape=[-1,image_height*image_width*3])
    # Output layer.
    output = tf.contrib.layers.fully_connected(inputs = x, 
                                               num_outputs = n_classes, 
                                               activation_fn = None)
    return tf.nn.softmax(output)

# Create some wrappers for simplicity
def conv2d(x, W, b, strides=1):
    # Conv2D wrapper, with bias and relu activation
    x = tf.nn.conv2d(x, W, strides=[1, strides, strides, 1], padding='SAME')
    x = tf.nn.bias_add(x, b)
    return tf.nn.relu(x)


def maxpool2d(x, k=2):
    # MaxPool2D wrapper
    return tf.nn.max_pool(x, ksize=[1, k, k, 1], strides=[1, k, k, 1], padding='SAME')

def conv_without_contrib(x, image_height, image_width, n_classes, dropout): 
    # Store layers weight & bias
    weights = {
        # 5x5 conv, 3 inputs, 32 outputs
        'wc1': tf.Variable(tf.random_normal([5, 5, 3, 32])),
        # 15x15 conv, 32 inputs, 64 outputs
        'wc2': tf.Variable(tf.random_normal([15, 15, 32, 64])),
        # fully connected, 64*195 inputs, 1024 outputs
        'wd1': tf.Variable(tf.random_normal([64*3000, 1024])),
        # 1024 inputs, 10 outputs (class prediction)
        'out': tf.Variable(tf.random_normal([1024, n_classes]))
    }

    biases = {
        'bc1': tf.Variable(tf.random_normal([32])),
        'bc2': tf.Variable(tf.random_normal([64])),
        'bd1': tf.Variable(tf.random_normal([1024])),
        'out': tf.Variable(tf.random_normal([n_classes]))
    }
    # Reshape input picture.
    x = tf.reshape(x, shape=[-1, image_height, image_width, 3])
    # Convolution Layer.
    conv1 = conv2d(x, weights['wc1'], biases['bc1'])
    # Max Pooling (down-sampling).
    conv1 = maxpool2d(conv1, k=1)
    # Convolution Layer.
    conv2 = conv2d(conv1, weights['wc2'], biases['bc2'])
    # Max Pooling (down-sampling).
    conv2 = maxpool2d(conv2, k=1)
    # Fully connected layer
    # Reshape conv2 output to fit fully connected layer input.
    fc1 = tf.reshape(conv2, [-1, weights['wd1'].get_shape().as_list()[0]])
    fc1 = tf.add(tf.matmul(fc1, weights['wd1']), biases['bd1'])
    fc1 = tf.nn.relu(fc1)
    # Apply Dropout.
    fc1 = tf.nn.dropout(fc1, dropout)
    # Output, class prediction.
    out = tf.add(tf.matmul(fc1, weights['out']), biases['out'])
    return tf.nn.softmax(out)

#Helper functions.
def getModelName(datasets):
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name
