import tensorflow as tf

#Definition of neural nets.
def conv_net(x, image_height, image_width, n_classes, train=False): 
    # Reshape input picture.
    x = tf.reshape(x, shape=[-1,image_height,image_width,3])
    # First Convolutional Layer.
    conv1 = tf.contrib.layers.conv2d(inputs = x, 
                                    num_outputs = 64, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = tf.nn.relu)
    pool1 = tf.contrib.layers.max_pool2d(inputs = conv1, kernel_size = [2,2], stride = [2,2])
    # Second Convolutional Layer.
    conv2 = tf.contrib.layers.conv2d(inputs = pool1, 
                                    num_outputs = 64, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = tf.nn.relu)
    pool2 = tf.contrib.layers.max_pool2d(inputs = conv2, kernel_size = [2,2], stride = [2,2])
    # Fully connected layer.
    pool2_flat = tf.contrib.layers.flatten(pool2, [-1, 7*7*64])
    fc = tf.contrib.layers.fully_connected(inputs = pool2_flat, 
                                           num_outputs = 1024, 
                                           activation_fn = tf.nn.relu)
    # Output layer.
    output = tf.contrib.layers.fully_connected(inputs = fc, 
                                               num_outputs = n_classes, 
                                               activation_fn = tf.nn.relu)
    return tf.nn.softmax(output)

#Helper functions.
def getModelName(datasets):
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name
