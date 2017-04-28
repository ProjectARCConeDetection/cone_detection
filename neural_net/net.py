import tensorflow as tf

#Definition of neural nets.
def conv_net(x, image_height, image_width, n_classes):
    # Store layers weight & bias.
    weights = {
        'wc1': tf.Variable(tf.random_normal([5, 5, 3, 16])),
        'wc2': tf.Variable(tf.random_normal([5, 5, 16, 32])),
        'fc1': tf.Variable(tf.random_normal([5*39*32, 1024])),
        'out': tf.Variable(tf.random_normal([1024, n_classes]))
    }

    biases = {
        'bc1': tf.Variable(tf.random_normal([16])),
        'bc2': tf.Variable(tf.random_normal([32])),
        'fc1': tf.Variable(tf.random_normal([1024])),
        'out': tf.Variable(tf.random_normal([n_classes]))
    }   
    # Reshape input picture.
    x = tf.reshape(x, shape=[-1,image_height,image_width,3])
    # Convolutional Layer.
    conv1  = tf.nn.conv2d(x, weights['wc1'], strides=[1,1,1,1], padding='SAME')
    conv1 = tf.nn.bias_add(conv1, biases['bc1'])
    conv1 = tf.nn.relu(conv1)
    conv1 = tf.nn.max_pool(conv1, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
    conv2  = tf.nn.conv2d(conv1, weights['wc2'], strides=[1,1,1,1], padding='SAME')
    conv2 = tf.nn.bias_add(conv2, biases['bc2'])
    conv2 = tf.nn.relu(conv2)
    conv2 = tf.nn.max_pool(conv2, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')
    # Fully connected layer.
    fc1 = tf.reshape(conv2, [-1, weights['fc1'].get_shape().as_list()[0]])
    fc1 = tf.add(tf.matmul(fc1, weights['fc1']), biases['fc1'])
    fc1 = tf.nn.relu(fc1)
    # Output, class prediction
    out = tf.add(tf.matmul(fc1, weights['out']), biases['out'])
    return tf.nn.softmax(out)


#Helper functions.
def getModelName(datasets):
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name