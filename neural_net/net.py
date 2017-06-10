import tensorflow as tf

#Definition of neural nets.
def fully_connected(input_image, eps_regularization):
    #First fully connected layer.
    fc1 = tf.contrib.layers.fully_connected(input_image, num_outputs=1024, activation_fn = tf.nn.relu, 
                        weights_initializer=tf.contrib.layers.xavier_initializer(), 
                        biases_initializer=tf.contrib.layers.xavier_initializer(),
                        weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization))
    #Output layer.
    output_layer = tf.contrib.layers.fully_connected(fc1, num_outputs=2)
    return output_layer

def conv_net(input_image, eps_regularization): 

    conv1 = tf.contrib.layers.conv2d(inputs = input_image, 
                                    num_outputs = 32, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = None,
                                    weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                    )
    pool1 = tf.contrib.layers.max_pool2d(inputs = conv1, kernel_size = [2,2], stride = [1,1])
    conv2 = tf.contrib.layers.conv2d(inputs = pool1, 
                                    num_outputs = 32, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = None,
                                    weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                    )
    pool2 = tf.contrib.layers.max_pool2d(inputs = conv2, kernel_size = [2,2], stride = [1,1])
    pool2_flat = tf.contrib.layers.flatten(pool1, [-1, 7*7*64])
    fc = tf.contrib.layers.fully_connected(inputs = pool2_flat, 
                                           num_outputs = 1024, 
                                           activation_fn = tf.nn.relu,
                                           weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                           )
    output = tf.contrib.layers.fully_connected(inputs = fc, 
                                               num_outputs = 2, 
                                               activation_fn = tf.nn.relu)
    return output
    
def conv_net_parallel(input_image, eps_regularization): 

    conv1 = tf.contrib.layers.conv2d(inputs = input_image, 
                                    num_outputs = 32, 
                                    kernel_size = [7,7], 
                                    padding = "SAME", 
                                    activation_fn = None,
                                    weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                    )
    pool1 = tf.contrib.layers.max_pool2d(inputs = conv1, kernel_size = [2,2], stride = [1,1])
    conv2 = tf.contrib.layers.conv2d(inputs = input_image, 
                                    num_outputs = 32, 
                                    kernel_size = [5,5], 
                                    padding = "SAME", 
                                    activation_fn = None,
                                    weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                    )
    pool2 = tf.contrib.layers.max_pool2d(inputs = conv2, kernel_size = [2,2], stride = [1,1])
    pool1_flat = tf.contrib.layers.flatten(pool1, [-1, 7*7*64])
    pool2_flat = tf.contrib.layers.flatten(pool2, [-1, 7*7*64])
    concat = tf.concat([pool1_flat, pool2_flat], 1)
    fc = tf.contrib.layers.fully_connected(inputs = concat, 
                                           num_outputs = 1024, 
                                           activation_fn = tf.nn.relu,
                                           weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization)
                                           )
    output = tf.contrib.layers.fully_connected(inputs = fc, 
                                               num_outputs = 2, 
                                               activation_fn = tf.nn.relu)
    return output   

#Helper functions.
def getModelName(datasets):
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name
