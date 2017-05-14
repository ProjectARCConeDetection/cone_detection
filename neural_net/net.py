import tensorflow as tf

#Definition of neural nets.
def fully_connected(input_image, eps_regularization):
    #First fully connected layer.
    fc1 = tf.contrib.layers.fully_connected(input_image, num_outputs=512, activation_fn = tf.nn.relu, 
                        weights_initializer=tf.contrib.layers.xavier_initializer(), 
                        biases_initializer=tf.contrib.layers.xavier_initializer(),
                        weights_regularizer=tf.contrib.layers.l1_regularizer(eps_regularization))
    #Output layer.
    output_layer = tf.contrib.layers.fully_connected(fc1, num_outputs=2)
    return output_layer

#Helper functions.
def getModelName(datasets):
    model_name = ""
    for element in datasets:
        model_name += "__" + element
    return model_name
