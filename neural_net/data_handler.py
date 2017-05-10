from __future__ import division, print_function

import csv
import numpy as np
import os
from PIL import Image
from random import randint, shuffle
import tensorflow as tf

class DataHandler:
    def __init__(self, image_height, image_width, batch_size, test_size, 
                path_candidates, path_model, datasets, equalising):
        #Define image and batch parameter.
        self.image_height = image_height
        self.image_width = image_width
        self.batch_size = batch_size
        #Set test size.
        self.test_size = test_size
        #Set paths and define dataset.
        self.path_candidates = path_candidates
        self.path_model = path_model
        self.datasets = datasets
        #Get training data.
        self.train_X, self.train_Y = self.getTrainingData(equalising)
        #Init batch counter.
        self.batch_counter = 0

    def equaliseSet(self,set_X, set_Y):
        #Count positives and negatives.
        positives = 0; negatives = 0;
        for element in set_Y:
            if(element[0]): positives += 1
            else: negatives += 1 
        #Return iff already equalised.
        if(positives == negatives): return set_X, set_Y
        #Find bias.
        negative_bias = (negatives > (len(set_X)/2))
        if(negative_bias): counter = positives
        if(not negative_bias): counter = negatives
        #Equalise.
        equalised_x = []; equalised_y = [];
        for index in range(0, len(set_X)-1):
            if(counter <= 0 and negative_bias and set_Y[index][1]): continue
            if(counter <= 0 and not negative_bias and set_Y[index][0]): continue
            equalised_x.append(set_X[index])
            equalised_y.append(set_Y[index])
            if(negative_bias and set_Y[index][1]): counter -= 1
            if(not negative_bias and set_Y[index][0]): counter -= 1
        return equalised_x, equalised_y

    def getBatch(self):
        length = len(self.train_X)-1
        batch_x = np.zeros((self.batch_size,self.image_height,self.image_width,3))
        batch_y = np.zeros((self.batch_size,2))
        #Get training batch.
        for index in range(0,self.batch_size-1):
            batch_x[index][:][:][:] = self.train_X[self.batch_counter + index]
            batch_y[index][:] = self.train_Y[self.batch_counter + index]
        #Update batch_counter.
        self.batch_counter += self.batch_size
        if(self.batch_counter + self.batch_size >= length): self.batch_counter = 0
        return batch_x,batch_y

    def getImageFromPath(self, path):
        img = Image.open(path)
        arr = np.array(img.getdata(),np.uint8)
        arr = arr.reshape(self.image_height, self.image_width, 3)
        return arr

    def getLabels(self, path, dataset):
        labeled_list = []
        reader = csv.reader(open(path))
        for row in reader:
            image = int(row[0])
            label = int(row[1])
            location = dataset
            labeled_list.append([image, label, dataset])
        return labeled_list

    def getRandomTestData(self):
        test_x = []; test_y = [];
        #Get random data.
        for iteration in range(0, self.test_size):
            random_int = randint(0,len(self.train_X)-1)
            test_x.append(self.train_X[random_int])
            test_y.append(self.train_Y[random_int])
        #Equalise dataset.
        test_x, test_y = self.equaliseSet(test_x, test_y)
        return test_x, test_y

    def getTrainingData(self, equalising):
        train_X = []; train_Y = []; labeled_list = []
        # Getting labeled list of all datasets.
        for dataset in self.datasets:
            path = os.path.join(self.path_candidates, dataset)
            labeled_list.append(self.getLabels(path + "/labeling.csv", dataset))
        # Shuffle list.
        labeled_list = labeled_list[0]
        shuffle(labeled_list)
        # Getting training data.
        positive = 0; negative = 0;
        for data in labeled_list:
            path = self.path_candidates + data[2] + "/" + str(data[0]) + ".jpg"
            img = self.getImageFromPath(path)
            label = data[1]
            train_X.append(img)
            if label: 
                train_Y.append(np.array([1,0]))
                positive += 1
            else: 
                train_Y.append(np.array([0,1]))
                negative += 1
        # Not equalising.
        if(not equalising):
            print("Length of data sets: %f with %f positive and %f negatives !" % (len(labeled_list),positive,negative))
            return train_X, train_Y
        # Equalise dataset.
        equalised_x, equalised_y = self.equaliseSet(train_X, train_Y)
        print("Length of data sets: %f, equalised !" % len(equalised_x))
        return equalised_x, equalised_y