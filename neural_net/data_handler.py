from __future__ import division, print_function

from net import *

import csv
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
from random import randint, shuffle
import tensorflow as tf
from skimage import color

class DataHandler:
    def __init__(self, image_height, image_width, batch_size, test_size, 
                path_candidates, path_model, datasets, datasets_test):
        #Define image and batch parameter.
        self.image_height = image_height
        self.image_width = image_width
        self.batch_size = batch_size
        #Set test size.
        self.test_size = test_size
        #Set paths and define dataset.
        self.path_candidates = path_candidates
        self.path_model = path_model
        #Get training data.
        self.train_X, self.train_Y = self.TrainingData(datasets)
        #Get test data.
        self.test_X, self.test_Y = self.TestData(datasets_test)
        #Init batch counter.
        self.batch_counter = 0
        #Init accuracy and loss lists.
        self.index = []
        self.acc_batch = []
        self.acc_train = []
        self.acc_test = []
        self.loss_batch = []
        self.figure_name = self.path_model + getModelName(datasets) + '.png'
        self.description = self.Description(datasets, datasets_test)

    def getBatch(self):
        length = len(self.train_X)-1
        batch_x = np.zeros((self.batch_size,self.image_height,self.image_width,3))
        batch_y = np.zeros((self.batch_size,2))
        #Get training batch.
        for index in range(0,self.batch_size-1):
            batch_x[index][:][:][:] = color.rgb2lab(self.train_X[self.batch_counter + index]) / 255.0
            batch_y[index][:] = self.train_Y[self.batch_counter + index]
        #Update batch_counter.
        self.batch_counter += self.batch_size
        if(self.batch_counter + self.batch_size >= length): self.batch_counter = 0
        return batch_x,batch_y

    def getTestBatch(self):
        test_x = np.zeros((self.test_size,self.image_height,self.image_width,3))
        test_y = np.zeros((self.test_size,2))
        #Get test batch.
        for index in range(0,self.test_size):
            random_int = randint(0,len(self.test_X)-1)
            test_x[index][:][:][:] = color.rgb2lab(self.test_X[random_int]) / 255.0
            test_y[index][:] = self.test_Y[random_int]
        return test_x, test_y

    def getTrainingTestBatch(self):
        test_x = np.zeros((self.test_size,self.image_height,self.image_width,3))
        test_y = np.zeros((self.test_size,2))
        #Get test batch.
        for index in range(0,self.test_size):
            random_int = randint(0,len(self.train_X)-1)
            test_x[index][:][:][:] = color.rgb2lab(self.train_X[random_int]) / 255.0
            test_y[index][:] = self.train_Y[random_int]
        return test_x, test_y

    def Description(self, datasets, datasets_test):
        description = "Training Sets: "
        for element in datasets:
            description += str(element) + "_"
        description += " and Testing Sets: "
        for element in datasets_test:
            description += str(element) + "_"
        return description

    def ImageFromPath(self, path):
        img = Image.open(path)
        arr = np.array(img.getdata(),np.uint8)
        arr = arr.reshape(self.image_height, self.image_width, 3)
        return arr

    def Labels(self, path, dataset):
        labeled_list = []
        reader = csv.reader(open(path))
        for row in reader:
            image = int(row[0])
            label = int(row[1])
            location = dataset
            labeled_list.append([image, label, dataset])
        return labeled_list

    def plotAccuracyPlot(self):
        #Create figure.
        fig = plt.figure()
        fig.suptitle('Accuarcy and Loss Graph', fontsize=14, fontweight='bold')
        ax = fig.add_subplot(111)
        fig.subplots_adjust(top=0.85)
        ax.set_title(self.description)
        #Add plots.
        plt.plot(self.index, self.acc_batch, 'r', label="acc_batch")
        plt.plot(self.index, self.loss_batch, 'r--', label="loss_batch")
        plt.plot(self.index, self.acc_train, 'k', label="acc_train") 
        plt.plot(self.index, self.acc_test, 'g', label="acc_test")
        plt.legend(loc=2)
        #Save and show.
        plt.savefig(self.figure_name)
        plt.show()

    def saveAccuracyAndLoss(self, index, acc_batch, acc_train, acc_test, loss_batch):
        self.index.append(index)
        self.acc_batch.append(acc_batch)
        self.acc_train.append(acc_train)
        self.acc_test.append(acc_test)
        self.loss_batch.append(loss_batch)

    def TestData(self, datasets_test):
        test_X = []; test_Y = []; labeled_list = []
        # Getting labeled list of all datasets.
        for dataset in datasets_test:
            path = os.path.join(self.path_candidates, dataset)
            labeled_list.append(self.Labels(path + "/labeling.csv", dataset))
        # Shuffle list.
        labeled_list = labeled_list[0]
        shuffle(labeled_list)
        # Getting training data.
        positive = 0; negative = 0;
        for data in labeled_list:
            path = self.path_candidates + data[2] + "/" + str(data[0]) + ".jpg"
            img = self.ImageFromPath(path)
            label = data[1]
            test_X.append(img)
            if label: 
                test_Y.append(np.array([1,0]))
                positive += 1
            else: 
                test_Y.append(np.array([0,1]))
                negative += 1
        print("Length of test data sets: %f with %f positive and %f negatives !" % (len(labeled_list),positive,negative))
        return test_X, test_Y

    def TrainingData(self, datasets):
        train_X = []; train_Y = []; labeled_list = []
        # Getting labeled list of all datasets.
        for dataset in datasets:
            path = os.path.join(self.path_candidates, dataset)
            labeled_list.extend(self.Labels(path + "/labeling.csv", dataset))
        # Shuffle list.
        shuffle(labeled_list)
        # Getting training data.
        positive = 0; negative = 0;
        for data in labeled_list:
            path = self.path_candidates + data[2] + "/" + str(data[0]) + ".jpg"
            img = self.ImageFromPath(path)
            label = data[1]
            train_X.append(img)
            if label: 
                train_Y.append(np.array([1,0]))
                positive += 1
            else: 
                train_Y.append(np.array([0,1]))
                negative += 1
        print("Length of training data sets: %f with %f positive and %f negatives !" % (len(labeled_list),positive,negative))
        return train_X, train_Y
