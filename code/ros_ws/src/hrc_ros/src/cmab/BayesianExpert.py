#!/usr/bin/env python3

import numpy as np
import pickle
bytes_read = open("covs.bin", "rb").read()
covs = pickle.loads(bytes_read)
bytes_read = open("means.bin", "rb").read()
means = pickle.loads(bytes_read)

bytes_read = open("humanDict.bin", "rb").read()
humanDict = pickle.loads(bytes_read)
bytes_read = open("humanDictInv.bin", "rb").read()
humanDictInv = pickle.loads(bytes_read)

bytes_read = open("robotDict.bin", "rb").read()
robotDict = pickle.loads(bytes_read)
bytes_read = open("robotDictInv.bin", "rb").read()
robotDictInv = pickle.loads(bytes_read)

def gaussian(x,mean,cov):
    """
    Calculates the gaussian density function
    @param x observation
    @param mean mean of gaussian
    @param cov of gaussian
    @return output of gaussian density function
    """
    d = cov.shape[0]
    cov = cov + np.eye(d)*0.0001
    x = np.reshape(x,(d,))
    mean = np.reshape(mean,(d,))
    diff = x-mean
    x = -0.5*diff.dot(np.linalg.inv(cov)).dot(diff)
    x = np.exp(x)
    y = 1/(np.sqrt(((2*np.pi)**d)*np.linalg.det(cov)))*x
    return y

def classify(x,mean=means,covs=covs):
    """
    Classify the observed bahaviour to certain human type
        Calculates the probability distribution for a given observation
        by calculating the gaussian value for each class.
    @param x observation
    @param means list of averages of each human class
    @param covs list of covariance matrices of each human class
    @return probability distribution of the human types
    """
    n = len(mean)
    p = np.array([gaussian(x,mean[i],covs[i])for i in range(n)])
    p = p/p.sum()
    return p
