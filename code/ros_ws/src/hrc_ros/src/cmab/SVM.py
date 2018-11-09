#!/usr/bin/env python3

import numpy as np
import pickle
#import scipy

from sklearn.model_selection import train_test_split
from sklearn import datasets
from sklearn import svm
from sklearn.model_selection import KFold, GridSearchCV, cross_val_score


bytes_read = open("svm.bin", "rb").read()
svm = pickle.loads(bytes_read)

def classify(x,svm=svm):
	"""
	Classify the human using a SVM
	@param x observation
	@param svm A SVM from sklearn.
	"""
	return svm.predict_proba(x)[0]
