#!/usr/bin/env python3

import pickle


bytes_read = open("covs.bin", "rb").read()
covs = pickle.loads(bytes_read)
filehandler = open("covs.bin", "wb")
pickle.dump(covs, filehandler, protocol = 2)

bytes_read = open("means.bin", "rb").read()
means = pickle.loads(bytes_read)
filehandler = open("means.bin", "wb")
pickle.dump(means, filehandler, protocol = 2)

bytes_read = open("humanDict.bin", "rb").read()
humanDict = pickle.loads(bytes_read)
filehandler = open("humanDict.bin", "wb")
pickle.dump(humanDict, filehandler, protocol = 2)

bytes_read = open("humanDictInv.bin", "rb").read()
humanDictInv = pickle.loads(bytes_read)
filehandler = open("humanDictInv.bin", "wb")
pickle.dump(humanDictInv, filehandler, protocol = 2)

bytes_read = open("robotDict.bin", "rb").read()
robotDict = pickle.loads(bytes_read)
filehandler = open("robotDict.bin", "wb")
pickle.dump(robotDict, filehandler, protocol = 2)

bytes_read = open("robotDictInv.bin", "rb").read()
robotDictInv = pickle.loads(bytes_read)
filehandler = open("robotDictInv.bin", "wb")
pickle.dump(robotDictInv, filehandler, protocol = 2)

bytes_read = open("lut.bin", "rb").read()
lut = pickle.loads(bytes_read)
filehandler = open("lut.bin", "wb")
pickle.dump(lut, filehandler, protocol = 2)

bytes_read = open("svm.bin", "rb").read()
svm = pickle.loads(bytes_read)
filehandler = open("svm.bin", "wb")
pickle.dump(svm, filehandler, protocol = 2)
