#!/usr/bin/env python2.7

import json
import sys, os

if __name__=='__main__':
    dirr = os.path.dirname(os.path.realpath(__file__))
    with open(dirr + "/abps_savings.json") as file:
        jsonFile = json.load(file)
        print jsonFile
    file.close()
